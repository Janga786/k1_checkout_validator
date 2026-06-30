#!/usr/bin/env python3
"""Booster K1 <-> ROS2 bridge: the real-robot driver for the barcode-seek nav stack.

Turns Nav2's velocity command into the robot's WALK and a head-aim command into the
robot's HEAD-TURN, and turns the robot's leg odometry into a ROS /odom that SLAM, Nav2
and qr_goal consume. ONE node, two modes:

  real      drives the actual K1 over DDS via booster_robotics_sdk_python:
            ChangeMode kPrepare->kWalking (gated), Move(vx,vy,vyaw), RotateHead(pitch,yaw),
            B1OdometerStateSubscriber -> /odom.
  --dry-run NO robot, NO SDK: logs every command and integrates /cmd_vel into a
            simulated /odom so the whole SLAM/Nav2/seek graph runs on the workstation.

The Booster SDK does NO reachability/limit checking, so THIS node is the safety authority.
It runs an explicit mode state machine and DOES NOTHING TO THE ROBOT until an operator
sends /start_walking -- launching the stack does not, by itself, make the robot move.

  IDLE --/start_walking--> PREPARING --(settle)--> WALKING --/cmd_vel--> walk
  WALKING --/soft_stop--> HALTED  (Move 0, stays in kWalking, KEEPS balancing) --/start_walking--> WALKING
  any --/estop--> ESTOP   (Move 0 + kPrepare -- stops & STANDS, balancing; NEVER limp)
  any --/damp---> DAMPED  (Move 0 + kDamping = LIMP/sag -- ONLY when held/hoisted)

STOPS NEVER DROP THE ROBOT. kDamping makes a biped go limp and fall, so it is NOT the
emergency stop -- /estop halts the gait (Move 0, the controller balances in place) and
settles to the kPrepare standing stance. kDamping is reachable only via the explicit /damp,
for when the robot is already held/hoisted. Shutdown leaves it standing in kPrepare too.

Safety layers (all here, none in the SDK):
  * gated start          -- no walking until /start_walking (auto only when auto_walk:=true)
  * return-code checks   -- ChangeMode/Move failures abort the transition / are surfaced
  * graceful stops only  -- /soft_stop pauses (stays kWalking); /estop stops+kPrepare (stands);
                            both keep BALANCING. kDamping (limp) only via explicit /damp.
  * low default speeds   -- max_vx 0.20 m/s etc.; raise via params once verified on hardware
  * travel geofence      -- auto soft-stop after max_travel m (runaway / bad-localization guard)
  * command watchdog     -- zero velocity if /cmd_vel goes stale

ROS in :  /cmd_vel (Twist) -> Move      /head_cmd (Float64MultiArray[pitch,yaw]) -> RotateHead
          /start_walking (Bool) -> prepare+walk    /soft_stop (Bool) -> halt, stay balancing
          /estop (Bool) -> stop + kPrepare (stand) /damp (Bool) -> kDamping (limp; held only)
ROS out:  /odom (Odometry)   /booster_state (String, latched)

Run under /usr/bin/python3 with ROS2 Humble sourced and PYTHONPATH=<sdk>/build.
"""
import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray, String

IDLE, PREPARING, WALKING, HALTED, ESTOP, DAMPED = \
    "IDLE", "PREPARING", "WALKING", "HALTED", "ESTOP", "DAMPED"


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# === ABSOLUTE HARD LIMITS — the robot's PUBLISHED limits minus a safety margin ===
# Sources: head joints from K1_22dof.xml  -> pitch [-0.349, 0.855] rad, yaw [-1.0, 1.0] rad;
#          base-velocity envelope from the official booster_deploy K1 walk cfg -> |v| <= 1.0.
# These cap BOTH the user params AND every single SDK call (the chokepoints _sdk_move /
# _sdk_rotate_head). Nothing in this node -- no param misconfig, planner glitch, or code
# path -- can command the robot outside these. The margins keep clearance from the
# mechanical end-stops and the controller envelope so we never even touch the limit.
HARD = {
    "vx":    (-0.80, 0.80),    # m/s    (envelope ±1.0 - 0.20 gap)
    "vy":    (-0.80, 0.80),    # m/s
    "vyaw":  (-0.80, 0.80),    # rad/s
    "pitch": (-0.30, 0.80),    # rad    (head_pitch joint [-0.349, 0.855] - ~0.05 gap)
    "yaw":   (-0.90, 0.90),    # rad    (head_yaw   joint [-1.0,   1.0]   - 0.10 gap)
}


class BoosterBridge(Node):
    def __init__(self, dry_run, net_iface):
        super().__init__("booster_bridge")
        self.dry_run = dry_run
        self.net_iface = net_iface

        # --- safety / motion params (LOW first-run defaults; raise once verified on hardware) ---
        self.declare_parameter("max_vx", 0.20)        # m/s forward
        self.declare_parameter("max_vx_back", 0.10)   # m/s backward (it can't see behind)
        self.declare_parameter("max_vy", 0.10)        # m/s lateral
        self.declare_parameter("max_vyaw", 0.30)      # rad/s turn
        self.declare_parameter("cmd_timeout", 0.5)    # s: zero velocity if /cmd_vel goes stale
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("odom_hz", 50.0)
        self.declare_parameter("auto_walk", False)    # real: require /start_walking; demos override
        self.declare_parameter("prepare_settle", 2.0) # s for kPrepare to settle before kWalking
        self.declare_parameter("max_travel", 5.0)     # m from start before auto soft-stop (0 = off)
        self.declare_parameter("head_pitch_min", -0.30)   # inside head_pitch joint range [-0.349, 0.855]
        self.declare_parameter("head_pitch_max", 0.80)
        self.declare_parameter("head_yaw_limit", 0.90)    # inside head_yaw joint range [-1.0, 1.0]
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        gp = lambda n: self.get_parameter(n).value
        # operational caps -- but NEVER allowed above the absolute HARD envelope, even if a
        # param is set higher (so a misconfig can't widen the safe range)
        self.max_vx = min(gp("max_vx"), HARD["vx"][1])
        self.max_vx_back = min(gp("max_vx_back"), -HARD["vx"][0])
        self.max_vy = min(gp("max_vy"), HARD["vy"][1])
        self.max_vyaw = min(gp("max_vyaw"), HARD["vyaw"][1])
        self.cmd_timeout = gp("cmd_timeout")
        self.control_hz, self.odom_hz = gp("control_hz"), gp("odom_hz")
        self.auto_walk = bool(gp("auto_walk"))   # gated by default; the launch enables it for demos
        self.prepare_settle = gp("prepare_settle")
        self.max_travel = gp("max_travel")
        # head caps -- clamped into the absolute HARD joint range (can be tighter, never wider)
        self.hp_min = max(gp("head_pitch_min"), HARD["pitch"][0])
        self.hp_max = min(gp("head_pitch_max"), HARD["pitch"][1])
        self.hy_lim = min(gp("head_yaw_limit"), HARD["yaw"][1])
        self.odom_frame, self.base_frame = gp("odom_frame"), gp("base_frame")

        # --- state ---
        self._lock = threading.Lock()
        self.vx = self.vy = self.vyaw = 0.0
        self.last_cmd_t = self.get_clock().now()
        self.state = IDLE
        self.ox = self.oy = self.otheta = 0.0
        self.geo_origin = None
        self.head_pitch = self.head_yaw = 0.0
        self.client = None
        self._RobotMode = None
        self._prep_timer = None
        self._move_fail = 0
        self._stop_health = threading.Event()

        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.state_pub = self.create_publisher(String, "booster_state", latch)
        self.create_subscription(Twist, "cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Float64MultiArray, "head_cmd", self._head_cb, 10)
        self.create_subscription(Bool, "start_walking", self._start_cb, latch)
        self.create_subscription(Bool, "soft_stop", self._soft_stop_cb, latch)
        self.create_subscription(Bool, "estop", self._estop_cb, latch)
        self.create_subscription(Bool, "damp", self._damp_cb, latch)

        if self.dry_run:
            self.get_logger().warn("DRY-RUN: no robot. Logging commands; /odom integrated from /cmd_vel.")
        else:
            self._init_sdk()

        self.create_timer(1.0 / self.control_hz, self._control_tick)
        self.create_timer(1.0 / self.odom_hz, self._publish_odom)
        self._set_state(IDLE)
        self.get_logger().info(
            f"booster_bridge up | mode={'DRY-RUN' if self.dry_run else 'REAL'} "
            f"| OP clamps vx[{-self.max_vx_back:.2f},{self.max_vx:.2f}] vy±{self.max_vy:.2f} "
            f"vyaw±{self.max_vyaw:.2f} head_pitch[{self.hp_min:.2f},{self.hp_max:.2f}] head_yaw±{self.hy_lim:.2f}")
        self.get_logger().info(
            f"booster_bridge HARD ceilings (cannot be exceeded): vx/vy/vyaw±{HARD['vx'][1]:.2f} "
            f"pitch[{HARD['pitch'][0]:.2f},{HARD['pitch'][1]:.2f}] yaw±{HARD['yaw'][1]:.2f} "
            f"| watchdog {self.cmd_timeout:.1f}s | geofence {self.max_travel:.1f}m | mode-fault watcher ON")
        if self.auto_walk:
            self.get_logger().info("auto_walk -> starting (demo / auto_walk:=true)")
            self._request_start()
        else:
            self.get_logger().warn(
                "IDLE: robot will NOT move until you send  /start_walking  "
                "(ros2 topic pub --once /start_walking std_msgs/msg/Bool '{data: true}')")

    # ---- SDK ----
    def _init_sdk(self):
        try:
            from booster_robotics_sdk_python import (
                B1LocoClient, ChannelFactory, RobotMode, B1OdometerStateSubscriber)
        except Exception as e:
            self.get_logger().error(
                f"booster_robotics_sdk_python import failed ({e}). "
                f"Source ROS2 + set PYTHONPATH=<sdk>/build, or run with --dry-run.")
            raise
        self._RobotMode = RobotMode
        ChannelFactory.Instance().Init(0, self.net_iface)
        self.client = B1LocoClient()
        self.client.Init()
        self.odom_sub = B1OdometerStateSubscriber(self._sdk_odom_cb)
        self.odom_sub.InitChannel()
        # background mode-fault watcher: latches E-STOP if the robot leaves kWalking on its own
        threading.Thread(target=self._health_loop, name="mode-health", daemon=True).start()
        self.get_logger().info(f"SDK up on iface '{self.net_iface or '(default)'}' (robot NOT yet commanded)")

    def _change_mode(self, mode_enum, name):
        """ChangeMode with a return-code check. True on success."""
        if self.dry_run or self.client is None:
            self.get_logger().info(f"[dry] ChangeMode({name})")
            return True
        res = self.client.ChangeMode(mode_enum)
        if res != 0:
            self.get_logger().error(f"ChangeMode({name}) FAILED (rc={res})")
            return False
        self.get_logger().info(f"ChangeMode({name}) ok")
        return True

    def _move0(self):
        """Immediately command zero velocity (the controller balances in place)."""
        with self._lock:
            self.vx = self.vy = self.vyaw = 0.0
        self._sdk_move(0.0, 0.0, 0.0)

    # ---- the ONLY two places robot motion is commanded: hard-clamped chokepoints ----
    def _sdk_move(self, vx, vy, vyaw):
        """Every Move() passes through here. Final hard-clamp to the absolute velocity
        envelope -- after this, what reaches the SDK is provably within HARD."""
        vx = _clamp(vx, *HARD["vx"])
        vy = _clamp(vy, *HARD["vy"])
        vyaw = _clamp(vyaw, *HARD["vyaw"])
        if self.dry_run or self.client is None:
            return 0
        return self.client.Move(vx, vy, vyaw)

    def _sdk_rotate_head(self, pitch, yaw):
        """Every RotateHead() passes through here. Final hard-clamp to the head joint range."""
        pitch = _clamp(pitch, *HARD["pitch"])
        yaw = _clamp(yaw, *HARD["yaw"])
        if self.dry_run or self.client is None:
            return 0
        return self.client.RotateHead(pitch, yaw)

    def _health_loop(self):
        """Background watch (off the control thread): while we expect WALKING, poll the robot's
        actual mode. If it leaves kWalking on its own (a fall/PROTECT, OR the operator deliberately
        damped it), HALT the bridge's drive commands and log it -- but do NOT change the robot's
        mode. The bridge NEVER autonomously stands a limp robot up: doing so once overrode a
        deliberate damp and stood the robot up unexpectedly (2026-06-30). Recovery is the operator's
        explicit call (/start_walking). Tolerant of transient comms (needs 3 consecutive misses)."""
        try:
            from booster_robotics_sdk_python import GetModeResponse
        except Exception:
            return
        miss = 0
        while not self._stop_health.is_set():
            time.sleep(0.5)
            if self.client is None or self.state != WALKING:
                miss = 0
                continue
            try:
                resp = GetModeResponse()
                rc = self.client.GetMode(resp)
            except Exception:
                continue
            if rc != 0:
                continue
            if self._RobotMode is not None and resp.mode != self._RobotMode.kWalking:
                miss += 1
                if miss >= 3:                              # ~1.5 s disagreement -> real fault / a deliberate damp
                    self.get_logger().error(
                        f"ROBOT-MODE FAULT: expected kWalking, robot reports {resp.mode}. "
                        "Halting drive commands. NOT changing the robot's mode -- a fall or a deliberate "
                        "damp is left AS-IS for the operator (the bridge never autonomously stands the robot up).")
                    with self._lock:
                        self.vx = self.vy = self.vyaw = 0.0    # stop intending motion (control loop won't send Move)
                    self._set_state(ESTOP)                     # leaves the robot in its ACTUAL mode; NO ChangeMode
                    miss = 0
            else:
                miss = 0

    def _robot_is_damping(self):
        """Read the robot's ACTUAL mode (read-only GetMode); True if it's kDamping (limp). Used so
        the stop paths NEVER auto-stand a deliberately-damped robot. False if it can't be read."""
        if self.dry_run or self.client is None or self._RobotMode is None:
            return False
        try:
            from booster_robotics_sdk_python import GetModeResponse
            resp = GetModeResponse()
            if self.client.GetMode(resp) == 0:
                return resp.mode == self._RobotMode.kDamping
        except Exception:
            pass
        return False

    def _sdk_odom_cb(self, msg):
        with self._lock:
            self.ox, self.oy, self.otheta = float(msg.x), float(msg.y), float(msg.theta)

    # ---- start / stop transitions ----
    def _request_start(self):
        if self.state in (PREPARING, WALKING):
            return
        if self.state == HALTED:                      # already balancing in kWalking -> just resume
            self._begin_walking(resume=True)
            return
        # IDLE / ESTOP (already in kPrepare) / DAMPED -> full kPrepare -> kWalking
        if not self._change_mode(self._RobotMode.kPrepare if self._RobotMode else None, "kPrepare"):
            self.get_logger().error("kPrepare failed -> not walking (robot left as-is)")
            return
        self._set_state(PREPARING)
        if self._prep_timer is not None:
            self._prep_timer.cancel()
        self._prep_timer = self.create_timer(self.prepare_settle, self._begin_walking)

    def _begin_walking(self, resume=False):
        if self._prep_timer is not None:
            self._prep_timer.cancel(); self._prep_timer = None
        if self.state not in (PREPARING, HALTED):
            return
        if not self._change_mode(self._RobotMode.kWalking if self._RobotMode else None, "kWalking"):
            self.get_logger().error("kWalking failed -> HALTED")
            self._set_state(HALTED); return
        with self._lock:
            self.geo_origin = (self.ox, self.oy)      # geofence measured from here
            self.vx = self.vy = self.vyaw = 0.0
        self._set_state(WALKING)

    def _soft_stop_cb(self, msg):
        """Graceful pause: stop moving but stay in kWalking (balancing in place). Resumable."""
        if not msg.data or self.state in (ESTOP, DAMPED):
            return
        self._move0()
        self._set_state(HALTED)
        self.get_logger().warn("SOFT-STOP: halted, still balancing (kWalking). /start_walking to resume.")

    def _estop_cb(self, msg):
        """EMERGENCY stop -- Move(0,0,0) halts the gait in place (the controller balances), then
        kPrepare settles to the stable standing stance -- UNLESS the robot is already deliberately
        damped, in which case it is left limp (a deliberate damp is NEVER auto-overridden)."""
        if not msg.data:
            return
        self._move0()                                 # immediate: stop; controller balances in place
        if self._prep_timer is not None:
            self._prep_timer.cancel(); self._prep_timer = None
        # Stand to kPrepare ONLY if not already deliberately damped -- nav_cleanup auto-publishes /estop,
        # so it must never stand up a robot you've damped.
        if self._robot_is_damping():
            self.get_logger().error(
                "E-STOP requested but robot is DAMPED (limp) -- leaving it damped, NOT standing it up. "
                "Send /start_walking to deliberately re-stand.")
        else:
            self._change_mode(self._RobotMode.kPrepare if self._RobotMode else None, "kPrepare")
            self.get_logger().error(
                "E-STOP: Move(0,0,0) + kPrepare -- stopped & STANDING, balancing (NOT limp). /start_walking to resume.")
        self._set_state(ESTOP)

    def _damp_cb(self, msg):
        """Explicit kDamping -- the robot goes LIMP and will sag/sit. ONLY when it is already
        held, hoisted, or you deliberately want it compliant. This is NOT the emergency stop."""
        if not msg.data:
            return
        self._move0()
        if self._prep_timer is not None:
            self._prep_timer.cancel(); self._prep_timer = None
        self._change_mode(self._RobotMode.kDamping if self._RobotMode else None, "kDamping")
        self._set_state(DAMPED)
        self.get_logger().error("DAMPING: kDamping -- robot is LIMP and will sag. Use only when held/hoisted.")

    def _start_cb(self, msg):
        if msg.data:
            self._request_start()

    def _set_state(self, s):
        self.state = s
        self.state_pub.publish(String(data=s))

    # ---- ROS callbacks ----
    def _cmd_cb(self, msg):
        vx = _clamp(msg.linear.x, -self.max_vx_back, self.max_vx)
        vy = _clamp(msg.linear.y, -self.max_vy, self.max_vy)
        vyaw = _clamp(msg.angular.z, -self.max_vyaw, self.max_vyaw)
        with self._lock:
            self.vx, self.vy, self.vyaw = vx, vy, vyaw
            self.last_cmd_t = self.get_clock().now()

    def _head_cb(self, msg):
        if len(msg.data) < 2 or self.state in (ESTOP, DAMPED):
            return
        pitch = _clamp(float(msg.data[0]), self.hp_min, self.hp_max)
        yaw = _clamp(float(msg.data[1]), -self.hy_lim, self.hy_lim)
        if abs(pitch - self.head_pitch) < 1e-3 and abs(yaw - self.head_yaw) < 1e-3:
            return
        self.head_pitch, self.head_yaw = pitch, yaw
        if self.dry_run:
            self.get_logger().info(f"[dry] RotateHead(pitch={pitch:+.2f}, yaw={yaw:+.2f})")
        else:
            self._sdk_rotate_head(pitch, yaw)            # hard-clamped to the head joint range

    # ---- control loop ----
    def _control_tick(self):
        now = self.get_clock().now()
        with self._lock:
            stale = (now - self.last_cmd_t).nanoseconds * 1e-9 > self.cmd_timeout
            walk = self.state == WALKING
            vx, vy, vyaw = (self.vx, self.vy, self.vyaw) if (walk and not stale) else (0.0, 0.0, 0.0)
            ox, oy, origin = self.ox, self.oy, self.geo_origin

        # geofence: stop a runaway / bad-localization walk (graceful, stays balancing)
        if walk and self.max_travel > 0 and origin is not None:
            if math.hypot(ox - origin[0], oy - origin[1]) > self.max_travel:
                self.get_logger().error(
                    f"GEOFENCE: traveled > {self.max_travel:.1f} m from start -> SOFT-STOP")
                self._move0()
                self._set_state(HALTED)
                return

        if self.dry_run:
            if self.state in (WALKING, HALTED):
                self._integrate(vx, vy, vyaw)
            return
        if self.client is None or self.state not in (WALKING, HALTED):
            return                                     # IDLE/PREPARING/ESTOP/DAMPED: don't command Move
        res = self._sdk_move(vx, vy, vyaw)             # hard-clamped; HALTED -> Move(0,0,0): stop, keep balancing
        if res != 0:
            self._move_fail += 1
            if self._move_fail in (1, 20, 100):        # surface, but don't spam at 20 Hz
                self.get_logger().error(f"Move() rc={res} (x{self._move_fail}) -- comms/mode problem")
        else:
            self._move_fail = 0

    def _integrate(self, vx, vy, vyaw):
        dt = 1.0 / self.control_hz
        with self._lock:
            c, s = math.cos(self.otheta), math.sin(self.otheta)
            self.ox += (vx * c - vy * s) * dt
            self.oy += (vx * s + vy * c) * dt
            self.otheta = math.atan2(math.sin(self.otheta + vyaw * dt),
                                     math.cos(self.otheta + vyaw * dt))

    def _publish_odom(self):
        with self._lock:
            x, y, th = self.ox, self.oy, self.otheta
            vx, vy, vyaw = self.vx, self.vy, self.vyaw
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        qx, qy, qz, qw = _yaw_to_quat(th)
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = qx, qy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.linear.y = vx, vy
        o.twist.twist.angular.z = vyaw
        self.odom_pub.publish(o)

    # ---- shutdown ----
    def safe_stop(self):
        """On exit, stop driving and settle a WALKING robot to a standing stance (kPrepare) so it
        doesn't fall -- but NEVER stand up a deliberately-damped robot (a damp is left as-is)."""
        if self.dry_run or self.client is None:
            return
        self._stop_health.set()
        try:
            self._sdk_move(0.0, 0.0, 0.0)
            if self.state in (WALKING, HALTED, PREPARING) and not self._robot_is_damping():
                self._change_mode(self._RobotMode.kPrepare, "kPrepare")
                self.get_logger().info("Move(0,0,0) + kPrepare on shutdown (left standing, balancing)")
            else:
                self.get_logger().info("Move(0,0,0) on shutdown; left robot in its current mode "
                                       "(damped/halted -- NOT auto-stood-up)")
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true",
                    help="no robot/SDK: log commands, integrate /cmd_vel into /odom")
    ap.add_argument("--net", default="", help="DDS network interface (e.g. eth0 / 192.168.10.x)")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = BoosterBridge(dry_run=args.dry_run, net_iface=args.net)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_stop()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
