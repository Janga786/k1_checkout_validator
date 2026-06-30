#!/usr/bin/env python3
"""Arm-scan stage for the Nav2 + SLAM pipeline.

Waits for qr_goal's /nav_arrived (the robot has parked at the scan pose), then
captures the live /qr_detection (QR relative to base), runs the validated 4-DOF
IK to put the hand scanner on it, and "scans" it (beam -> QR turns green). The
robot base is driven by Nav2 / mini_sim (odom->base_link); this node only adds
the arm joint states + the QR / scanner / status markers.

  WAIT  -> REACH -> SCAN -> DONE
"""
import math
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import PointStamped, Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_geometry_msgs import do_transform_point

from k1_qr_nav.arm_kin import K1Arm, DEFAULT_URDF, hand_tip
from k1_qr_nav.arm_reach import quat_axis_to, CONE_DEG
from k1_qr_nav.safe_plan import plan_scan

TRUNK_H = 0.62
ITEM = "Iodized Salt"
# CORRECTED end-effector: the real hand is at the END of the Left_Arm_4 mesh
# (+0.22 m in the hand_link frame -- hand_link is the ELBOW-YAW joint, not the
# hand). Scanner mounts on the real hand; tip = hand + standoff along the read
# direction. (Gazebo physics caught the old elbow-as-hand bug.)
HAND_OFFSET = hand_tip("left")                       # mesh-derived (URDF source of truth, was 0.22)
SCANNER_AXIS = np.array([0.598, 0.762, -0.250])      # re-optimized mount for the real hand
SCANNER_AXIS = SCANNER_AXIS / np.linalg.norm(SCANNER_AXIS)
STANDOFF = 0.05                                       # scanner working distance (hand stands back)
TIP_XYZ = HAND_OFFSET + STANDOFF * SCANNER_AXIS


def ease(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


class ArmScan(Node):
    def __init__(self):
        super().__init__("arm_scan")
        self.declare_parameter("qr_x", 5.3)
        self.declare_parameter("qr_y", 1.4)
        self.declare_parameter("qr_height", 0.789)      # QR height above the floor (arm zone)
        self.declare_parameter("qr_box_cx", 0.0)        # box the QR is mounted on (0 = none)
        self.declare_parameter("qr_box_cy", 0.0)
        self.declare_parameter("qr_box_size", 0.0)
        self.declare_parameter("qr_box_h", 0.85)
        self.declare_parameter("ideal_fwd", 0.30)       # servo centres the QR here (real-hand reach)
        self.declare_parameter("ideal_left", 0.24)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("force_unsafe", False)   # demo: inject a body-folding target
        g = lambda k: self.get_parameter(k).value
        self.force_unsafe = g("force_unsafe")
        self.qr_world = (g("qr_x"), g("qr_y"), g("qr_height"))
        self.box = (g("qr_box_cx"), g("qr_box_cy"), g("qr_box_size"), g("qr_box_h"))
        self.ideal = (g("ideal_fwd"), g("ideal_left"))
        self.map_frame = g("map_frame")
        self.qr_trunk_z = g("qr_height") - TRUNK_H

        self.arm = K1Arm("left", tip_xyz=tuple(TIP_XYZ), scanner_axis=tuple(SCANNER_AXIS))
        self.arm_joints = [j.name for j in self.arm.joints]
        self.all_joints = [j.get("name") for j in ET.parse(DEFAULT_URDF).getroot().findall("joint")
                           if j.get("type") == "revolute"]
        self.scanner = self._build_scanner("left_hand_link")

        self.goal_q = np.zeros(self.arm.n)
        self.q_arm = np.zeros(self.arm.n)
        self.reach_time = 2.6
        self.plan = None
        self.phase = "WAIT"
        self.t_phase = 0.0
        self.scanned = False
        self.tries = 0
        self.servo_tries = 0
        self.dt = 1.0 / 30.0

        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "nav_arrived", self._arrived, latch)
        self.js = self.create_publisher(JointState, "joint_states", 10)
        self.mk = self.create_publisher(MarkerArray, "scan_markers", 10)
        self.cmd = self.create_publisher(Twist, "cmd_vel", 10)
        # command the PHYSICAL gz arm joints (bridged to the JointPositionControllers)
        self.jcmd = [self.create_publisher(Float64, f"arm/j{k}", 10) for k in range(self.arm.n)]
        self.create_timer(self.dt, self.tick)
        self.get_logger().info("arm_scan up | waiting for nav arrival...")

    def _arrived(self, msg):
        if msg.data and self.phase == "WAIT":
            self.phase, self.t_phase, self.servo_tries = "SERVO", 0.0, 0

    def _servo(self):
        """Terminal visual-servo: nudge the base (post-Nav2) so the QR lands at the
        ideal spot in the arm's zone. Closes the loop on the QR position (via TF,
        drift-free here). Returns True when centred or it gives up (then stops)."""
        try:
            tf = self.tfbuf.lookup_transform("Trunk", self.map_frame, rclpy.time.Time())
        except Exception:
            self.servo_tries += 1
            return self.servo_tries > 90            # no TF yet -> wait briefly, then give up
        qrp = PointStamped()
        qrp.header.frame_id = self.map_frame
        qrp.point.x, qrp.point.y, qrp.point.z = self.qr_world
        qt = do_transform_point(qrp, tf)
        e_fwd = qt.point.x - self.ideal[0]
        e_left = qt.point.y - self.ideal[1]
        self.servo_tries += 1
        if math.hypot(e_fwd, e_left) < 0.008 or self.servo_tries > 300:
            self.cmd.publish(Twist())               # centred (or timed out) -> stop
            return True
        cmd = Twist()
        cmd.linear.x = float(np.clip(1.2 * e_fwd, -0.15, 0.15))   # drive in/out to set the range
        cmd.angular.z = float(np.clip(2.5 * e_left, -0.5, 0.5))   # turn to centre it laterally
        self.cmd.publish(cmd)
        return False

    def _solve(self):
        """QR world pose -> arm (Trunk) frame via TF, then collision-checked IK. We
        use the remembered map position (the camera may not see the QR at the scan
        pose) and transform the REAL box into the arm frame for the keep-out."""
        qrp = PointStamped()
        qrp.header.frame_id = self.map_frame
        qrp.point.x, qrp.point.y, qrp.point.z = self.qr_world
        try:
            tf = self.tfbuf.lookup_transform("Trunk", self.map_frame, rclpy.time.Time())
        except Exception:
            self.tries += 1
            if self.tries < 60:
                return False
            tf = None
        if tf is not None:
            qt = do_transform_point(qrp, tf)
            target = np.array([qt.point.x, qt.point.y, qt.point.z])
        else:
            target = np.array([0.30, 0.24, self.qr_trunk_z])      # fallback: nominal scan pose
        # the box the QR is on, transformed into the arm frame -> keep-out behind the face
        box = None
        bcx, bcy, bsz, bh = self.box
        if bsz > 0.0 and tf is not None:
            bcp = PointStamped()
            bcp.header.frame_id = self.map_frame
            bcp.point.x, bcp.point.y, bcp.point.z = bcx, bcy, bh / 2.0
            bc = do_transform_point(bcp, tf)
            lo = np.array([bc.point.x - bsz / 2, bc.point.y - bsz / 2, bc.point.z - bh / 2])
            hi = np.array([bc.point.x + bsz / 2, bc.point.y + bsz / 2, bc.point.z + bh / 2])
            lo[0] = max(lo[0], target[0] + 0.01 + 0.03)       # keep-out behind face (+ capsule radius)
            box = (lo, hi)
        if self.force_unsafe:                            # demo: reach back-across into the chest
            target = np.array([-0.05, -0.10, 0.05])
            self.get_logger().warn("force_unsafe demo: injecting a body-folding target")
        # SAFETY GATE: full self-protection (body-avoiding IK + box + limits + full-trajectory
        # self-collision + velocity cap). Only move on a validated-safe plan; else refuse.
        self.plan = plan_scan(self.arm, target, box=box, link_radius=0.03)
        p = self.plan
        if p["safe"]:
            self.goal_q = p["q"]
            self.reach_time = max(0.5, p["duration_s"])
            self.get_logger().info(
                f"arrived -> QR in arm frame {np.round(target,3).tolist()} | reach {p['reach_mm']:.0f}mm "
                f"aim {p['aim_deg']:.0f}deg | SAFE plan: {p['duration_s']:.1f}s, peak {p['peak_vel']:.2f} rad/s "
                "-> reaching")
        else:
            self.get_logger().warn(
                f"arrived -> QR in arm frame {np.round(target,3).tolist()} | ARM MOVE REFUSED "
                f"(unsafe): {p['reasons']} -> arm stays tucked, NOT moving")
        return True

    def tick(self):
        if self.phase == "SERVO":
            if self._servo():                       # base nudged so the QR is centred
                if self._solve():                   # gate: REACH only if the plan is safe
                    self.phase, self.t_phase = ("REACH" if self.plan["safe"] else "REFUSED"), 0.0
        elif self.phase == "REACH":
            self.q_arm = ease(self.t_phase / self.reach_time) * self.goal_q
            self.t_phase += self.dt
            if self.t_phase >= self.reach_time:
                self.phase, self.t_phase, self.scanned = "SCAN", 0.0, True
                self.get_logger().info(f"SCANNED -> {ITEM}")
        elif self.phase == "SCAN":
            self.q_arm = self.goal_q
            self.t_phase += self.dt
            if self.t_phase >= 3.0:
                self.phase = "DONE"
        # WAIT / DONE: hold current q_arm
        self.publish()

    def publish(self):
        now = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = now
        amap = dict(zip(self.arm_joints, self.q_arm))
        js.name = self.all_joints
        js.position = [float(amap.get(n, 0.0)) for n in self.all_joints]
        self.js.publish(js)
        for k in range(self.arm.n):              # drive the physical gz arm joints
            self.jcmd[k].publish(Float64(data=float(self.q_arm[k])))
        self._markers(now)

    def _markers(self, stamp):
        arr = MarkerArray()
        qx, qy, qz = self.qr_world
        # QR panel + post (map frame), green once scanned
        qr = Marker()
        qr.header.frame_id = self.map_frame; qr.header.stamp = stamp
        qr.ns = "qr"; qr.id = 0; qr.type = Marker.CUBE; qr.action = Marker.ADD
        qr.pose.position = Point(x=float(qx), y=float(qy), z=float(qz)); qr.pose.orientation.w = 1.0
        qr.scale.x, qr.scale.y, qr.scale.z = 0.02, 0.12, 0.12
        c = (0.1, 0.9, 0.1, 1.0) if self.scanned else (0.1, 0.1, 0.1, 1.0)
        qr.color.r, qr.color.g, qr.color.b, qr.color.a = c
        arr.markers.append(qr)
        bcx, bcy, bsz, bh = self.box
        if bsz > 0.0:                                    # the box the QR is mounted on
            bx = Marker()
            bx.header.frame_id = self.map_frame; bx.header.stamp = stamp
            bx.ns = "box"; bx.id = 0; bx.type = Marker.CUBE; bx.action = Marker.ADD
            bx.pose.position = Point(x=float(bcx), y=float(bcy), z=float(bh / 2)); bx.pose.orientation.w = 1.0
            bx.scale.x, bx.scale.y, bx.scale.z = float(bsz), float(bsz), float(bh)
            bx.color.r, bx.color.g, bx.color.b, bx.color.a = 0.55, 0.45, 0.35, 0.7
            arr.markers.append(bx)
        else:
            post = Marker()
            post.header.frame_id = self.map_frame; post.header.stamp = stamp
            post.ns = "post"; post.id = 0; post.type = Marker.CYLINDER; post.action = Marker.ADD
            post.pose.position = Point(x=float(qx), y=float(qy), z=float(qz / 2)); post.pose.orientation.w = 1.0
            post.scale.x, post.scale.y, post.scale.z = 0.03, 0.03, float(qz)
            post.color.r, post.color.g, post.color.b, post.color.a = 0.4, 0.4, 0.4, 1.0
            arr.markers.append(post)
        # status text above the QR
        st = Marker()
        st.header.frame_id = self.map_frame; st.header.stamp = stamp
        st.ns = "status"; st.id = 0; st.type = Marker.TEXT_VIEW_FACING; st.action = Marker.ADD
        st.pose.position = Point(x=float(qx), y=float(qy), z=float(qz + 0.22)); st.pose.orientation.w = 1.0
        st.scale.z = 0.10
        msg = {"WAIT": "Navigating (Nav2 + SLAM)...", "SERVO": "Aligning to QR (visual servo)...",
               "REACH": "Reaching to scan...", "SCAN": f"SCANNED  v  {ITEM}",
               "DONE": f"SCANNED  v  {ITEM}",
               "REFUSED": "ARM MOVE REFUSED -- unsafe (safety gate)"}[self.phase]
        if self.phase == "REFUSED":
            col = (0.95, 0.2, 0.1, 1.0)                      # red
        else:
            col = (0.1, 0.9, 0.1, 1.0) if self.phase in ("SCAN", "DONE") else (0.95, 0.95, 0.95, 1.0)
        st.color.r, st.color.g, st.color.b, st.color.a = col
        st.text = msg
        arr.markers.append(st)
        # hand scanner
        for m in self.scanner:
            m.header.stamp = stamp
            arr.markers.append(m)
        self.mk.publish(arr)

    def _build_scanner(self, frame):
        ax = SCANNER_AXIS
        q_ax = quat_axis_to([1, 0, 0], ax)
        down = np.array([0.0, 0.0, -1.0])
        grip = down - np.dot(down, ax) * ax
        grip = grip / np.linalg.norm(grip) if np.linalg.norm(grip) > 1e-6 else np.array([0, 0, -1.0])
        q_grip = quat_axis_to([1, 0, 0], grip)
        gray = (0.20, 0.20, 0.24, 1.0)
        out = []

        def box(mid, q, sx, sy, sz, color, ns):
            m = Marker()
            m.ns = ns; m.id = 0; m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position = Point(x=float(mid[0]), y=float(mid[1]), z=float(mid[2]))
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
            m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
            m.color.r, m.color.g, m.color.b, m.color.a = color
            out.append(m)

        box(HAND_OFFSET + 0.018 * ax, q_ax, 0.036, 0.046, 0.030, gray, "scan_body")
        box(HAND_OFFSET + 0.018 * ax + 0.030 * grip, q_grip, 0.045, 0.020, 0.022, gray, "scan_grip")
        lens = Marker()
        lens.ns = "scan_lens"; lens.id = 0; lens.type = Marker.CYLINDER; lens.action = Marker.ADD
        p = HAND_OFFSET + 0.037 * ax; lq = quat_axis_to([0, 0, 1], ax)
        lens.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        lens.pose.orientation.x, lens.pose.orientation.y, lens.pose.orientation.z, lens.pose.orientation.w = lq
        lens.scale.x, lens.scale.y, lens.scale.z = 0.026, 0.026, 0.006
        lens.color.r, lens.color.g, lens.color.b, lens.color.a = 0.05, 0.05, 0.05, 1.0
        out.append(lens)
        beam = Marker()
        beam.ns = "scan_beam"; beam.id = 0; beam.type = Marker.ARROW; beam.action = Marker.ADD
        s, e = HAND_OFFSET + 0.030 * ax, HAND_OFFSET + STANDOFF * ax   # beam stops at the read point
        beam.points = [Point(x=float(s[0]), y=float(s[1]), z=float(s[2])),
                       Point(x=float(e[0]), y=float(e[1]), z=float(e[2]))]
        beam.scale.x, beam.scale.y, beam.scale.z = 0.006, 0.014, 0.02
        beam.color.r, beam.color.g, beam.color.b, beam.color.a = 0.95, 0.15, 0.10, 0.9
        out.append(beam)
        for m in out:
            m.header.frame_id = frame
        return out


def main():
    rclpy.init()
    node = ArmScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
