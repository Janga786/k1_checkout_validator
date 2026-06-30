#!/usr/bin/env python3
"""Odom-frame cart tracker -- APPROACH (and ORBIT) a cart using the FAST leg odometry, robust to
this robot's slow (~0.4 fps) camera *content* rate.

WHY (measured 2026-06-30): the head camera reports 10 fps but its actual image only changes ~0.4 fps
when the robot is idle/standing. Servoing on the instantaneous detection (cart_approach) therefore
stutters and walks on ~2 s-old headings -> it drifts into things. Here the camera is used ONLY to
PIN the cart's position in the /odom frame (fused over several AGREEING detections, outlier-rejected),
and the robot drives toward that pinned point on /odom (~400 Hz, always fresh). Benefits:
  * control is smooth + decoupled from the camera rate
  * the cart stays "remembered" even between camera updates / when it briefly leaves view
  * it will NOT move until it has a confident lock (>=establish_n agreeing fixes) -> a jumping
    detector can't make it lurch at the wrong object (the "walking into things" fix)
  * a circular ORBIT around the pinned point is then trivial (your "walk around it" goal)

  cart_locate   /cart_detection (PointStamped optical: x=right,y=down,z=forward) + /cart_bearing([u,score])
  booster_bridge  /odom (robot pose in the odom frame)
        v
  cart_track    pin cart in odom -> APPROACH to standoff -> (ORBIT) -> /cmd_vel + /head_cmd
        v
  booster_bridge  GATES (/start_walking) + hard-clamps -> Move

Reuses the proven approach_cmd law (cart_approach_law), fed ODOM-derived bearing/range. SAFE: only
publishes /cmd_vel; the bridge still gates + clamps. NO obstacle avoidance -- it drives straight to
the pinned point, so give it a clear path to the cart.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray

from k1_qr_nav.cart_approach_law import approach_cmd, _clamp


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CartTrack(Node):
    def __init__(self):
        super().__init__("cart_track")
        d = self.declare_parameter
        d("standoff", 1.0)
        d("arrive_tol", 0.12)
        d("min_approach", 0.06)
        d("vx_cap", 0.18)
        d("vyaw_cap", 0.30)
        d("k_fwd", 0.35)
        d("k_yaw", 1.2)
        d("yaw_sign", -1.0)          # flip if it turns the wrong way toward the cart
        d("center_thresh", 0.20)
        d("head_pitch", 0.20)
        d("control_hz", 10.0)
        d("min_score", 0.15)         # detection score floor for a localization fix
        d("establish_n", 3)          # AGREEING fixes needed to lock the cart (rejects a jumping detector)
        d("establish_spread", 1.0)   # m: the establishing fixes must agree within this (monocular range is noisy)
        d("reject_dist", 1.5)        # m: once locked, ignore a fix farther than this from the estimate
        d("ema_alpha", 0.3)          # fuse weight for accepted fixes
        d("lock_timeout", 8.0)       # s without a fix while LOCALIZING -> drop pending fixes (re-agree)
        d("orbit", False)            # after arriving, walk a circle around the cart (phase 2)
        d("orbit_dir", 1.0)          # +1 / -1
        d("orbit_speed", 0.10)       # m/s tangential (<= bridge max_vy)
        d("lost_timeout", 2.0)       # s without a FRESH detection while moving -> STOP (the odom pin alone drifts)
        d("turn_limit", 3.5)         # rad: cumulative turn with NO forward progress -> STOP (spin guard)
        d("cam_fresh", 0.5)          # s: use the LIVE camera bearing/range (drift-free) when a fix is this recent
        g = lambda n: self.get_parameter(n).value
        self.standoff = g("standoff"); self.arrive_tol = g("arrive_tol"); self.min_approach = g("min_approach")
        self.vx_cap = g("vx_cap"); self.vyaw_cap = g("vyaw_cap"); self.k_fwd = g("k_fwd"); self.k_yaw = g("k_yaw")
        self.yaw_sign = g("yaw_sign"); self.center_thresh = g("center_thresh"); self.head_pitch = g("head_pitch")
        self.control_hz = g("control_hz")        # stored (the spin guard uses dt = 1/control_hz)
        self.min_score = g("min_score"); self.establish_n = int(g("establish_n"))
        self.establish_spread = g("establish_spread"); self.reject_dist = g("reject_dist"); self.alpha = g("ema_alpha")
        self.lock_timeout = g("lock_timeout")
        self.orbit = g("orbit"); self.orbit_dir = g("orbit_dir"); self.orbit_speed = g("orbit_speed")
        self.lost_timeout = g("lost_timeout"); self.turn_limit = g("turn_limit"); self.cam_fresh = g("cam_fresh")

        self.have_odom = False; self.rx = self.ry = self.rth = 0.0     # robot pose in odom
        self.cart = None                                              # (x,y) in odom once locked
        self._fixes = []                                             # pending fixes [(x,y,t)] before lock
        self.last_score = 1.0; self.last_score_t = None
        self.cam_bear = None; self.cam_rng = None; self.last_fix_t = None   # LIVE camera bearing/range (drift-free)
        self._turn_acc = 0.0                                                # cumulative turn since last forward step
        self.state = "LOCALIZING"                                    # LOCALIZING -> APPROACH -> ARRIVED/ORBIT

        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.head_pub = self.create_publisher(Float64MultiArray, "head_cmd", 10)
        self.reached_pub = self.create_publisher(Bool, "cart_reached", latch)
        self.create_subscription(Odometry, "odom", self._odom_cb, 20)
        self.create_subscription(PointStamped, "cart_detection", self._det_cb, 10)
        self.create_subscription(Float64MultiArray, "cart_bearing", self._bear_cb, 10)
        self.create_timer(1.0 / g("control_hz"), self._tick)
        self.get_logger().info(
            f"cart_track up | standoff={self.standoff:.2f} m | lock={self.establish_n} agreeing fixes | "
            f"LIVE-camera-preferred control (drift-free), odom pin bridges gaps | lost>{self.lost_timeout:.1f}s->STOP | "
            f"spin>{self.turn_limit:.1f}rad->STOP | orbit={'ON' if self.orbit else 'off'} | won't move until locked")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_cb(self, m):
        self.rx = m.pose.pose.position.x; self.ry = m.pose.pose.position.y
        self.rth = _yaw_from_quat(m.pose.pose.orientation); self.have_odom = True

    def _bear_cb(self, m):
        if len(m.data) >= 2:
            self.last_score = float(m.data[1]); self.last_score_t = self._now()

    def _det_to_odom(self, msg):
        """Cart camera-relative (forward, right) -> a point in the odom frame, using the robot's
        current odom pose. Frame-aware: optical (x=right,z=forward) or base_link (x=forward,y=left)."""
        fr = msg.header.frame_id or ""
        if "optical" in fr:
            forward, right = msg.point.z, msg.point.x
        else:
            forward, right = msg.point.x, -msg.point.y
        if forward <= 0.0:
            return None
        cx = self.rx + forward * math.cos(self.rth) + right * math.sin(self.rth)
        cy = self.ry + forward * math.sin(self.rth) - right * math.cos(self.rth)
        return cx, cy

    def _det_cb(self, msg):
        if not self.have_odom:
            return
        sfresh = self.last_score_t is not None and (self._now() - self.last_score_t) < 0.7
        if sfresh and self.last_score < self.min_score:
            return
        fr = msg.header.frame_id or ""
        if "optical" in fr:
            forward, right = msg.point.z, msg.point.x
        else:
            forward, right = msg.point.x, -msg.point.y
        if forward <= 0.0:
            return
        t = self._now(); a = self.alpha
        # LIVE (drift-free) camera bearing/range, EMA-smoothed -> the PREFERRED control signal
        cb = math.atan2(right, forward); crng = math.hypot(right, forward)
        self.cam_bear = cb if self.cam_bear is None else (1 - a) * self.cam_bear + a * cb
        self.cam_rng = crng if self.cam_rng is None else (1 - a) * self.cam_rng + a * crng
        self.last_fix_t = t
        # pin/refine the cart in the odom frame (used ONLY to bridge brief camera gaps)
        cx = self.rx + forward * math.cos(self.rth) + right * math.sin(self.rth)
        cy = self.ry + forward * math.sin(self.rth) - right * math.cos(self.rth)
        if self.cart is None:
            self._fixes = [(x, y, tt) for (x, y, tt) in self._fixes if t - tt < self.lock_timeout]
            self._fixes.append((cx, cy, t))
            recent = self._fixes[-self.establish_n:]
            if len(recent) >= self.establish_n:
                mx = sum(x for x, _, _ in recent) / len(recent)
                my = sum(y for _, y, _ in recent) / len(recent)
                spread = max(math.hypot(x - mx, y - my) for x, y, _ in recent)
                if spread <= self.establish_spread:
                    self.cart = (mx, my); self.state = "APPROACH"; self._fixes = []
                    self.get_logger().info(
                        f"cart LOCKED in odom at ({mx:+.2f},{my:+.2f}) from {len(recent)} agreeing fixes "
                        f"(spread {spread:.2f} m) -> APPROACH")
        else:
            if math.hypot(cx - self.cart[0], cy - self.cart[1]) > self.reject_dist:
                return                                          # outlier (chair/edge) -> ignore
            self.cart = ((1 - a) * self.cart[0] + a * cx, (1 - a) * self.cart[1] + a * cy)

    def _cart_rel(self):
        dx = self.cart[0] - self.rx; dy = self.cart[1] - self.ry
        forward = dx * math.cos(self.rth) + dy * math.sin(self.rth)
        right = dx * math.sin(self.rth) - dy * math.cos(self.rth)
        return forward, right, math.atan2(right, forward), math.hypot(dx, dy)

    def _tick(self):
        self.head_pub.publish(Float64MultiArray(data=[float(self.head_pitch), 0.0]))   # head forward
        cmd = Twist()
        if self.cart is None or not self.have_odom:
            self.get_logger().info(
                f"localizing cart ({len(self._fixes)}/{self.establish_n} agreeing fixes) -> HOLD (no motion)",
                throttle_duration_sec=2.0)
            self.cmd_pub.publish(cmd); return

        if self.state == "ARRIVED":
            self.cmd_pub.publish(cmd); return

        now = self._now()
        _of, _ori, o_bear, o_rng = self._cart_rel()                # odom-pin bearing/range (drifts)
        age = (now - self.last_fix_t) if self.last_fix_t is not None else 1e9

        # LOST-TARGET STOP: driving on the odom pin alone past this drifts (humanoid heading drift)
        # -- this was the cause of the near-cart 180-spin-away. Stop and wait to re-acquire.
        if age > self.lost_timeout:
            self.get_logger().warn(
                f"cart not seen for {age:.1f}s -> STOP (odom pin drifts; re-acquire the cart)",
                throttle_duration_sec=2.0)
            self._turn_acc = 0.0
            self.cmd_pub.publish(cmd); return

        # prefer the LIVE camera bearing/range (drift-free) when fresh; bridge a brief gap on the odom pin
        if age < self.cam_fresh and self.cam_bear is not None:
            bear, rng, src = self.cam_bear, self.cam_rng, "cam"
        else:
            bear, rng, src = o_bear, o_rng, "odom"

        if self.state == "APPROACH" and rng <= self.standoff + self.arrive_tol:
            self.reached_pub.publish(Bool(data=True))
            if self.orbit:
                self.state = "ORBIT"
                self.get_logger().info(f"REACHED standoff ({rng:.2f} m) -> ORBIT around cart")
            else:
                self.state = "ARRIVED"
                self.get_logger().info(f"REACHED standoff: {rng:.2f} m ({src}) -> stop & hold")
                self.cmd_pub.publish(cmd); return

        if self.state == "ORBIT":
            cmd.angular.z = _clamp(self.yaw_sign * self.k_yaw * bear, -self.vyaw_cap, self.vyaw_cap)  # keep facing
            cmd.linear.x = _clamp(self.k_fwd * (rng - self.standoff), -self.vx_cap, self.vx_cap)      # hold radius
            cmd.linear.y = _clamp(self.orbit_dir * self.orbit_speed, -0.10, 0.10)                     # strafe tangentially
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f"orbit: range={rng:.2f} bear={math.degrees(bear):+.0f}deg -> vx={cmd.linear.x:.2f} "
                f"vy={cmd.linear.y:+.2f} vyaw={cmd.angular.z:+.2f}", throttle_duration_sec=1.0)
            return

        # APPROACH -- the proven control law on the (camera-preferred) bearing/range
        vx, vyaw = approach_cmd(bear, rng, standoff=self.standoff, k_yaw=self.k_yaw, k_fwd=self.k_fwd,
                                vyaw_cap=self.vyaw_cap, vx_cap=self.vx_cap, center_thresh=self.center_thresh,
                                yaw_sign=self.yaw_sign, min_approach=self.min_approach)

        # SPIN GUARD: turning hard with NO forward progress -> STOP (backstop for a runaway spin).
        dt = 1.0 / self.control_hz
        self._turn_acc = 0.0 if vx > 1e-3 else self._turn_acc + abs(vyaw) * dt
        if self._turn_acc > self.turn_limit:
            self.get_logger().warn(
                f"spin guard: turned {self._turn_acc:.1f} rad with no forward progress -> STOP",
                throttle_duration_sec=2.0)
            self._turn_acc = 0.0
            self.cmd_pub.publish(cmd); return

        cmd.linear.x, cmd.angular.z = vx, vyaw
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"approach({src}): range={rng:.2f} bear={math.degrees(bear):+.0f}deg "
            f"cart_odom=({self.cart[0]:+.2f},{self.cart[1]:+.2f}) robot=({self.rx:+.2f},{self.ry:+.2f}) "
            f"-> vx={vx:.2f} vyaw={vyaw:+.2f}", throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = CartTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
