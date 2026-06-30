#!/usr/bin/env python3
"""Reactive cart-approach (visual servo): walk to the cart and stop `standoff` m away.

NO Nav2, NO SLAM, NO map/TF. With the head held FORWARD (yaw=0) the camera bearing equals
the BODY bearing, so the robot simply turns its body to centre the cart and walks straight
in until the (height-estimated) range reaches the standoff, then stops and holds.

  cart_locate   /cart_detection (PointStamped) + /cart_bearing ([u_norm, score])
        |   real:  head_camera_optical  -> x=right, y=down, z=forward
        |   fake:  gz_qr_truth base_link -> x=forward, y=left
        v
  cart_approach  ->  /cmd_vel (Twist vx,vyaw)   /head_cmd ([pitch,0] hold forward)   /cart_reached (Bool)
        v
  booster_bridge  hard-clamps + GATES (/start_walking) -> Move(vx,0,vyaw)

This replaces the Nav2 goal pipeline (qr_goal + navigate_to_pose + costmaps + map/odom TF) for
the "walk to a cart I can see" case. It removes the failure modes the static audit found in that
path: Nav2 lifecycle never activating, the head-pan goal desync (head is fixed here), the
goal-behind-robot math, and the map-TF timing -- none of which exist in a body-relative servo.

NOISE HANDLING (the real detector bounces score 0.09-0.74, range 1-5 m):
  * score gate          -- drop low-confidence frames
  * EMA smoothing       -- on bearing & range
  * range-jump reject   -- a clipped-bbox spike jumps far from the smoothed range -> ignore
  * lost-target timeout -- no good detection for `lost_timeout` s -> STOP (or opt-in body search)
TURN-FIRST: only drive forward once the cart is roughly centred, so a badly off-axis cart is
faced before walking (no crabbing toward empty floor).

SAFETY: this node only PUBLISHES /cmd_vel. booster_bridge still GATES (nothing moves until
/start_walking) and HARD-CLAMPS vx/vyaw. Caps here are <= the bridge's operational caps.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool, Float64MultiArray

from k1_qr_nav.cart_approach_law import approach_cmd, _clamp   # pure control law (single source of truth)


class CartApproach(Node):
    def __init__(self):
        super().__init__("cart_approach")
        d = self.declare_parameter
        d("standoff", 1.0)            # stop this far from the cart (m)
        d("arrive_tol", 0.10)         # latch 'arrived' (and stop) within standoff+this (m)
        d("min_approach", 0.06)       # m/s floor on the forward term so it doesn't creep to a halt
        d("rearm_band", 0.40)         # resume approach if the cart later recedes beyond standoff+this
        d("vx_cap", 0.18)             # forward speed cap (m/s); <= bridge max_vx (0.20)
        d("vyaw_cap", 0.30)           # turn speed cap (rad/s); == bridge max_vyaw
        d("k_fwd", 0.35)              # m/s per m of range error
        d("k_yaw", 1.2)               # rad/s per rad of bearing error
        d("yaw_sign", -1.0)           # -1: +bearing(right) -> turn right; flip if it turns the wrong way
        d("center_thresh", 0.20)      # rad: only walk forward when |bearing| < this (~11 deg)
        d("min_score", 0.30)          # drop detections below this confidence
        d("lost_timeout", 1.0)        # s without a good detection -> STOP
        d("ema_alpha", 0.4)           # smoothing weight on each new sample (0..1)
        d("max_range_jump", 1.5)      # m: reject a sample jumping more than this vs the filtered range
        d("head_pitch", 0.20)         # rad down: hold the head here, yaw=0 (forward)
        d("control_hz", 10.0)
        d("search_when_lost", False)  # if true, slow body rotate to reacquire a lost cart (default: STOP)
        d("search_vyaw", 0.20)        # rad/s body-search speed when search_when_lost
        g = lambda n: self.get_parameter(n).value
        self.standoff = g("standoff"); self.arrive_tol = g("arrive_tol")
        self.min_approach = g("min_approach"); self.rearm_band = g("rearm_band")
        self.vx_cap = g("vx_cap"); self.vyaw_cap = g("vyaw_cap")
        self.k_fwd = g("k_fwd"); self.k_yaw = g("k_yaw"); self.yaw_sign = g("yaw_sign")
        self.center_thresh = g("center_thresh"); self.min_score = g("min_score")
        self.lost_timeout = g("lost_timeout"); self.alpha = g("ema_alpha")
        self.max_jump = g("max_range_jump"); self.head_pitch = g("head_pitch")
        self.search_when_lost = g("search_when_lost"); self.search_vyaw = g("search_vyaw")

        # filtered estimate of the cart, body-relative
        self.f_bear = None      # rad, + = cart to the right
        self.f_range = None     # m
        self.last_good_t = None
        self.last_score = 1.0   # default-pass (the fake target has no /cart_bearing channel)
        self.last_score_t = None
        self.arrived = False

        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.head_pub = self.create_publisher(Float64MultiArray, "head_cmd", 10)
        self.reached_pub = self.create_publisher(Bool, "cart_reached", latch)
        self.create_subscription(PointStamped, "cart_detection", self._det_cb, 10)
        self.create_subscription(Float64MultiArray, "cart_bearing", self._bear_cb, 10)
        self.create_timer(1.0 / g("control_hz"), self._tick)
        self.get_logger().info(
            f"cart_approach up | standoff={self.standoff:.2f} m | head FORWARD pitch={self.head_pitch:.2f} "
            f"| vx<={self.vx_cap:.2f} vyaw<={self.vyaw_cap:.2f} | score>={self.min_score:.2f} "
            f"| turn-first(|bear|<{math.degrees(self.center_thresh):.0f}deg) | lost>{self.lost_timeout:.1f}s->STOP "
            f"| ONLY publishes /cmd_vel -- bridge still gates/clamps")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _bear_cb(self, msg):
        if len(msg.data) >= 2:
            self.last_score = float(msg.data[1])
            self.last_score_t = self._now()

    def _det_cb(self, msg):
        # frame-aware: extract (forward, right) from whatever convention the detection arrives in
        fr = msg.header.frame_id or ""
        if "optical" in fr:
            forward, right = msg.point.z, msg.point.x       # optical: x=right, z=forward
        else:
            forward, right = msg.point.x, -msg.point.y      # base_link: x=forward, y=left
        if forward <= 0.0:
            return                                          # behind the camera -> ignore
        # score gate -- only when a fresh score is available (fake target => last_score stays 1.0)
        score_fresh = self.last_score_t is not None and (self._now() - self.last_score_t) < 0.7
        if score_fresh and self.last_score < self.min_score:
            return
        rng = math.hypot(right, forward)
        bear = math.atan2(right, forward)                   # + = cart to the right
        # outlier reject: a clipped-bbox range spike jumps far from the smoothed range
        if self.f_range is not None and abs(rng - self.f_range) > self.max_jump:
            return
        a = self.alpha
        self.f_bear = bear if self.f_bear is None else (1 - a) * self.f_bear + a * bear
        self.f_range = rng if self.f_range is None else (1 - a) * self.f_range + a * rng
        self.last_good_t = self._now()

    def _tick(self):
        # hold the head forward + slightly down so the camera bearing == the body bearing
        self.head_pub.publish(Float64MultiArray(data=[float(self.head_pitch), 0.0]))

        cmd = Twist()
        fresh = self.last_good_t is not None and (self._now() - self.last_good_t) < self.lost_timeout
        if not fresh:
            if self.search_when_lost:
                cmd.angular.z = self.search_vyaw            # opt-in slow body sweep to reacquire
                self.get_logger().info("cart lost -> body search", throttle_duration_sec=2.0)
            else:
                self.get_logger().info("cart lost / not yet seen -> STOP", throttle_duration_sec=2.0)
            self.cmd_pub.publish(cmd)
            return

        bear, rng = self.f_bear, self.f_range
        if self.arrived:
            if rng > self.standoff + self.rearm_band:       # cart receded -> resume approach
                self.arrived = False
                self.get_logger().info(f"cart receded to {rng:.2f} m -> resume approach")
            else:
                self.cmd_pub.publish(cmd)                    # hold (zeros)
                return

        if rng <= self.standoff + self.arrive_tol:
            self.arrived = True
            self.reached_pub.publish(Bool(data=True))
            self.get_logger().info(
                f"REACHED standoff: range {rng:.2f} m <= {self.standoff + self.arrive_tol:.2f} m -> stop & hold")
            self.cmd_pub.publish(cmd)                        # zeros
            return

        vx, vyaw = approach_cmd(
            bear, rng, standoff=self.standoff, k_yaw=self.k_yaw, k_fwd=self.k_fwd,
            vyaw_cap=self.vyaw_cap, vx_cap=self.vx_cap,
            center_thresh=self.center_thresh, yaw_sign=self.yaw_sign,
            min_approach=self.min_approach)
        cmd.linear.x, cmd.angular.z = vx, vyaw
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"approach: range={rng:.2f} bear={math.degrees(bear):+.0f}deg -> vx={vx:.2f} vyaw={vyaw:+.2f}",
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = CartApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())                   # leave a zero command behind
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
