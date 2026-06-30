#!/usr/bin/env python3
"""Active head behaviour: SWEEP to find a barcode, then TRACK to keep it in view.

The K1's head is a 2-DOF (pitch, yaw) mount carrying the ZED. With a ~97 deg camera FOV
plus a +-54 deg head-yaw sweep, the robot covers a ~200 deg cone WITHOUT turning its body
-- so it can find a barcode that isn't dead-ahead. Once `barcode_locate` reports a bearing,
the head stops sweeping and centres the camera on the barcode (proportional yaw servo on the
detection's normalised pixel column), keeping it framed while Nav2 walks the body there.

Publishes /head_cmd (Float64MultiArray [pitch, yaw], rad) -> booster_bridge -> RotateHead.
Subscribes /barcode_bearing ([u_norm, score]) and /nav_arrived (Bool).

States:  SEARCH (triangle-wave yaw sweep) -> TRACK (centre on the barcode) -> HOLD (arrived).
A TRACK with no fresh bearing for `lost_timeout` s falls back to SEARCH.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray


class HeadSearch(Node):
    def __init__(self):
        super().__init__("head_search")
        self.declare_parameter("yaw_range", 0.85)       # rad: sweep +-this (head_yaw_limit is 0.95)
        self.declare_parameter("search_pitch", 0.25)    # rad down: barcodes sit around torso height
        self.declare_parameter("sweep_period", 6.0)     # s for a full left-right-left sweep
        self.declare_parameter("track_gain", 0.6)       # rad per unit u_norm per step (neg feedback)
        self.declare_parameter("track_sign", -1.0)      # flip if the head servos the wrong way
        self.declare_parameter("track_pitch", 0.15)     # rad down while tracking
        self.declare_parameter("lost_timeout", 1.5)     # s without a bearing -> back to SEARCH
        self.declare_parameter("rate_hz", 10.0)
        gp = lambda n: self.get_parameter(n).value
        self.yaw_range = gp("yaw_range")
        self.search_pitch = gp("search_pitch")
        self.sweep_period = gp("sweep_period")
        self.track_gain = gp("track_gain")
        self.track_sign = gp("track_sign")
        self.track_pitch = gp("track_pitch")
        self.lost_timeout = gp("lost_timeout")
        self.rate = gp("rate_hz")

        self.state = "SEARCH"
        self.track_yaw = 0.0
        self.last_bearing_t = None
        self.last_u = 0.0
        self.t0 = self.get_clock().now()
        self.arrived = False

        self.head_pub = self.create_publisher(Float64MultiArray, "head_cmd", 10)
        self.create_subscription(Float64MultiArray, "barcode_bearing", self._bearing_cb, 10)
        self.create_subscription(Bool, "nav_arrived", self._arrived_cb, 10)
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f"head_search up | sweep +-{math.degrees(self.yaw_range):.0f} deg "
            f"period {self.sweep_period:.0f}s | SEARCH -> TRACK -> HOLD")

    def _now_s(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def _bearing_cb(self, msg):
        if len(msg.data) < 1:
            return
        self.last_u = float(msg.data[0])
        self.last_bearing_t = self._now_s()
        if self.state == "SEARCH" and not self.arrived:
            self.get_logger().info(f"barcode in view (u={self.last_u:+.2f}) -> TRACK")
            self.state = "TRACK"

    def _arrived_cb(self, msg):
        if msg.data and not self.arrived:
            self.arrived = True
            self.state = "HOLD"
            self.get_logger().info("nav arrived -> HOLD (head fixed on the barcode)")

    def _tick(self):
        t = self._now_s()
        fresh = self.last_bearing_t is not None and (t - self.last_bearing_t) < self.lost_timeout

        if self.state == "TRACK" and not fresh:
            self.get_logger().info("lost the barcode -> SEARCH")
            self.state = "SEARCH"

        if self.state == "HOLD":
            pitch, yaw = self.track_pitch, self.track_yaw
        elif self.state == "TRACK":
            # proportional servo: drive the barcode's pixel column u_norm toward 0
            self.track_yaw += self.track_sign * self.track_gain * self.last_u / self.rate * 2.0
            self.track_yaw = max(-self.yaw_range, min(self.yaw_range, self.track_yaw))
            pitch, yaw = self.track_pitch, self.track_yaw
        else:  # SEARCH: triangle-wave sweep across the yaw range
            phase = (t % self.sweep_period) / self.sweep_period      # 0..1
            tri = 4.0 * abs(phase - 0.5) - 1.0                       # 1 -> -1 -> 1
            yaw = self.yaw_range * tri
            pitch = self.search_pitch
            self.track_yaw = yaw                                     # seed TRACK from where we are

        self.head_pub.publish(Float64MultiArray(data=[float(pitch), float(yaw)]))


def main():
    rclpy.init()
    node = HeadSearch()
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
