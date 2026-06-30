#!/usr/bin/env python3
"""Direct pursuit controller: drive the robot to the QR (no Nav2/SLAM needed).

Subscribes /qr_detection (the QR's position relative to base_link) and publishes
/cmd_vel to turn toward + approach the QR, stopping at a standoff. This is the
"terminal visual servo" from the mobile-manipulation plan -- a reliable way to
see the K1 perform in Gazebo while the full Nav2 stack (proven in the RViz sim)
gets its lifecycle bring-up sorted under heavy sim load.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist


class DriveToQr(Node):
    def __init__(self):
        super().__init__("drive_to_qr")
        self.declare_parameter("standoff", 0.7)
        self.declare_parameter("v_max", 0.5)
        self.declare_parameter("w_max", 0.8)
        self.standoff = self.get_parameter("standoff").value
        self.v_max = self.get_parameter("v_max").value
        self.w_max = self.get_parameter("w_max").value
        self.latest = None
        self.reached = False
        self.create_subscription(PointStamped, "qr_detection", self._cb, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"drive_to_qr up | standoff {self.standoff} m")

    def _cb(self, msg):
        self.latest = msg

    def _tick(self):
        cmd = Twist()
        if self.latest is None:
            self.pub.publish(cmd)
            return
        x, y = self.latest.point.x, self.latest.point.y       # QR rel base: x fwd, y left
        rng = math.hypot(x, y)
        bearing = math.atan2(y, x)
        if rng <= self.standoff + 0.05:
            if not self.reached:
                self.get_logger().info(f"REACHED the QR (range {rng:.2f} m) - scanner in range ✓")
                self.reached = True
            self.pub.publish(cmd)                              # stop
            return
        self.reached = False
        w = max(-self.w_max, min(self.w_max, 1.5 * bearing))   # turn toward the QR
        v = self.v_max if abs(bearing) < 0.4 else 0.1          # drive once roughly aligned
        v = min(v, 0.7 * (rng - self.standoff))                # ease in near the goal
        cmd.linear.x = max(0.0, v)
        cmd.angular.z = w
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = DriveToQr()
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
