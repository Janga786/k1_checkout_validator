#!/usr/bin/env python3
"""Ground-truth QR 'detection' for the Gazebo demo.

The QR panel's world pose is known; this publishes /qr_detection (PointStamped,
base_link) whenever it's within the head camera's FOV + range -- the SAME
interface qr_locate.py produces on the real robot.

Robot pose comes from /odom (NOT tf2): the bridged Gazebo /tf has out-of-order
timestamps that make tf2 lookups fail, but /odom carries a clean pose + stamp.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


class GzQrTruth(Node):
    def __init__(self):
        super().__init__("gz_qr_truth")
        self.declare_parameter("qr_x", 6.85)
        self.declare_parameter("qr_y", 1.4)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("fov_deg", 90.0)
        self.declare_parameter("range_m", 12.0)
        g = lambda k: self.get_parameter(k).value
        self.qx, self.qy = g("qr_x"), g("qr_y")
        self.base = g("base_frame")
        self.fov = math.radians(g("fov_deg"))
        self.range_m = g("range_m")
        self.odom = None
        self.create_subscription(Odometry, "odom", self._odom, 10)
        self.pub = self.create_publisher(PointStamped, "qr_detection", 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f"gz_qr_truth up | QR world ({self.qx},{self.qy}) | pose from /odom")

    def _odom(self, msg):
        self.odom = msg

    def tick(self):
        if self.odom is None:
            return
        p = self.odom.pose.pose
        rx, ry = p.position.x, p.position.y
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        dx, dy = self.qx - rx, self.qy - ry
        if math.hypot(dx, dy) > self.range_m:
            return
        bearing = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(bearing) > self.fov / 2.0:
            return
        msg = PointStamped()
        # stamp with the /odom time we used, so qr_goal can transform this
        # detection with the matching robot pose (no skew while turning)
        msg.header.stamp = self.odom.header.stamp
        msg.header.frame_id = self.base
        msg.point.x = dx * math.cos(yaw) + dy * math.sin(yaw)
        msg.point.y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GzQrTruth()
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
