#!/usr/bin/env python3
"""Publish a clean odom->base_link TF from the /odom topic.

Gazebo's OdometryPublisher also emits this transform on gz /tf, but the ros_gz
bridge relays it with out-of-order timestamps -- tf2 then drops every lookup
with TF_OLD_DATA, which silently breaks SLAM, Nav2 costmaps, and QR detection.
The /odom *topic* keeps clean sim stamps, so re-broadcasting the transform from
it gives the whole stack a reliable odom->base_link. A monotonic guard drops any
out-of-order /odom sample so consumers never see a "jump back in time".
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self):
        super().__init__("odom_tf")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.br = TransformBroadcaster(self)
        self.last_ns = -1
        self.create_subscription(Odometry, "odom", self._cb, 20)
        self.get_logger().info(
            f"odom_tf up | {self.odom_frame} -> {self.base_frame} from /odom (clean, monotonic)")

    def _cb(self, msg):
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if ns <= self.last_ns:
            return            # drop out-of-order /odom -> keep the TF monotonic
        self.last_ns = ns
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTf()
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
