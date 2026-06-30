#!/usr/bin/env python3
"""Drive Nav2 to a QR code.

Subscribes /qr_detection (PointStamped, in any frame), transforms it into the map
frame, computes a standoff goal *in front of* the QR (facing it), and sends a Nav2
NavigateToPose goal. The same node runs in the sim and on the real robot -- only
the detection source differs (mini_sim here; qr_locate.py on the K1).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
import tf2_ros
from tf2_geometry_msgs import do_transform_point


class QrGoal(Node):
    def __init__(self):
        super().__init__("qr_goal")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("standoff", 0.6)          # frontal: stop this far in front of the QR
        self.declare_parameter("scan_fwd", 0.0)          # scan pose: QR at (fwd,left) in the robot
        self.declare_parameter("scan_left", 0.0)         #   frame while the robot FACES `face_yaw`
        self.declare_parameter("face_yaw", 0.0)          #   (into the QR's face). scan_fwd>0 enables.
        self.declare_parameter("replan_thresh", 0.4)     # re-issue goal if the goal moves > this
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.standoff = self.get_parameter("standoff").value
        self.scan_fwd = self.get_parameter("scan_fwd").value
        self.scan_left = self.get_parameter("scan_left").value
        self.face_yaw = self.get_parameter("face_yaw").value
        self.replan = self.get_parameter("replan_thresh").value

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.arrived_pub = self.create_publisher(Bool, "nav_arrived", latch)
        self.latest = None
        self.goal_xy = None
        self.active = False
        self.arrived = False
        self.create_subscription(PointStamped, "qr_detection", self._cb, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info("qr_goal up | waiting for a QR detection + TF + Nav2 ...")

    def _cb(self, msg):
        self.latest = msg

    def _tick(self):
        if self.latest is None:
            return
        try:                                              # QR detection -> map frame
            # transform at the detection's OWN stamp (not 'latest'): the robot
            # pose then matches the instant of detection, so a turning robot
            # doesn't make the QR's map position skew/rotate.
            stamp = rclpy.time.Time.from_msg(self.latest.header.stamp)
            tf = self.buf.lookup_transform(self.map_frame, self.latest.header.frame_id, stamp,
                                           timeout=Duration(seconds=0.3))
        except Exception:
            self.get_logger().info("waiting for TF to map ...", throttle_duration_sec=3.0)
            return
        qr = do_transform_point(self.latest, tf)
        qx, qy = qr.point.x, qr.point.y
        try:                                              # robot pose in map
            rb = self.buf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return
        rx, ry = rb.transform.translation.x, rb.transform.translation.y
        if math.hypot(qx - rx, qy - ry) < 1e-3:
            return
        if self.scan_fwd != 0.0 or self.scan_left != 0.0:   # scan pose: face the QR face, offset park
            yaw = self.face_yaw
            cy, sy = math.cos(yaw), math.sin(yaw)
            gx = qx - (self.scan_fwd * cy - self.scan_left * sy)
            gy = qy - (self.scan_fwd * sy + self.scan_left * cy)
        else:                                               # frontal standoff, facing the QR
            yaw = math.atan2(qy - ry, qx - rx)
            gx = qx - self.standoff * math.cos(yaw)
            gy = qy - self.standoff * math.sin(yaw)

        if (self.active or self.arrived) and self.goal_xy \
                and math.hypot(gx - self.goal_xy[0], gy - self.goal_xy[1]) < self.replan:
            return                                          # already going there / parked, QR hasn't moved
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("waiting for Nav2 navigate_to_pose action ...", throttle_duration_sec=3.0)
            return
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = gx, gy
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_xy = (gx, gy)
        self.active = True
        self.get_logger().info(
            f"QR @ map ({qx:+.2f},{qy:+.2f}) -> goal ({gx:+.2f},{gy:+.2f}) facing {math.degrees(yaw):.0f} deg")
        g = NavigateToPose.Goal()
        g.pose = goal
        self.client.send_goal_async(g, feedback_callback=self._fb).add_done_callback(self._resp)

    def _fb(self, fb):
        self.get_logger().info(f"  nav: {fb.feedback.distance_remaining:.2f} m remaining",
                               throttle_duration_sec=2.0)

    def _resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn("Nav2 rejected the goal")
            self.active = False
            return
        gh.get_result_async().add_done_callback(self._done)

    def _done(self, fut):
        status = fut.result().status
        self.active = False
        if status == 4:                                     # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("REACHED the scan pose - QR is in arm range ✓")
            self.arrived = True
            self.arrived_pub.publish(Bool(data=True))
        else:
            self.get_logger().warn(f"nav goal ended without success (status={status}); will retry")


def main():
    rclpy.init()
    node = QrGoal()
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
