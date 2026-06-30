#!/usr/bin/env python3
"""Lightweight 2-D kinematic simulator to test the SLAM + Nav2 + QR-goal stack
without a physics engine. Publishes exactly what Nav2/SLAM consume:

  * integrates /cmd_vel (diff-drive) -> pose; publishes /odom + TF odom->base_link
  * ray-casts a 2-D wall world -> /scan (LaserScan, frame = base_link)
  * publishes /qr_detection (PointStamped, base_link) when the QR is within the
    camera FOV+range and not occluded -- the SAME interface qr_locate.py produces
    on the real robot (there it's the camera-optical frame + the URDF camera->base
    TF; here we publish base_link directly).

World + QR pose are parameters.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


def yaw_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _rect(x0, y0, x1, y1):
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def default_world():
    """Room perimeter (~7 x 4 m) in the odom/map plane."""
    return _rect(-1.0, -2.0, 6.0, 2.0)


def obstacles(layout):
    """Obstacle layout the robot must plan around (the QR box is added separately).
    Different layouts force different PATHS to the same scan pose -> nav robustness.
    All are kept OFF the start->QR sightline (y ~ 0.26x) so the QR stays visible and
    qr_goal always gets an initial detection."""
    if layout == 1:                       # box above the sightline -> route below
        return _rect(2.3, 0.95, 2.8, 1.9)
    if layout == 2:                       # slalom: low box then high box, both clear of sightline
        return _rect(1.8, -0.9, 2.3, 0.25) + _rect(3.2, 1.05, 3.7, 2.0)
    if layout == 3:                       # central box + one near the scan approach
        return _rect(2.5, -0.7, 3.0, 0.45) + _rect(4.0, -0.4, 4.5, 0.7)
    return _rect(2.5, -0.7, 3.0, 0.5)     # 0 (default): central box, below the sightline


class MiniSim(Node):
    def __init__(self):
        super().__init__("mini_sim")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("qr_x", 5.3)
        self.declare_parameter("qr_y", 1.4)
        self.declare_parameter("qr_box_cx", 0.0)     # box the QR is mounted on (0 size = none)
        self.declare_parameter("qr_box_cy", 0.0)
        self.declare_parameter("qr_box_size", 0.0)
        self.declare_parameter("layout", 0)          # obstacle layout (0..3) for nav robustness
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)
        g = lambda k: self.get_parameter(k).value
        self.base_frame, self.odom_frame = g("base_frame"), g("odom_frame")
        self.qr = (g("qr_x"), g("qr_y"))
        self.x, self.y, self.yaw = g("start_x"), g("start_y"), g("start_yaw")
        self.vx = self.wz = 0.0
        self.walls = default_world() + obstacles(g("layout"))
        bx, by, bs = g("qr_box_cx"), g("qr_box_cy"), g("qr_box_size")
        if bs > 0.0:                                          # footprint of the box the QR is on
            h = bs / 2.0
            self.walls += [(bx - h, by - h, bx + h, by - h), (bx + h, by - h, bx + h, by + h),
                           (bx + h, by + h, bx - h, by + h), (bx - h, by + h, bx - h, by - h)]
        self.n_beams, self.amin, self.amax, self.rmax = 180, -math.pi, math.pi, 8.0
        self.cam_fov, self.cam_range = math.radians(90), 6.5

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.qr_pub = self.create_publisher(PointStamped, "qr_detection", 10)
        self.create_subscription(Twist, "cmd_vel", self._cmd, 10)
        self.tf = TransformBroadcaster(self)
        self.t_last = self.get_clock().now()
        self.create_timer(0.05, self._step)                  # 20 Hz
        self.get_logger().info(f"mini_sim up | QR at {self.qr} | start ({self.x},{self.y})")

    def _cmd(self, m: Twist):
        self.vx, self.wz = m.linear.x, m.angular.z

    def _step(self):
        now = self.get_clock().now()
        dt = (now - self.t_last).nanoseconds * 1e-9
        self.t_last = now
        if not (0.0 < dt < 0.5):
            dt = 0.05
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        self.yaw += self.wz * dt
        stamp = now.to_msg()
        self._pub_odom(stamp)
        self.scan_pub.publish(self._scan(stamp))
        d = self._qr(stamp)
        if d is not None:
            self.qr_pub.publish(d)

    def _pub_odom(self, stamp):
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = self.x, self.y
        od.pose.pose.orientation = yaw_quat(self.yaw)
        od.twist.twist.linear.x, od.twist.twist.angular.z = self.vx, self.wz
        self.odom_pub.publish(od)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation = yaw_quat(self.yaw)
        self.tf.sendTransform(t)

    def _cast(self, a: float) -> float:
        ox, oy, dx, dy, best = self.x, self.y, math.cos(a), math.sin(a), self.rmax
        for (x1, y1, x2, y2) in self.walls:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
            if t >= 0.0 and 0.0 <= u <= 1.0 and t < best:
                best = t
        return best

    def _scan(self, stamp) -> LaserScan:
        s = LaserScan()
        s.header.stamp = stamp
        s.header.frame_id = self.base_frame
        s.angle_min, s.angle_max = self.amin, self.amax
        s.angle_increment = (self.amax - self.amin) / self.n_beams
        s.range_min, s.range_max = 0.05, self.rmax
        s.ranges = [self._cast(self.yaw + self.amin + i * s.angle_increment) for i in range(self.n_beams)]
        s.ranges = [r if r < self.rmax else float("inf") for r in s.ranges]
        return s

    def _qr(self, stamp):
        dx, dy = self.qr[0] - self.x, self.qr[1] - self.y
        rng = math.hypot(dx, dy)
        if rng > self.cam_range:
            return None
        bearing = (math.atan2(dy, dx) - self.yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(bearing) > self.cam_fov / 2.0:
            return None
        if self._cast(self.yaw + bearing) < rng - 0.15:                 # wall in the way
            return None
        p = PointStamped()
        p.header.stamp = stamp
        p.header.frame_id = self.base_frame
        p.point.x = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)    # forward
        p.point.y = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)   # left
        return p


def main():
    rclpy.init()
    node = MiniSim()
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
