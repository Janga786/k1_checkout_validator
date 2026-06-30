#!/usr/bin/env python3
"""Full pipeline in RViz: the K1 drives to a QR panel, then its arm scans it.

  DRIVE  : a kinematic pursuit glides the base to a *scan pose* where the QR
           sits in the left arm's workspace. (The Nav2/SLAM brain is proven
           separately in run_sim_nav / run_gazebo; here the focus is the
           end-to-end drive -> reach -> scan with the real articulated arm.)
  REACH  : the validated 4-DOF IK extends the arm so the hand scanner reaches
           the QR (optimized down-and-outboard mount).
  SCAN   : the scanner beam lands on the QR -> it turns green -> "SCANNED".

Publishes world->base_link TF (moving base), /joint_states (arm animated, body
neutral) and /scan_markers (QR panel, hand scanner, driven path, status text).
robot_state_publisher renders the full articulated URDF.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster

from k1_qr_nav.arm_kin import K1Arm, DEFAULT_URDF
from k1_qr_nav.arm_reach import quat_axis_to, SCANNER_AXIS, STANDOFF, CONE_DEG

TRUNK_H = 0.62                              # Trunk height above base_link (ground)
QR_WORLD = np.array([3.0, 0.30, TRUNK_H + 0.169])     # QR panel pose in the world
QR_TRUNK = np.array([0.12, 0.302, 0.169])  # QR relative to Trunk at the scan pose (reachable+aimed)
SCAN_XY = QR_WORLD[:2] - QR_TRUNK[:2]       # base parks here, facing +X
ITEM = "Iodized Salt"                       # what the QR decodes to (for flavour)


def ease(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MobileScan(Node):
    def __init__(self):
        super().__init__("mobile_scan")
        self.arm = K1Arm("left", tip_xyz=tuple(STANDOFF * SCANNER_AXIS),
                         scanner_axis=tuple(SCANNER_AXIS))
        self.arm_joints = [j.name for j in self.arm.joints]
        self.all_joints = [j.get("name") for j in ET.parse(DEFAULT_URDF).getroot().findall("joint")
                           if j.get("type") == "revolute"]
        sol = self.arm.ik_best_aim(QR_TRUNK, desired_aim=(1, 0, 0), seeds=20)
        self.goal_q = sol["q"]
        self.get_logger().info(
            f"scan pose: base {np.round(SCAN_XY,2).tolist()} facing +X | QR in arm frame "
            f"{QR_TRUNK.tolist()} -> reach {sol['err']*1000:.0f}mm aim {sol.get('aim_err',99):.0f}deg")
        self.scanner = self._build_scanner("left_hand_link")

        self.br = TransformBroadcaster(self)
        self.js = self.create_publisher(JointState, "joint_states", 10)
        self.mk = self.create_publisher(MarkerArray, "scan_markers", 10)

        self.pos = np.array([0.0, 0.0])
        self.yaw = 0.0
        self.q_arm = np.zeros(self.arm.n)
        self.phase = "DRIVE"
        self.t_phase = 0.0
        self.scanned = False
        self.trail = []
        self.dt = 1.0 / 30.0
        self.create_timer(self.dt, self.tick)
        self.get_logger().info("mobile_scan up | DRIVE -> REACH -> SCAN")

    # --------------------------------------------------------- state machine
    def tick(self):
        dt = self.dt
        if self.phase == "DRIVE":
            d = SCAN_XY - self.pos
            dist = float(np.linalg.norm(d))
            if dist > 0.04:
                heading = math.atan2(d[1], d[0])
                ye = wrap(heading - self.yaw)
                self.yaw += np.clip(ye, -2.2 * dt, 2.2 * dt)
                v = min(0.9 if abs(ye) < 0.3 else 0.12, 1.5 * dist)
                self.pos = self.pos + v * dt * np.array([math.cos(self.yaw), math.sin(self.yaw)])
                if not self.trail or np.linalg.norm(self.pos - self.trail[-1]) > 0.05:
                    self.trail.append(self.pos.copy())
            else:
                ye = wrap(0.0 - self.yaw)                    # settle facing +X
                if abs(ye) > 0.03:
                    self.yaw += np.clip(ye, -1.6 * dt, 1.6 * dt)
                else:
                    self.yaw = 0.0
                    self.phase, self.t_phase = "REACH", 0.0
        elif self.phase == "REACH":
            self.q_arm = ease(self.t_phase / 2.6) * self.goal_q
            self.t_phase += dt
            if self.t_phase >= 2.6:
                self.phase, self.t_phase, self.scanned = "SCAN", 0.0, True
                self.get_logger().info(f"SCANNED -> {ITEM}")
        elif self.phase == "SCAN":
            self.q_arm = self.goal_q
            self.t_phase += dt
            if self.t_phase >= 3.2:
                self.phase, self.t_phase = "RETRACT", 0.0
        elif self.phase == "RETRACT":
            self.q_arm = (1 - ease(self.t_phase / 2.0)) * self.goal_q
            self.t_phase += dt
            if self.t_phase >= 2.0:                          # reset + loop
                self.pos, self.yaw, self.scanned, self.trail = np.array([0.0, 0.0]), 0.0, False, []
                self.phase, self.t_phase = "DRIVE", 0.0
        self.publish()

    # --------------------------------------------------------------- output
    def publish(self):
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(self.pos[0])
        t.transform.translation.y = float(self.pos[1])
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.br.sendTransform(t)

        js = JointState()
        js.header.stamp = now
        amap = dict(zip(self.arm_joints, self.q_arm))
        js.name = self.all_joints
        js.position = [float(amap.get(n, 0.0)) for n in self.all_joints]
        self.js.publish(js)

        self._markers(now)

    def _markers(self, stamp):
        arr = MarkerArray()
        # QR panel (world), green once scanned
        qr = Marker()
        qr.header.frame_id = "world"; qr.header.stamp = stamp
        qr.ns = "qr"; qr.id = 0; qr.type = Marker.CUBE; qr.action = Marker.ADD
        qr.pose.position = Point(x=float(QR_WORLD[0]), y=float(QR_WORLD[1]), z=float(QR_WORLD[2]))
        qr.pose.orientation.w = 1.0
        qr.scale.x, qr.scale.y, qr.scale.z = 0.02, 0.12, 0.12
        if self.scanned:
            qr.color.r, qr.color.g, qr.color.b, qr.color.a = 0.1, 0.9, 0.1, 1.0
        else:
            qr.color.r, qr.color.g, qr.color.b, qr.color.a = 0.1, 0.1, 0.1, 1.0
        arr.markers.append(qr)
        # post under the QR
        post = Marker()
        post.header.frame_id = "world"; post.header.stamp = stamp
        post.ns = "post"; post.id = 0; post.type = Marker.CYLINDER; post.action = Marker.ADD
        post.pose.position = Point(x=float(QR_WORLD[0]), y=float(QR_WORLD[1]), z=float(QR_WORLD[2] / 2))
        post.pose.orientation.w = 1.0
        post.scale.x, post.scale.y, post.scale.z = 0.03, 0.03, float(QR_WORLD[2])
        post.color.r, post.color.g, post.color.b, post.color.a = 0.4, 0.4, 0.4, 1.0
        arr.markers.append(post)
        # driven path
        if len(self.trail) > 1:
            pl = Marker()
            pl.header.frame_id = "world"; pl.header.stamp = stamp
            pl.ns = "path"; pl.id = 0; pl.type = Marker.LINE_STRIP; pl.action = Marker.ADD
            pl.scale.x = 0.02
            pl.color.r, pl.color.g, pl.color.b, pl.color.a = 0.2, 0.5, 1.0, 0.8
            pl.points = [Point(x=float(p[0]), y=float(p[1]), z=0.02) for p in self.trail]
            arr.markers.append(pl)
        # status text above the QR
        st = Marker()
        st.header.frame_id = "world"; st.header.stamp = stamp
        st.ns = "status"; st.id = 0; st.type = Marker.TEXT_VIEW_FACING; st.action = Marker.ADD
        st.pose.position = Point(x=float(QR_WORLD[0]), y=float(QR_WORLD[1]), z=float(QR_WORLD[2] + 0.22))
        st.pose.orientation.w = 1.0
        st.scale.z = 0.10
        msg = {"DRIVE": "Navigating to QR...", "REACH": "Reaching to scan...",
               "SCAN": f"SCANNED  v  {ITEM}", "RETRACT": "Done - retracting"}[self.phase]
        ok = self.phase == "SCAN"
        st.color.r, st.color.g, st.color.b, st.color.a = (0.1, 0.9, 0.1, 1.0) if ok else (0.95, 0.95, 0.95, 1.0)
        st.text = msg
        arr.markers.append(st)
        # hand scanner (moves with left_hand_link)
        for m in self.scanner:
            m.header.stamp = stamp
            arr.markers.append(m)
        self.mk.publish(arr)

    # ---------------------------------------------------------- scanner mesh
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

        box(0.018 * ax, q_ax, 0.036, 0.046, 0.030, gray, "scan_body")
        box(0.018 * ax + 0.030 * grip, q_grip, 0.045, 0.020, 0.022, gray, "scan_grip")
        lens = Marker()
        lens.ns = "scan_lens"; lens.id = 0; lens.type = Marker.CYLINDER; lens.action = Marker.ADD
        p = 0.037 * ax
        lq = quat_axis_to([0, 0, 1], ax)
        lens.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        lens.pose.orientation.x, lens.pose.orientation.y, lens.pose.orientation.z, lens.pose.orientation.w = lq
        lens.scale.x, lens.scale.y, lens.scale.z = 0.026, 0.026, 0.006
        lens.color.r, lens.color.g, lens.color.b, lens.color.a = 0.05, 0.05, 0.05, 1.0
        out.append(lens)
        beam = Marker()
        beam.ns = "scan_beam"; beam.id = 0; beam.type = Marker.ARROW; beam.action = Marker.ADD
        s, e = 0.040 * ax, (0.040 + STANDOFF + 0.04) * ax
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
    node = MobileScan()
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
