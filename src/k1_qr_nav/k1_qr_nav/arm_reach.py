#!/usr/bin/env python3
"""Animate the K1 left arm reaching to a barcode, for RViz.

Uses the validated 4-DOF IK (arm_kin) to place the hand scanner's read point on
a barcode in the Trunk frame, with the optimized down-and-outboard scanner mount.
Publishes /joint_states animating neutral -> reach -> hold -> retract, cycling
through a list of barcodes, plus Markers for:
  * each barcode (green when the scanner aim is within the read cone, else red)
  * a SCANNER mounted rigidly on the hand (body + lens + red read beam + grip),
    drawn in the left_hand_link frame so it moves with the hand and visibly
    points along the read direction.
robot_state_publisher turns the joint states into the TF the RobotModel renders.
"""
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from k1_qr_nav.arm_kin import K1Arm, DEFAULT_URDF, hand_tip

# CORRECTED end-effector: the real hand is at the END of the Left_Arm_4 mesh (+0.228 m in
# the hand_link frame -- hand_link is the ELBOW-YAW joint, NOT the hand). The scanner mounts
# on the real HAND, so its read point = hand tip + standoff along the beam. The old model
# put the scanner ~6 cm from the elbow joint (Gazebo physics caught that bug).
HAND_OFFSET = hand_tip("left")                       # mesh-derived tip (URDF source of truth)
SCANNER_AXIS = np.array([0.598, 0.762, -0.250])      # re-optimized mount for the real hand
SCANNER_AXIS = SCANNER_AXIS / np.linalg.norm(SCANNER_AXIS)
STANDOFF = 0.05          # read distance from the hand tip along the beam (m)
CONE_DEG = 60.0          # scanner read cone


def ease(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


def quat_axis_to(ref, v):
    """quaternion (x,y,z,w) rotating unit `ref` onto unit `v`."""
    ref = np.asarray(ref, float)
    v = np.asarray(v, float)
    v = v / np.linalg.norm(v)
    d = float(np.dot(ref, v))
    if d > 0.999999:
        return (0.0, 0.0, 0.0, 1.0)
    if d < -0.999999:
        perp = np.cross(ref, [1.0, 0.0, 0.0])
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(ref, [0.0, 1.0, 0.0])
        perp = perp / np.linalg.norm(perp)
        return (float(perp[0]), float(perp[1]), float(perp[2]), 0.0)
    axis = np.cross(ref, v)
    axis = axis / np.linalg.norm(axis)
    ang = np.arccos(d)
    s = np.sin(ang / 2.0)
    return (float(axis[0] * s), float(axis[1] * s), float(axis[2] * s), float(np.cos(ang / 2.0)))


class ArmReach(Node):
    def __init__(self):
        super().__init__("arm_reach")
        self.declare_parameter("base_frame", "Trunk")
        self.declare_parameter("hand_frame", "left_hand_link")
        # reachable + well-aimed barcodes for the CORRECTED real-hand model (x ~0.30, the
        # real-hand reach zone -- the old elbow model used x ~0.12)
        self.declare_parameter("targets",
                               [0.30, 0.24, 0.169, 0.30, 0.20, 0.10, 0.29, 0.21, 0.20])
        self.base = self.get_parameter("base_frame").value
        self.hand = self.get_parameter("hand_frame").value
        flat = list(self.get_parameter("targets").value)
        self.targets = [np.array(flat[i:i + 3]) for i in range(0, len(flat), 3)]

        # read point = the real HAND TIP + standoff along the beam, so the scanner sits on
        # the END EFFECTOR (not the elbow) and visibly points at the barcode it reaches.
        self.arm = K1Arm("left", tip_xyz=tuple(HAND_OFFSET + STANDOFF * SCANNER_AXIS),
                         scanner_axis=tuple(SCANNER_AXIS))
        self.arm_joints = [j.name for j in self.arm.joints]
        self.all_joints = [j.get("name") for j in ET.parse(DEFAULT_URDF).getroot().findall("joint")
                           if j.get("type") == "revolute"]

        self.sol = []
        for t in self.targets:
            r = self.arm.ik_best_aim(t, desired_aim=(1, 0, 0), seeds=16)
            self.sol.append(r)
            self.get_logger().info(
                f"barcode {np.round(t, 3).tolist()} -> reach err {r['err']*1000:4.0f} mm | "
                f"reachable={r['reachable']} | scanner aim {r.get('aim_err', float('nan')):3.0f} deg "
                f"({'SCAN OK' if r.get('aim_err', 99) <= CONE_DEG else 'out of cone'})")

        self.scanner_markers = self._build_scanner()       # constant in the hand frame
        self.js = self.create_publisher(JointState, "joint_states", 10)
        self.mk = self.create_publisher(MarkerArray, "arm_markers", 10)
        self.phases = [("wait", 1.0), ("reach", 2.2), ("hold", 1.8), ("retract", 1.6)]
        self.i = 0
        self.p = 0
        self.t_phase = 0.0
        self.dt = 1.0 / 30.0
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f"arm_reach up | {len(self.targets)} barcodes | scanner on {self.hand}")

    # ---------------------------------------------------------------- scanner
    def _build_scanner(self):
        """A simple hand-mounted scanner (body + lens + read beam + grip), in the
        hand frame, pointing along SCANNER_AXIS. Returned as a list of Markers."""
        ax = SCANNER_AXIS
        q_ax = quat_axis_to([1, 0, 0], ax)                 # +X (box/arrow long axis) -> beam
        # grip hangs "down": the part of -Z perpendicular to the beam
        down = np.array([0.0, 0.0, -1.0])
        grip = down - np.dot(down, ax) * ax
        grip = grip / np.linalg.norm(grip) if np.linalg.norm(grip) > 1e-6 else np.array([0, 0, -1.0])
        q_grip = quat_axis_to([1, 0, 0], grip)
        gray = (0.20, 0.20, 0.24, 1.0)
        out = []

        def box(mid, q, sx, sy, sz, color, ns):
            m = Marker()
            m.ns = ns; m.id = len(out); m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position = Point(x=float(mid[0]), y=float(mid[1]), z=float(mid[2]))
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
            m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
            m.color.r, m.color.g, m.color.b, m.color.a = color
            out.append(m)

        box(HAND_OFFSET + 0.018 * ax, q_ax, 0.036, 0.046, 0.030, gray, "scanner_body")     # body
        box(HAND_OFFSET + 0.018 * ax + 0.030 * grip, q_grip, 0.045, 0.020, 0.022, gray, "scanner_grip")  # grip
        # lens (dark disc) on the front face
        lens = Marker()
        lens.ns = "scanner_lens"; lens.id = len(out); lens.type = Marker.CYLINDER; lens.action = Marker.ADD
        lq = quat_axis_to([0, 0, 1], ax)
        p = HAND_OFFSET + 0.037 * ax
        lens.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        lens.pose.orientation.x, lens.pose.orientation.y, lens.pose.orientation.z, lens.pose.orientation.w = lq
        lens.scale.x, lens.scale.y, lens.scale.z = 0.026, 0.026, 0.006
        lens.color.r, lens.color.g, lens.color.b, lens.color.a = 0.05, 0.05, 0.05, 1.0
        out.append(lens)
        # read beam (bright red arrow) from the lens along the beam, past the read point
        beam = Marker()
        beam.ns = "scanner_beam"; beam.id = len(out); beam.type = Marker.ARROW; beam.action = Marker.ADD
        s = HAND_OFFSET + 0.040 * ax
        e = HAND_OFFSET + (0.040 + STANDOFF + 0.04) * ax
        beam.points = [Point(x=float(s[0]), y=float(s[1]), z=float(s[2])),
                       Point(x=float(e[0]), y=float(e[1]), z=float(e[2]))]
        beam.scale.x, beam.scale.y, beam.scale.z = 0.006, 0.014, 0.02
        beam.color.r, beam.color.g, beam.color.b, beam.color.a = 0.95, 0.15, 0.10, 0.9
        out.append(beam)
        for m in out:
            m.header.frame_id = self.hand
        return out

    # ----------------------------------------------------------------- anim
    def _q_arm(self):
        goal = self.sol[self.i]["q"]
        name, dur = self.phases[self.p]
        u = ease(self.t_phase / dur)
        if name == "wait":
            return np.zeros(self.arm.n)
        if name == "reach":
            return u * goal
        if name == "hold":
            return goal
        return (1 - u) * goal

    def tick(self):
        q = self._q_arm()
        now = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = now
        amap = dict(zip(self.arm_joints, q))
        js.name = self.all_joints
        js.position = [float(amap.get(n, 0.0)) for n in self.all_joints]
        self.js.publish(js)
        self._markers(now)

        self.t_phase += self.dt
        if self.t_phase >= self.phases[self.p][1]:
            self.t_phase = 0.0
            self.p += 1
            if self.p >= len(self.phases):
                self.p = 0
                self.i = (self.i + 1) % len(self.targets)

    def _markers(self, stamp):
        arr = MarkerArray()
        t = self.targets[self.i]
        aim_ok = self.sol[self.i].get("aim_err", 99) <= CONE_DEG
        bc = Marker()
        bc.header.frame_id = self.base; bc.header.stamp = stamp
        bc.ns = "barcode"; bc.id = 0; bc.type = Marker.CUBE; bc.action = Marker.ADD
        bc.pose.position = Point(x=float(t[0]), y=float(t[1]), z=float(t[2]))
        bc.pose.orientation.w = 1.0
        bc.scale.x, bc.scale.y, bc.scale.z = 0.012, 0.05, 0.05
        bc.color.r, bc.color.g, bc.color.b, bc.color.a = (0.1, 0.9, 0.1, 1.0) if aim_ok else (0.9, 0.1, 0.1, 1.0)
        arr.markers.append(bc)
        for m in self.scanner_markers:
            m.header.stamp = stamp
            arr.markers.append(m)
        self.mk.publish(arr)


def main():
    rclpy.init()
    node = ArmReach()
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
