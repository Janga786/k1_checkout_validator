#!/usr/bin/env python3
"""Generate the Gazebo K1 for the CAMERA-ONLY nav demo with an ARTICULATED left arm.

Like make_k1_arm.py (baked body + 4 real revolute arm joints + JointPositionController
per joint on topics arm/j0..j3) BUT:
  * the base is FREE/floating (no world anchor) and glides via /cmd_vel (VelocityControl)
  * publishes /odom (OdometryPublisher, 2-D) for the nav stack
  * an rgbd_camera at the head (HFOV 90, ~10deg down) feeds RTAB-Map
  * the base link is named 'base_link' (= the Trunk, spawned at z=0.62) so the nav
    stack's robot_base_frame works; arm_scan's 'Trunk' is base_link (identity TF)
The left arm physically reaches the QR when arm_scan commands arm/j0..j3.

Out: src/k1_qr_nav/models/k1_glider_arm/{model.sdf,model.config}
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

URDF = "/home/boosterk1/booster_assets/robots/K1/K1_22dof-ZED.urdf"
MESH_BASE = "/home/boosterk1/booster_assets/robots/K1"
OUT_DIR = "/home/boosterk1/k1_qr_nav_ws/src/k1_qr_nav/models/k1_glider_arm"
FK_ROOT = "Trunk"                               # URDF root for forward kinematics
BASE = "base_link"                              # output name of the base link (at the FLOOR)
ARM_LINKS = ["Left_Arm_1", "Left_Arm_2", "Left_Arm_3", "left_hand_link"]
RAISE = 0.62                                     # lift meshes so base_link sits at the floor


def raise_z(T):
    T = T.copy(); T[2, 3] += RAISE; return T


def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r); cp, sp = math.cos(p), math.sin(p); cy, sy = math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def R_to_rpy(R):
    p = math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2]))
    return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])


def xform(xyz, rpy):
    T = np.eye(4); T[:3, :3] = rpy_to_R(*rpy); T[:3, 3] = xyz
    return T


def parse_origin(el):
    o = el.find("origin"); xyz, rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    if o is not None:
        if o.get("xyz"): xyz = [float(v) for v in o.get("xyz").split()]
        if o.get("rpy"): rpy = [float(v) for v in o.get("rpy").split()]
    return xyz, rpy


def pose_str(T):
    x, y, z = T[:3, 3]; r, p, yw = R_to_rpy(T[:3, :3])
    return f"{x:.5f} {y:.5f} {z:.5f} {r:.5f} {p:.5f} {yw:.5f}"


def mesh_uri(uri):
    return f"file://{os.path.join(MESH_BASE, uri)}"


root = ET.parse(URDF).getroot()
joints, children = {}, {}
for j in root.findall("joint"):
    par, ch = j.find("parent").get("link"), j.find("child").get("link")
    xyz, rpy = parse_origin(j)
    children.setdefault(par, []).append((ch, xform(xyz, rpy)))
    a = j.find("axis")
    joints[ch] = dict(name=j.get("name"), parent=par, axis=[float(v) for v in a.get("xyz").split()] if a is not None else [0, 0, 1],
                      lo=float(j.find("limit").get("lower")) if j.find("limit") is not None and j.find("limit").get("lower") else -3.14,
                      hi=float(j.find("limit").get("upper")) if j.find("limit") is not None and j.find("limit").get("upper") else 3.14)

link_visuals = {}
for link in root.findall("link"):
    vs = []
    for v in link.findall("visual"):
        g = v.find("geometry"); m = g.find("mesh") if g is not None else None
        if m is None: continue
        xyz, rpy = parse_origin(v)
        vs.append((m.get("filename"), m.get("scale", "1 1 1"), xform(xyz, rpy)))
    if vs: link_visuals[link.get("name")] = vs

poses, stack = {FK_ROOT: np.eye(4)}, [FK_ROOT]
while stack:
    p = stack.pop()
    for ch, T in children.get(p, []):
        poses[ch] = poses[p] @ T; stack.append(ch)

# --- base link: every mesh visual EXCEPT the articulated arm links ---
base_vis, i = [], 0
for link, vs in link_visuals.items():
    if link in ARM_LINKS or link not in poses: continue
    for uri, scale, Tv in vs:
        base_vis.append(f'      <visual name="bv{i}"><pose>{pose_str(raise_z(poses[link] @ Tv))}</pose>'
                        f'<geometry><mesh><uri>{mesh_uri(uri)}</uri><scale>{scale}</scale></mesh></geometry>'
                        f'<material><ambient>0.55 0.57 0.6 1</ambient><diffuse>0.7 0.72 0.75 1</diffuse></material></visual>')
        i += 1

# --- articulated arm links + joints + position controllers ---
INERTIA = "<inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>"
arm_links_xml, arm_joints_xml, controllers = [], [], []
for k, link in enumerate(ARM_LINKS):
    jd = joints[link]
    parent = BASE if jd["parent"] == FK_ROOT else jd["parent"]      # arm hangs off the base
    vis = ""
    for uri, scale, Tv in link_visuals.get(link, []):
        vis += (f'<visual name="{link}_v"><pose>{pose_str(Tv)}</pose>'
                f'<geometry><mesh><uri>{mesh_uri(uri)}</uri><scale>{scale}</scale></mesh></geometry>'
                f'<material><ambient>0.2 0.4 0.7 1</ambient><diffuse>0.3 0.55 0.9 1</diffuse></material></visual>')
    arm_links_xml.append(f'''    <link name="{link}">
      <pose>{pose_str(raise_z(poses[link]))}</pose>
      <gravity>false</gravity>
      <inertial><mass>0.4</mass>{INERTIA}</inertial>
      {vis}
    </link>''')
    arm_joints_xml.append(f'''    <joint name="{jd['name']}" type="revolute">
      <parent>{parent}</parent><child>{link}</child>
      <axis><xyz expressed_in="__model__">{jd['axis'][0]} {jd['axis'][1]} {jd['axis'][2]}</xyz>
        <limit><lower>{jd['lo']}</lower><upper>{jd['hi']}</upper><effort>50</effort></limit>
        <dynamics><damping>0.3</damping></dynamics></axis>
    </joint>''')
    controllers.append(f'''  <plugin filename="gz-sim-joint-position-controller-system"
    name="gz::sim::systems::JointPositionController">
    <joint_name>{jd['name']}</joint_name><topic>arm/j{k}</topic>
    <p_gain>40.0</p_gain><i_gain>0.0</i_gain><d_gain>3.0</d_gain></plugin>''')

MODEL = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="k1_glider_arm">
    <link name="{BASE}">
      <gravity>false</gravity>
      <inertial><mass>20</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>0.5</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
{chr(10).join(base_vis)}
      <sensor name="head_camera" type="rgbd_camera">
        <pose>0.08 0 0.88 0 0.175 0</pose>
        <topic>head_camera</topic>
        <update_rate>15</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>1.5708</horizontal_fov>
          <image><width>640</width><height>360</height><format>R8G8B8</format></image>
          <clip><near>0.2</near><far>12.0</far></clip>
          <optical_frame_id>head_camera_optical</optical_frame_id>
        </camera>
      </sensor>
    </link>
{chr(10).join(arm_links_xml)}
{chr(10).join(arm_joints_xml)}
    <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
      <topic>cmd_vel</topic>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <dimensions>2</dimensions><odom_frame>odom</odom_frame>
      <robot_base_frame>{BASE}</robot_base_frame>
      <odom_topic>odom</odom_topic><tf_topic>tf</tf_topic>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
{chr(10).join(controllers)}
  </model>
</sdf>
'''
os.makedirs(OUT_DIR, exist_ok=True)
open(os.path.join(OUT_DIR, "model.sdf"), "w").write(MODEL)
open(os.path.join(OUT_DIR, "model.config"), "w").write(
    '<?xml version="1.0"?>\n<model><name>k1_glider_arm</name><version>1.0</version>'
    '<sdf version="1.9">model.sdf</sdf><description>K1 glider with articulated left arm '
    'for camera-only nav + scan.</description></model>\n')
print(f"wrote {OUT_DIR}/model.sdf  ({i} base visuals, {len(ARM_LINKS)} arm links)")
for k, link in enumerate(ARM_LINKS):
    print(f"  arm/j{k} -> {joints[link]['name']} (parent {joints[link]['parent']}, axis {joints[link]['axis']})")
