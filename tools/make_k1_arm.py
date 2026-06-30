#!/usr/bin/env python3
"""Generate a Gazebo (SDF) K1 with an ARTICULATED left arm + collisions, for
physics-based collision testing against the box the QR is mounted on.

The body (torso/legs/head/right arm) is baked into one fixed base link (like
the glider); the LEFT ARM is 4 real revolute joints (Shoulder_Pitch/Roll,
Elbow_Pitch/Yaw) with mesh visuals AND mesh collisions, a JointPositionController
per joint, and a contact sensor per link. Gazebo physics then tells us -- for
real -- whether any arm link touches the box.

Out: src/k1_qr_nav/models/k1_arm/{model.sdf,model.config}
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

URDF = "/home/boosterk1/booster_assets/robots/K1/K1_22dof-ZED.urdf"
MESH_BASE = "/home/boosterk1/booster_assets/robots/K1"
OUT_DIR = "/home/boosterk1/k1_qr_nav_ws/src/k1_qr_nav/models/k1_arm"
BASE_LINK = "Trunk"
ARM_LINKS = ["Left_Arm_1", "Left_Arm_2", "Left_Arm_3", "left_hand_link"]
TRUNK_Z = 0.62                                  # base height above the floor


def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    p = math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2]))
    return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])


def xform(xyz, rpy):
    T = np.eye(4); T[:3, :3] = rpy_to_R(*rpy); T[:3, 3] = xyz
    return T


def parse_origin(el):
    o = el.find("origin")
    xyz, rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    if o is not None:
        if o.get("xyz"):
            xyz = [float(v) for v in o.get("xyz").split()]
        if o.get("rpy"):
            rpy = [float(v) for v in o.get("rpy").split()]
    return xyz, rpy


def pose_str(T):
    x, y, z = T[:3, 3]
    r, p, yw = R_to_rpy(T[:3, :3])
    return f"{x:.5f} {y:.5f} {z:.5f} {r:.5f} {p:.5f} {yw:.5f}"


root = ET.parse(URDF).getroot()

joints = {}                                     # child_link -> joint dict
children = {}
for j in root.findall("joint"):
    par, ch = j.find("parent").get("link"), j.find("child").get("link")
    xyz, rpy = parse_origin(j)
    children.setdefault(par, []).append((ch, xform(xyz, rpy)))
    a = j.find("axis"); l = j.find("limit")
    joints[ch] = dict(name=j.get("name"), parent=par, xyz=xyz, rpy=rpy,
                      axis=[float(v) for v in a.get("xyz").split()] if a is not None else [0, 0, 1],
                      lo=float(l.get("lower")) if (l is not None and l.get("lower")) else -3.14,
                      hi=float(l.get("upper")) if (l is not None and l.get("upper")) else 3.14)

link_visuals = {}
for link in root.findall("link"):
    vs = []
    for v in link.findall("visual"):
        g = v.find("geometry"); m = g.find("mesh") if g is not None else None
        if m is None:
            continue
        xyz, rpy = parse_origin(v)
        vs.append((m.get("filename"), m.get("scale", "1 1 1"), xform(xyz, rpy)))
    if vs:
        link_visuals[link.get("name")] = vs

# forward kinematics at zero joint angles
poses = {BASE_LINK: np.eye(4)}
stack = [BASE_LINK]
while stack:
    p = stack.pop()
    for ch, T in children.get(p, []):
        poses[ch] = poses[p] @ T
        stack.append(ch)


def mesh_uri(uri):
    return f"file://{os.path.join(MESH_BASE, uri)}"


# --- base link: every visual EXCEPT the articulated arm links ---
base_vis, i = [], 0
for link, vs in link_visuals.items():
    if link in ARM_LINKS or link not in poses:
        continue
    for uri, scale, Tv in vs:
        Tm = poses[link] @ Tv
        base_vis.append(
            f'      <visual name="bv{i}"><pose>{pose_str(Tm)}</pose>'
            f'<geometry><mesh><uri>{mesh_uri(uri)}</uri><scale>{scale}</scale></mesh></geometry>'
            f'<material><ambient>0.55 0.57 0.6 1</ambient><diffuse>0.7 0.72 0.75 1</diffuse></material></visual>')
        i += 1
base_vis_xml = "\n".join(base_vis)

# --- articulated arm links + joints + controllers + contact sensors ---
arm_links_xml, arm_joints_xml, controllers, INERTIA = [], [], [], \
    "<inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>"
for k, link in enumerate(ARM_LINKS):
    jd = joints[link]
    vis = []
    for uri, scale, Tv in link_visuals.get(link, []):
        vis.append(f'<visual name="{link}_v"><pose>{pose_str(Tv)}</pose>'
                   f'<geometry><mesh><uri>{mesh_uri(uri)}</uri><scale>{scale}</scale></mesh></geometry>'
                   f'<material><ambient>0.2 0.4 0.7 1</ambient><diffuse>0.3 0.55 0.9 1</diffuse></material></visual>')
        col_uri, col_scale, col_T = uri, scale, Tv          # collision uses the same mesh
    arm_links_xml.append(f'''    <link name="{link}">
      <pose>{pose_str(poses[link])}</pose>
      <gravity>false</gravity>
      <inertial><mass>0.4</mass>{INERTIA}</inertial>
      {''.join(vis)}
      <collision name="{link}_col"><pose>{pose_str(col_T)}</pose>
        <geometry><mesh><uri>{mesh_uri(col_uri)}</uri><scale>{col_scale}</scale></mesh></geometry>
      </collision>
      <sensor name="{link}_contact" type="contact"><always_on>1</always_on><update_rate>60</update_rate>
        <contact><collision>{link}_col</collision></contact>
        <topic>arm_contact</topic>
      </sensor>
    </link>''')
    arm_joints_xml.append(f'''    <joint name="{jd['name']}" type="revolute">
      <parent>{jd['parent']}</parent>
      <child>{link}</child>
      <axis><xyz expressed_in="__model__">{jd['axis'][0]} {jd['axis'][1]} {jd['axis'][2]}</xyz>
        <limit><lower>{jd['lo']}</lower><upper>{jd['hi']}</upper><effort>50</effort></limit>
        <dynamics><damping>0.2</damping></dynamics></axis>
    </joint>''')
    controllers.append(f'''  <plugin filename="gz-sim-joint-position-controller-system"
    name="gz::sim::systems::JointPositionController">
    <joint_name>{jd['name']}</joint_name><topic>arm/j{k}</topic>
    <p_gain>120.0</p_gain><i_gain>0.0</i_gain><d_gain>8.0</d_gain></plugin>''')

MODEL = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="k1_arm">
    <pose>0 0 {TRUNK_Z} 0 0 0</pose>
    <joint name="anchor" type="fixed"><parent>world</parent><child>{BASE_LINK}</child></joint>
    <link name="{BASE_LINK}">
      <gravity>false</gravity>
      <inertial><mass>20</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>0.5</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
{base_vis_xml}
    </link>
{chr(10).join(arm_links_xml)}
{chr(10).join(arm_joints_xml)}
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
{chr(10).join(controllers)}
  </model>
</sdf>
'''
CONFIG = '''<?xml version="1.0"?>
<model><name>k1_arm</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>K1 with articulated left arm + collisions for physics collision tests.</description></model>
'''
os.makedirs(OUT_DIR, exist_ok=True)
with open(os.path.join(OUT_DIR, "model.sdf"), "w") as f:
    f.write(MODEL)
with open(os.path.join(OUT_DIR, "model.config"), "w") as f:
    f.write(CONFIG)
print(f"wrote {OUT_DIR}/model.sdf  ({i} base visuals, {len(ARM_LINKS)} articulated arm links)")
