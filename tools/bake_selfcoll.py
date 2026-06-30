#!/usr/bin/env python3
"""Bake the K1 left arm STATICALLY at a given config q into a self-contained gz world
with real mesh collisions + contact sensors, so gz physics reports — exactly at q,
with no controller/settling artifacts — whether the arm hits the body.

Usage: bake_selfcoll.py q0 q1 q2 q3   -> writes /tmp/selfcoll_baked.sdf
Everything (FK, axes, meshes) is the SAME URDF the numpy self_collision check uses.
"""
import math, os, sys, xml.etree.ElementTree as ET
import numpy as np

URDF = "/home/boosterk1/booster_assets/robots/K1/K1_22dof-ZED.urdf"
MB = "/home/boosterk1/booster_assets/robots/K1"
ARM = ["Left_Arm_1", "Left_Arm_2", "Left_Arm_3", "left_hand_link"]
CONTACT = ["Left_Arm_3", "left_hand_link"]
BODY = ["Trunk", "Head_1", "Head_2_ZED", "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw", "Left_Shank"]
q = [float(v) for v in sys.argv[1:5]]


def rpy_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def axis_R(a, th):
    a = np.asarray(a, float); a = a / np.linalg.norm(a); c, s = math.cos(th), math.sin(th)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + s * K + (1 - c) * K @ K


def R_to_rpy(R):
    p = math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2]))
    return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])


def T(xyz, rpy):
    M = np.eye(4); M[:3, :3] = rpy_R(*rpy); M[:3, 3] = xyz
    return M


def ps(M):
    x, y, z = M[:3, 3]; r, p, yw = R_to_rpy(M[:3, :3])
    return f"{x:.6f} {y:.6f} {z:.6f} {r:.6f} {p:.6f} {yw:.6f}"


root = ET.parse(URDF).getroot()
J, ch, mesh = {}, {}, {}
for j in root.findall("joint"):
    par, c = j.find("parent").get("link"), j.find("child").get("link")
    o = j.find("origin"); xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    a = j.find("axis")
    J[c] = dict(parent=par, Torigin=T(xyz, rpy), name=j.get("name"),
                axis=[float(v) for v in a.get("xyz").split()] if a is not None else [0, 0, 1])
    ch.setdefault(par, []).append((c, T(xyz, rpy)))
for lk in root.findall("link"):
    m = lk.find("visual/geometry/mesh")
    if m is not None: mesh[lk.get("name")] = m.get("filename")

# body FK (q=0)
pose = {"Trunk": np.eye(4)}; st = ["Trunk"]
while st:
    p = st.pop()
    for c, Tc in ch.get(p, []): pose[c] = pose[p] @ Tc; st.append(c)

# arm FK AT q
armT, cur = {}, np.eye(4)
for k, link in enumerate(ARM):
    jd = J[link]
    cur = cur @ jd["Torigin"] @ T([0, 0, 0], [0, 0, 0])
    cur = cur @ (lambda M: M)(np.block([[axis_R(jd["axis"], q[k]), np.zeros((3, 1))], [0, 0, 0, 1]]))
    armT[link] = cur.copy()


def uri(u):
    return f"file://{os.path.join(MB, u)}"


body_xml = ""
for link, fn in mesh.items():
    if link in ARM or link not in pose: continue
    body_xml += f'<visual name="v_{link}"><pose>{ps(pose[link])}</pose><geometry><mesh><uri>{uri(fn)}</uri></mesh></geometry></visual>'
    if link in BODY:
        body_xml += f'<collision name="c_{link}"><pose>{ps(pose[link])}</pose><geometry><mesh><uri>{uri(fn)}</uri></mesh></geometry></collision>'

arm_xml, joint_xml = "", ""
for link in ARM:
    fn = mesh[link]
    sensor = (f'<sensor name="{link}_contact" type="contact"><always_on>1</always_on><update_rate>100</update_rate>'
              f'<contact><collision>{link}_col</collision></contact></sensor>') if link in CONTACT else ""
    arm_xml += f'''<link name="{link}"><pose>{ps(armT[link])}</pose><gravity>false</gravity>
      <inertial><mass>0.4</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <visual name="{link}_v"><geometry><mesh><uri>{uri(fn)}</uri></mesh></geometry></visual>
      <collision name="{link}_col"><geometry><mesh><uri>{uri(fn)}</uri></mesh></geometry></collision>{sensor}</link>'''
    parent = "base_link" if J[link]["parent"] == "Trunk" else J[link]["parent"]
    joint_xml += f'<joint name="fx_{link}" type="fixed"><parent>{parent}</parent><child>{link}</child></joint>'

WORLD = f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="bake">
  <physics name="dart" type="dart"><max_step_size>0.001</max_step_size></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
  <model name="k1"><pose>0 0 0.62 0 0 0</pose><self_collide>true</self_collide>
    <joint name="anchor" type="fixed"><parent>world</parent><child>base_link</child></joint>
    <link name="base_link"><gravity>false</gravity>
      <inertial><mass>20</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>0.5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      {body_xml}</link>
    {arm_xml}{joint_xml}
  </model>
</world></sdf>'''
open("/tmp/selfcoll_baked.sdf", "w").write(WORLD)
print(f"baked arm at q={[round(x,3) for x in q]} -> /tmp/selfcoll_baked.sdf")
