#!/usr/bin/env python3
"""Generate a FIXED-BASE K1 with real COLLISION geometry on body + arm + contact
sensors, so gz physics can be the ground truth for the numpy self-collision check.

Everything (FK, joint axes, meshes) is derived from the SAME URDF as K1Arm and the
self-collision check -- so a disagreement means a real bug, not a model mismatch.
Body parts get mesh collisions; the distal arm links (forearm + hand, far from the
shoulder so any body contact is real) get mesh collisions + contact sensors.
self_collide=true; the base is anchored so we can command static configs and read
whether gz reports an arm<->body contact.

Out: src/k1_qr_nav/models/k1_selfcoll/{model.sdf,model.config}
"""
import math, os, xml.etree.ElementTree as ET
import numpy as np

URDF = "/home/boosterk1/booster_assets/robots/K1/K1_22dof-ZED.urdf"
MESH_BASE = "/home/boosterk1/booster_assets/robots/K1"
OUT = "/home/boosterk1/k1_qr_nav_ws/src/k1_qr_nav/models/k1_selfcoll"
FK_ROOT = "Trunk"
ARM_LINKS = ["Left_Arm_1", "Left_Arm_2", "Left_Arm_3", "left_hand_link"]
CONTACT_LINKS = ["Left_Arm_3", "left_hand_link"]          # distal: any body contact is real
# body parts that get collision geometry (match the numpy self_collision BODY_PARTS)
BODY = ["Trunk", "Head_1", "Head_2_ZED", "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw", "Left_Shank"]


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def R_to_rpy(R):
    p = math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2]))
    return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])


def xform(xyz, rpy):
    T = np.eye(4); T[:3, :3] = rpy_to_R(*rpy); T[:3, 3] = xyz
    return T


def parse_origin(el):
    o = el.find("origin"); xyz, rpy = [0, 0, 0], [0, 0, 0]
    if o is not None:
        if o.get("xyz"): xyz = [float(v) for v in o.get("xyz").split()]
        if o.get("rpy"): rpy = [float(v) for v in o.get("rpy").split()]
    return xyz, rpy


def pose_str(T):
    x, y, z = T[:3, 3]; r, p, yw = R_to_rpy(T[:3, :3])
    return f"{x:.5f} {y:.5f} {z:.5f} {r:.5f} {p:.5f} {yw:.5f}"


def uri(u):
    return f"file://{os.path.join(MESH_BASE, u)}"


root = ET.parse(URDF).getroot()
joints, children, link_mesh = {}, {}, {}
for j in root.findall("joint"):
    par, ch = j.find("parent").get("link"), j.find("child").get("link")
    xyz, rpy = parse_origin(j); children.setdefault(par, []).append((ch, xform(xyz, rpy)))
    a = j.find("axis")
    joints[ch] = dict(name=j.get("name"), parent=par,
                      axis=[float(v) for v in a.get("xyz").split()] if a is not None else [0, 0, 1])
for link in root.findall("link"):
    m = link.find("visual/geometry/mesh")
    if m is not None: link_mesh[link.get("name")] = m.get("filename")
pose, stack = {FK_ROOT: np.eye(4)}, [FK_ROOT]
while stack:
    p = stack.pop()
    for ch, T in children.get(p, []): pose[ch] = pose[p] @ T; stack.append(ch)

# base link: body visuals + collisions for the BODY parts
base = []
for link, fn in link_mesh.items():
    if link in ARM_LINKS or link not in pose: continue
    base.append(f'      <visual name="v_{link}"><pose>{pose_str(pose[link])}</pose>'
                f'<geometry><mesh><uri>{uri(fn)}</uri></mesh></geometry></visual>')
    if link in BODY:
        base.append(f'      <collision name="c_{link}"><pose>{pose_str(pose[link])}</pose>'
                    f'<geometry><mesh><uri>{uri(fn)}</uri></mesh></geometry></collision>')

INERTIA = "<inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>"
arm_links, arm_joints, ctrls = [], [], []
for k, link in enumerate(ARM_LINKS):
    jd = joints[link]; parent = "base_link" if jd["parent"] == FK_ROOT else jd["parent"]
    fn = link_mesh[link]
    sensor = (f'<sensor name="{link}_contact" type="contact"><always_on>1</always_on><update_rate>50</update_rate>'
              f'<contact><collision>{link}_col</collision></contact><topic>selfcontact</topic></sensor>'
              if link in CONTACT_LINKS else "")
    arm_links.append(f'''    <link name="{link}">
      <pose>{pose_str(pose[link])}</pose><gravity>false</gravity>
      <inertial><mass>0.4</mass>{INERTIA}</inertial>
      <visual name="{link}_v"><geometry><mesh><uri>{uri(fn)}</uri></mesh></geometry></visual>
      <collision name="{link}_col"><geometry><mesh><uri>{uri(fn)}</uri></mesh></geometry></collision>
      {sensor}
    </link>''')
    arm_joints.append(f'''    <joint name="{jd['name']}" type="revolute">
      <parent>{parent}</parent><child>{link}</child>
      <axis><xyz expressed_in="__model__">{jd['axis'][0]} {jd['axis'][1]} {jd['axis'][2]}</xyz>
        <limit><lower>-3.2</lower><upper>3.2</upper><effort>50</effort></limit><dynamics><damping>0.5</damping></dynamics></axis>
    </joint>''')
    ctrls.append(f'''  <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
    <joint_name>{jd['name']}</joint_name><topic>arm/j{k}</topic><p_gain>60</p_gain><d_gain>4</d_gain></plugin>''')

MODEL = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="k1_selfcoll">
    <pose>0 0 0.62 0 0 0</pose>
    <self_collide>true</self_collide>
    <joint name="anchor" type="fixed"><parent>world</parent><child>base_link</child></joint>
    <link name="base_link"><gravity>false</gravity>
      <inertial><mass>20</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>0.5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
{chr(10).join(base)}
    </link>
{chr(10).join(arm_links)}
{chr(10).join(arm_joints)}
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
{chr(10).join(ctrls)}
  </model>
</sdf>
'''
os.makedirs(OUT, exist_ok=True)
open(os.path.join(OUT, "model.sdf"), "w").write(MODEL)
open(os.path.join(OUT, "model.config"), "w").write(
    '<?xml version="1.0"?>\n<model><name>k1_selfcoll</name><version>1.0</version>'
    '<sdf version="1.9">model.sdf</sdf><description>fixed-base K1 w/ collisions for self-collision ground truth</description></model>\n')
print(f"wrote {OUT}/model.sdf  (body collisions: {BODY}; contact sensors: {CONTACT_LINKS})")
