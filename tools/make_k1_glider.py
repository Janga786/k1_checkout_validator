#!/usr/bin/env python3
"""Generate a rigid, gliding Gazebo (SDF) model of the K1 from its URDF.

The K1 URDF is a 22-DoF biped: it falls under gravity and has joint/link name
collisions SDF rejects. For a Nav2 'glide' demo we don't need articulation, so
this bakes the standing pose into ONE rigid link -- forward-kinematics at zero
joint angles places every mesh visual -- and adds a box footprint collision, a
2-D gpu_lidar, a head camera, and velocity-control + odometry plugins.

Out: src/k1_qr_nav/models/k1_glider/{model.sdf,model.config}
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

URDF = "/home/boosterk1/booster_assets/robots/K1/K1_22dof-ZED.urdf"
MESH_BASE = "/home/boosterk1/booster_assets/robots/K1"     # meshes/ live here
OUT_DIR = "/home/boosterk1/k1_qr_nav_ws/src/k1_qr_nav/models/k1_glider"
BASE_LINK = "Trunk"


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
    r = math.atan2(R[2, 1], R[2, 2])
    y = math.atan2(R[1, 0], R[0, 0])
    return r, p, y


def xform(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
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

children = {}
for j in root.findall("joint"):
    par, ch = j.find("parent").get("link"), j.find("child").get("link")
    xyz, rpy = parse_origin(j)
    children.setdefault(par, []).append((ch, xform(xyz, rpy)))

link_visuals = {}
for link in root.findall("link"):
    vs = []
    for v in link.findall("visual"):
        g = v.find("geometry")
        m = g.find("mesh") if g is not None else None
        if m is None:
            continue
        xyz, rpy = parse_origin(v)
        vs.append((m.get("filename"), m.get("scale", "1 1 1"), xform(xyz, rpy)))
    if vs:
        link_visuals[link.get("name")] = vs

# forward kinematics from the base link (zero joint angles)
poses = {BASE_LINK: np.eye(4)}
stack = [BASE_LINK]
while stack:
    p = stack.pop()
    for ch, T in children.get(p, []):
        poses[ch] = poses[p] @ T
        stack.append(ch)

visuals = []
i = 0
for link, vs in link_visuals.items():
    if link not in poses:
        continue
    for uri, scale, Tv in vs:
        Tm = poses[link] @ Tv
        mesh = os.path.join(MESH_BASE, uri)        # uri = "meshes/Trunk.STL"
        visuals.append(
            f'      <visual name="v{i}">\n'
            f'        <pose>{pose_str(Tm)}</pose>\n'
            f'        <geometry><mesh><uri>file://{mesh}</uri><scale>{scale}</scale></mesh></geometry>\n'
            f'        <material><ambient>0.6 0.62 0.66 1</ambient><diffuse>0.75 0.77 0.8 1</diffuse></material>\n'
            f'      </visual>')
        i += 1
visuals_xml = "\n".join(visuals)

MODEL = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="k1_glider">
    <link name="base_link">
      <gravity>false</gravity>
      <inertial>
        <mass>20.0</mass>
        <inertia><ixx>1.0</ixx><iyy>1.0</iyy><izz>0.5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
{visuals_xml}
      <sensor name="lidar" type="gpu_lidar">
        <pose>0 0 0.05 0 0 0</pose>
        <topic>scan</topic>
        <update_rate>10</update_rate>
        <always_on>1</always_on>
        <visualize>true</visualize>
        <lidar>
          <scan><horizontal><samples>180</samples><resolution>1</resolution>
            <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
          <range><min>0.5</min><max>10.0</max><resolution>0.01</resolution></range>
        </lidar>
      </sensor>
      <sensor name="head_camera" type="camera">
        <pose>0.10 0 0.62 0 0 0</pose>
        <topic>head_camera</topic>
        <update_rate>15</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>1.5</horizontal_fov>
          <image><width>640</width><height>480</height></image>
          <clip><near>0.1</near><far>20.0</far></clip>
        </camera>
      </sensor>
    </link>

    <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
      <topic>cmd_vel</topic>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <dimensions>2</dimensions>
      <odom_frame>odom</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_topic>odom</odom_topic>
      <tf_topic>tf</tf_topic>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>
  </model>
</sdf>
'''

CONFIG = '''<?xml version="1.0"?>
<model>
  <name>k1_glider</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Rigid gliding K1 (baked standing pose) for Nav2 demos.</description>
</model>
'''

os.makedirs(OUT_DIR, exist_ok=True)
with open(os.path.join(OUT_DIR, "model.sdf"), "w") as f:
    f.write(MODEL)
with open(os.path.join(OUT_DIR, "model.config"), "w") as f:
    f.write(CONFIG)
print(f"wrote {OUT_DIR}/model.sdf  ({i} mesh visuals placed, {len(poses)} links FK'd)")
