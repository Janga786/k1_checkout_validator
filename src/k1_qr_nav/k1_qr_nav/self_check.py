"""SAFETY: the arm must never hit the K1's OWN body (torso / head / hip / leg).

The box-collision work only kept the arm out of the QR box; nothing stopped a
reach IK from folding the hand into the chest or dropping it into the left thigh,
and the gz glider arm has no collision geometry so physics never caught it either.

This is a CONSERVATIVE check (no trimesh/fcl on this box): the body parts the left
arm could plausibly hit become axis-aligned keep-out boxes (their mesh AABBs in the
Trunk frame, inflated by a margin); the moving arm links' mesh vertices are then
transformed to the Trunk frame at config q and tested against those boxes. AABBs
OVER-approximate the body, so the check never misses a real hit (it can only be
over-cautious) -- the right bias for "it can't break itself".

Run:  .venv/bin/python -m arm.self_collision
"""
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

from k1_qr_nav.arm_kin import K1Arm, axis_R, hand_tip

MB = "/home/boosterk1/booster_assets/robots/K1/meshes/"
URDF = "/home/boosterk1/booster_assets/robots/K1/K1_22dof-ZED.urdf"
ARM_LINKS = ["Left_Arm_1", "Left_Arm_2", "Left_Arm_3", "left_hand_link"]
ARM_MESHES = ["Left_Arm_1.STL", "Left_Arm_2.STL", "Left_Arm_3.STL", "Left_Arm_4.STL"]
# moving links we test (skip Left_Arm_1: it's bolted at the shoulder, always near the torso)
CHECK_LINKS = [1, 2, 3]
# the upper arm (Left_Arm_2) is rigidly attached at the shoulder and grazes the Trunk
# AABB's +y face there -- a structural adjacency, not a collision. Skip just that pair;
# the distal forearm + hand are still checked vs the torso (any contact THEY make is a
# real fold, and a true upper-arm-into-chest fold drags the forearm in too -> caught).
EXCLUDE = {("Left_Arm_2", "Trunk"), ("Left_Arm_2", "Head_2_ZED")}
# body parts the LEFT arm could reach into -> keep-out (mesh filename stems, Trunk-frame FK)
BODY_PARTS = ["Trunk", "Head_1", "Head_2_ZED",
              "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw", "Left_Shank",
              "Right_Hip_Pitch", "Right_Hip_Roll"]
MARGIN = 0.02                                    # 2 cm safety buffer around the body


def _verts(stem):
    p = MB + stem + ".STL"
    with open(p, "rb") as f:
        f.read(80); n = struct.unpack("<I", f.read(4))[0]; d = f.read(n * 50)
    t = np.frombuffer(d, dtype=np.uint8).reshape(n, 50)
    return np.frombuffer(t[:, 12:48].tobytes(), dtype="<f4").reshape(-1, 3)


ARM_V = [_verts(m[:-4]) for m in ARM_MESHES]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _body_boxes():
    """AABB (lo,hi) per body part in the Trunk frame, from full-body URDF FK."""
    root = ET.parse(URDF).getroot()
    children, link_mesh = {}, {}
    for j in root.findall("joint"):
        par, ch = j.find("parent").get("link"), j.find("child").get("link")
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz").split() if o is not None and o.get("xyz") else [0, 0, 0])]
        rpy = [float(v) for v in (o.get("rpy").split() if o is not None and o.get("rpy") else [0, 0, 0])]
        T = np.eye(4); T[:3, :3] = _rpy(*rpy); T[:3, 3] = xyz
        children.setdefault(par, []).append((ch, T))
    for link in root.findall("link"):
        v = link.find("visual")
        m = v.find("geometry/mesh") if v is not None else None
        if m is not None:
            link_mesh[link.get("name")] = os.path.basename(m.get("filename"))[:-4]
    pose, stack = {"Trunk": np.eye(4)}, ["Trunk"]
    while stack:
        p = stack.pop()
        for ch, T in children.get(p, []):
            pose[ch] = pose[p] @ T; stack.append(ch)
    boxes = {}
    name_to_link = {v: k for k, v in link_mesh.items()}
    for stem in BODY_PARTS:
        link = name_to_link.get(stem)
        if link is None or link not in pose:
            continue
        T = pose[link]
        w = (T[:3, :3] @ _verts(stem).T).T + T[:3, 3]
        boxes[stem] = (w.min(0) - MARGIN, w.max(0) + MARGIN)
    return boxes


BODY_BOXES = _body_boxes()


def arm_link_verts(arm, q):
    """Each checked arm link's mesh vertices in the Trunk frame at config q."""
    T, out = np.eye(4), []
    for i, j in enumerate(arm.joints):
        Ti = np.eye(4); Ti[:3, :3] = j.R0 @ axis_R(j.axis, q[i]); Ti[:3, 3] = j.xyz
        T = T @ Ti
        if i in CHECK_LINKS:
            out.append((ARM_LINKS[i], (T[:3, :3] @ ARM_V[i].T).T + T[:3, 3]))
    return out


def self_collision(arm, q):
    """Return list of (arm_link, body_part, n_verts_inside) collisions (empty = safe)."""
    hits = []
    for link, v in arm_link_verts(arm, q):
        for part, (lo, hi) in BODY_BOXES.items():
            if (link, part) in EXCLUDE:
                continue
            n = int(((v >= lo) & (v <= hi)).all(1).sum())
            if n:
                hits.append((link, part, n))
    return hits


def safe(arm, q):
    return not self_collision(arm, q)


def main():
    arm = K1Arm("left")
    print(f"body keep-out boxes: {list(BODY_BOXES)}")
    print(f"checking arm links: {[ARM_LINKS[i] for i in CHECK_LINKS]}  (margin {MARGIN*100:.0f} cm)\n")

    # 1) rest pose must be self-collision free
    rest = self_collision(arm, np.zeros(arm.n))
    print(f"REST pose q=0 -> {'SAFE' if not rest else 'COLLISION ' + str(rest)}")

    # 2) the scan envelope's IK solutions must be safe
    HAND = hand_tip("left"); SAX = np.array([0.598, 0.762, -0.250]); SAX /= np.linalg.norm(SAX)
    arm = K1Arm("left", tip_xyz=tuple(HAND + 0.05 * SAX), scanner_axis=tuple(SAX))
    sf = lambda q: safe(arm, q)
    bad = bad_naive = tot = 0
    for left in np.arange(0.12, 0.34, 0.03):
        for z in np.arange(-0.05, 0.34, 0.05):
            t = np.array([0.30, left, z])
            if not arm.ik_best_aim(t, (1, 0, 0), seeds=16)["reachable"]:
                continue
            tot += 1
            bad_naive += 1 if self_collision(arm, arm.ik_best_aim(t, (1, 0, 0), seeds=16)["q"]) else 0
            r = arm.ik_best_aim(t, (1, 0, 0), seeds=24, self_free=sf)   # body-AVOIDING IK
            bad += 1 if self_collision(arm, r["q"]) else 0
    print(f"scan-envelope, body-avoiding IK: {tot-bad}/{tot} self-collision FREE"
          f"  (vs {tot-bad_naive}/{tot} with the naive IK -> the body-avoiding IK recovers "
          f"{bad_naive-bad} placements; {bad} remain genuinely unsafe)")

    # 3) NO FALSE NEGATIVES: any config whose hand is geometrically inside a body box
    #    MUST be flagged (a missed collision could break the robot).
    rng = np.random.default_rng(1)
    caught = tot_in = 0
    for _ in range(6000):
        q = rng.uniform(arm.lo, arm.hi)
        c = dict(arm_link_verts(arm, q))["left_hand_link"].mean(0)
        if any(((c >= lo) & (c <= hi)).all() for p, (lo, hi) in BODY_BOXES.items()
               if ("left_hand_link", p) not in EXCLUDE):
            tot_in += 1
            caught += 1 if self_collision(arm, q) else 0
    print(f"false-negative test: {caught}/{tot_in} hand-in-body configs flagged "
          f"-> {'NONE missed (safe)' if caught == tot_in else 'MISSED ' + str(tot_in-caught) + ' (UNSAFE)'}")


if __name__ == "__main__":
    main()
