"""4-DOF forward/inverse kinematics for the Booster K1 arm.

Parsed straight from the K1 URDF (no hardcoded geometry). The arm chain is
Shoulder_Pitch -> Shoulder_Roll -> Elbow_Pitch -> Elbow_Yaw -> hand, with NO
wrist -- so the hand's orientation is coupled to its position. We solve for the
scanner-tip POSITION (3 task DoF) with the 4th joint as redundancy, and report
the resulting scanner AIM, so reachability.py can map where this wristless arm
can actually present the scanner to a barcode.

Frame: the Trunk (torso) link.  X forward, Y left, Z up.
"""
from __future__ import annotations

import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

DEFAULT_URDF = "/home/boosterk1/booster_assets/robots/K1/K1_22dof-ZED.urdf"


def _stl_verts(path):
    """Vertices (N,3) of a binary STL."""
    with open(path, "rb") as f:
        f.read(80); n = struct.unpack("<I", f.read(4))[0]; d = f.read(n * 50)
    t = np.frombuffer(d, np.uint8).reshape(n, 50)
    return np.frombuffer(t[:, 12:48].tobytes(), "<f4").reshape(-1, 3)


def hand_tip(side="left", urdf=DEFAULT_URDF):
    """Scanner-mount reference = the far (+Y) tip of the {side}_hand_link mesh, in the
    hand frame. DERIVED from the URDF + mesh (source of truth), not the old hardcoded 0.22."""
    root = ET.parse(urdf).getroot()
    fn = None
    for lk in root.findall("link"):
        if lk.get("name") == f"{side}_hand_link":
            g = lk.find("visual/geometry/mesh"); fn = g.get("filename") if g is not None else None
    if fn is None:
        return np.zeros(3)
    p = fn.replace("file://", "")
    if not os.path.isabs(p):
        p = os.path.join(os.path.dirname(os.path.abspath(urdf)), p)
    return np.array([0.0, float(_stl_verts(p)[:, 1].max()), 0.0])


def rpy_to_R(r, p, y):
    """URDF fixed-axis roll-pitch-yaw -> rotation matrix (R = Rz Ry Rx)."""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def axis_R(axis, th):
    """Rodrigues rotation about a unit-ish axis by angle th."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    c, s = np.cos(th), np.sin(th)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def axis_R_batch(axis, th):
    """Rodrigues for a fixed axis and a batch of angles th (N,) -> (N,3,3)."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    th = np.asarray(th, float)
    c, s = np.cos(th), np.sin(th)
    C = 1.0 - c
    N = th.shape[0]
    R = np.empty((N, 3, 3))
    R[:, 0, 0] = c + x * x * C; R[:, 0, 1] = x * y * C - z * s; R[:, 0, 2] = x * z * C + y * s
    R[:, 1, 0] = y * x * C + z * s; R[:, 1, 1] = c + y * y * C; R[:, 1, 2] = y * z * C - x * s
    R[:, 2, 0] = z * x * C - y * s; R[:, 2, 1] = z * y * C + x * s; R[:, 2, 2] = c + z * z * C
    return R


def seg_aabb_hit(p0, p1, lo, hi):
    """Does the segment p0->p1 intersect the axis-aligned box [lo, hi]? (slab method)"""
    p0 = np.asarray(p0, float)
    d = np.asarray(p1, float) - p0
    tmin, tmax = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-9:
            if p0[i] < lo[i] or p0[i] > hi[i]:
                return False
        else:
            t1 = (lo[i] - p0[i]) / d[i]
            t2 = (hi[i] - p0[i]) / d[i]
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return False
    return True


class Joint:
    def __init__(self, name, xyz, rpy, axis, lo, hi):
        self.name = name
        self.xyz = np.asarray(xyz, float)
        self.R0 = rpy_to_R(*rpy)
        self.axis = np.asarray(axis, float)
        self.lo = lo
        self.hi = hi


class K1Arm:
    """The K1's left or right 4-DOF arm, parsed from the URDF.

    tip_xyz / scanner_axis describe the barcode scanner mounted on the hand:
    where its read point sits (in the hand frame) and which way it reads. These
    are calibratable; the defaults are a reasonable placeholder.
    """

    def __init__(self, side="left", urdf=DEFAULT_URDF, tip_xyz=(0.03, 0.0, 0.0),
                 scanner_axis=(1.0, 0.0, 0.0)):
        self.side = side
        root = ET.parse(urdf).getroot()
        jmap = {}
        for j in root.findall("joint"):
            o, a, l = j.find("origin"), j.find("axis"), j.find("limit")
            child = j.find("child").get("link")
            jmap[child] = dict(
                name=j.get("name"), parent=j.find("parent").get("link"),
                xyz=[float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()],
                rpy=[float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()],
                axis=[float(v) for v in a.get("xyz").split()] if a is not None else [0, 0, 1],
                lo=float(l.get("lower")) if (l is not None and l.get("lower")) else -np.pi,
                hi=float(l.get("upper")) if (l is not None and l.get("upper")) else np.pi,
                vel=float(l.get("velocity")) if (l is not None and l.get("velocity")) else float("inf"),
                eff=float(l.get("effort")) if (l is not None and l.get("effort")) else float("inf"),
            )
        # walk the chain hand -> ... -> root
        chain, cur = [], f"{side}_hand_link"
        while cur in jmap:
            chain.append(jmap[cur])
            cur = jmap[cur]["parent"]
        chain.reverse()
        if not chain:
            raise ValueError(f"no arm chain found for side={side!r}")
        self.root_link = cur
        self.joints = [Joint(d["name"], d["xyz"], d["rpy"], d["axis"], d["lo"], d["hi"]) for d in chain]
        self.n = len(self.joints)
        self.tip_xyz = np.asarray(tip_xyz, float)
        self.scanner_axis = np.asarray(scanner_axis, float)
        self.scanner_axis /= np.linalg.norm(self.scanner_axis)
        self.lo = np.array([j.lo for j in self.joints])
        self.hi = np.array([j.hi for j in self.joints])
        # action-space limits straight from the URDF <limit> (source of truth, not guessed)
        self.vel = np.array([d["vel"] for d in chain])     # rad/s, per joint
        self.eff = np.array([d["eff"] for d in chain])     # Nm, per joint
        # hand tip = the far (+Y) end of the hand-link mesh -- where the scanner mounts.
        # DERIVED from the mesh, not the old hardcoded 0.22.
        self.hand_tip = hand_tip(side, urdf)

    # ------------------------------------------------------------------ FK
    def fk(self, q):
        """Forward kinematics. Returns (tip_pos[3], R_base_hand[3x3], (origins, axes))."""
        T = np.eye(4)
        origins, axes = [], []
        for i, j in enumerate(self.joints):
            Ti = np.eye(4)
            Ti[:3, :3] = j.R0 @ axis_R(j.axis, q[i])
            Ti[:3, 3] = j.xyz
            T = T @ Ti
            origins.append(T[:3, 3].copy())
            axes.append(T[:3, :3] @ j.axis)        # joint axis expressed in base frame
        R_hand = T[:3, :3]
        tip = T[:3, 3] + R_hand @ self.tip_xyz
        return tip, R_hand, (np.array(origins), np.array(axes))

    def tip(self, q):
        return self.fk(q)[0]

    def scanner_aim(self, q):
        """Unit vector the scanner points along, in the base frame."""
        _, R, _ = self.fk(q)
        v = R @ self.scanner_axis
        return v / np.linalg.norm(v)

    def jacobian(self, q):
        """3x4 position Jacobian of the scanner tip (analytic)."""
        tip, _, (origins, axes) = self.fk(q)
        J = np.zeros((3, self.n))
        for i in range(self.n):
            J[:, i] = np.cross(axes[i], tip - origins[i])
        return J

    # --------------------------------------------------- vectorized FK / IK
    def fk_batch(self, Q):
        """FK for a batch of joint vectors Q (N,n).

        Returns tip (N,3), R_hand (N,3,3), origins (N,n,3), axes (N,n,3) -- the
        latter two are joint origins and joint axes in the base frame, for the
        batched Jacobian. Pure numpy, no python loop over samples.
        """
        Q = np.atleast_2d(np.asarray(Q, float))
        N = Q.shape[0]
        T = np.tile(np.eye(4), (N, 1, 1))
        origins = np.zeros((N, self.n, 3))
        axes = np.zeros((N, self.n, 3))
        for i, j in enumerate(self.joints):
            Ti = np.tile(np.eye(4), (N, 1, 1))
            Ti[:, :3, :3] = j.R0 @ axis_R_batch(j.axis, Q[:, i])     # (N,3,3)
            Ti[:, :3, 3] = j.xyz
            T = T @ Ti
            origins[:, i, :] = T[:, :3, 3]
            axes[:, i, :] = np.einsum("nij,j->ni", T[:, :3, :3], j.axis)
        R_hand = T[:, :3, :3]
        tip = T[:, :3, 3] + np.einsum("nij,j->ni", R_hand, self.tip_xyz)
        return tip, R_hand, origins, axes

    def ik_batch(self, targets, seeds=8, iters=100, tol=1e-3, lam=0.05, rng=None):
        """Damped-least-squares IK for many targets at once (P,3).

        Runs `seeds` initial guesses per target as one big batch. Returns
        dict(Q (P,seeds,n), err (P,seeds), R_hand (P,seeds,3,3), tip (P,seeds,3)).
        ~100x faster than looping scalar ik() for grid sweeps.
        """
        targets = np.atleast_2d(np.asarray(targets, float))
        P = targets.shape[0]
        S = seeds
        if rng is None:
            rng = np.random.default_rng(0)
        Q0 = np.empty((P, S, self.n))
        Q0[:, 0, :] = 0.5 * (self.lo + self.hi)
        Q0[:, 1:, :] = self.lo + (self.hi - self.lo) * rng.random((P, S - 1, self.n))
        Q = Q0.reshape(P * S, self.n)
        Trep = np.repeat(targets, S, axis=0)
        eye3 = np.eye(3)
        for _ in range(iters):
            tip, _, origins, axes = self.fk_batch(Q)
            err = Trep - tip                                   # (M,3)
            d = tip[:, None, :] - origins                      # (M,n,3)
            J = np.transpose(np.cross(axes, d), (0, 2, 1))     # (M,3,n)
            JJ = J @ np.transpose(J, (0, 2, 1)) + (lam ** 2) * eye3
            y = np.linalg.solve(JJ, err[..., None])[..., 0]    # (M,3)
            dq = np.einsum("mab,ma->mb", J, y)                 # J^T y -> (M,n)
            Q = np.clip(Q + dq, self.lo, self.hi)
        tip, R_hand, _, _ = self.fk_batch(Q)
        err = np.linalg.norm(Trep - tip, axis=1).reshape(P, S)
        return dict(Q=Q.reshape(P, S, self.n), err=err,
                    R_hand=R_hand.reshape(P, S, 3, 3), tip=tip.reshape(P, S, 3))

    def clamp(self, q):
        return np.minimum(np.maximum(q, self.lo), self.hi)

    # ------------------------------------------------------------------ IK
    def ik(self, target, seed=None, iters=200, tol=1e-3, lam=0.05, seeds=10, rng=None):
        """Damped-least-squares IK for the tip POSITION (multi-seed).

        Returns dict(q, err, reachable, tip, aim). The 4th DoF is redundant for a
        3-D position task; a small null-space pull keeps joints mid-range.
        """
        target = np.asarray(target, float)
        if rng is None:
            rng = np.random.default_rng(0)
        qmid = 0.5 * (self.lo + self.hi)
        cands = [seed if seed is not None else qmid.copy()]
        for _ in range(seeds - 1):
            cands.append(self.lo + (self.hi - self.lo) * rng.random(self.n))
        best = None
        for q0 in cands:
            q = self.clamp(np.array(q0, float))
            for _ in range(iters):
                tip, _, (origins, axes) = self.fk(q)
                err = target - tip
                if np.linalg.norm(err) < tol:
                    break
                J = np.zeros((3, self.n))
                for i in range(self.n):
                    J[:, i] = np.cross(axes[i], tip - origins[i])
                JJ = J @ J.T + (lam ** 2) * np.eye(3)
                dq = J.T @ np.linalg.solve(JJ, err)
                N = np.eye(self.n) - J.T @ np.linalg.solve(JJ, J)   # ~null-space projector
                dq += N @ (0.05 * (qmid - q))
                q = self.clamp(q + dq)
            e = np.linalg.norm(target - self.tip(q))
            if best is None or e < best[1]:
                best = (q, e)
            if e < tol:
                break
        q, e = best
        return dict(q=q, err=float(e), reachable=bool(e <= 0.01), tip=self.tip(q),
                    aim=self.scanner_aim(q))

    def links_in_box(self, q, lo, hi, radius=0.0):
        """True if any ARM segment (shoulder->...->hand->scanner-tip) comes within
        `radius` of the axis-aligned box [lo, hi] -- segments treated as thick
        capsules (box inflated by `radius`). INCLUDES the forearm/hand->tip segment
        (that's the part that actually swings near the box); callers keep the box
        keep-out just BEHIND the QR face (target.x + margin + radius) so the scanner
        legitimately touching the face isn't flagged."""
        lo = np.asarray(lo, float) - radius
        hi = np.asarray(hi, float) + radius
        _, _, (origins, _) = self.fk(q)
        chain = list(origins) + [self.tip(q)]    # shoulder..hand joints + the scanner tip
        return any(seg_aabb_hit(a, b, lo, hi) for a, b in zip(chain[:-1], chain[1:]))

    def ik_best_aim(self, target, desired_aim, iters=120, tol=1e-3, lam=0.05, seeds=8, rng=None,
                    box=None, link_radius=0.0, self_free=None):
        """IK that also uses the redundant 4th DoF to AIM the scanner.

        Runs several seeds, keeps the position-feasible solutions (the redundancy
        manifold), and returns the one whose scanner aim best matches desired_aim,
        PREFERRING solutions whose arm doesn't pass through `box` (lo, hi) -- the
        obstacle the QR is mounted on. Returns dict like ik() plus 'aim_err',
        'collide'.
        """
        target = np.asarray(target, float)
        da = np.asarray(desired_aim, float)
        da = da / np.linalg.norm(da)
        if rng is None:
            rng = np.random.default_rng(0)
        cands = [0.5 * (self.lo + self.hi)]
        for _ in range(seeds - 1):
            cands.append(self.lo + (self.hi - self.lo) * rng.random(self.n))
        feasible = []
        for q0 in cands:
            q = self.clamp(np.array(q0, float))
            for _ in range(iters):
                tip, _, (origins, axes) = self.fk(q)
                err = target - tip
                if np.linalg.norm(err) < tol:
                    break
                J = np.zeros((3, self.n))
                for i in range(self.n):
                    J[:, i] = np.cross(axes[i], tip - origins[i])
                JJ = J @ J.T + (lam ** 2) * np.eye(3)
                q = self.clamp(q + J.T @ np.linalg.solve(JJ, err))
            e = np.linalg.norm(target - self.tip(q))
            if e <= 0.01:
                aim = self.scanner_aim(q)
                aim_err = float(np.degrees(np.arccos(np.clip(aim @ da, -1.0, 1.0))))
                collide = bool(self.links_in_box(q, box[0], box[1], link_radius)) if box is not None else False
                self_bad = bool(self_free is not None and not self_free(q))   # arm hits its own body?
                feasible.append((aim_err, e, q.copy(), aim, collide, self_bad))
        if not feasible:
            r = self.ik(target, iters=iters, tol=tol, lam=lam, seeds=seeds, rng=rng)
            r["aim_err"] = float("nan")
            r["collide"] = False
            r["self_collide"] = bool(self_free is not None and not self_free(r["q"]))
            return r
        # prefer solutions clear of BOTH the box and the K1's own body, then box-only, then any
        clean = [f for f in feasible if not f[4] and not f[5]]
        boxfree = [f for f in feasible if not f[4]]
        pool = clean or boxfree or feasible
        pool.sort(key=lambda f: f[0])
        aim_err, e, q, aim, collide, self_bad = pool[0]
        return dict(q=q, err=float(e), reachable=True, tip=self.tip(q), aim=aim,
                    aim_err=aim_err, collide=collide, self_collide=self_bad)


if __name__ == "__main__":
    arm = K1Arm("left", tip_xyz=(0, 0, 0))
    print(f"root link: {arm.root_link}  | {arm.n} joints: {[j.name for j in arm.joints]}")
    tip, R, _ = arm.fk(np.zeros(arm.n))
    print(f"FK(zeros) tip = {np.round(tip, 4)}   (hand-by-hand sum = [0.0025, 0.3109, 0.171])")
    print(f"limits lo = {np.round(arm.lo,2)}")
    print(f"limits hi = {np.round(arm.hi,2)}")
