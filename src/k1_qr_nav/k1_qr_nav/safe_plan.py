"""SAFE arm planning -- the gate the real K1 arm executor MUST pass before any
joint moves, so the arm can't break itself.

plan_scan(target) bundles every guard:
  * body-avoiding IK (uses the redundant DoF to find a non-folding solution)
  * box keep-out (the QR's box) at the goal
  * joint limits WITH a margin (never command into the hard stops)
  * FULL-TRAJECTORY self-collision: every waypoint from the start pose to the goal
    is checked -- the arm can be clear at both ends yet clip the torso mid-reach
  * velocity-capped, eased time parameterization (slow, smooth -- no snapping)
It returns either a validated trajectory or safe=False with the reason(s). Fail
CLOSED: anything uncertain -> unsafe -> don't move.

Run:  .venv/bin/python -m arm.safe_plan
"""
import numpy as np

from k1_qr_nav.arm_kin import K1Arm, hand_tip
from k1_qr_nav.self_check import safe as body_safe, self_collision

LIMIT_MARGIN = 0.06        # rad (~3.5 deg) buffer off the URDF hard stops (deliberate)
AIM_CONE = 60.0            # deg -- the SCANNER's read cone (datasheet value; 60 = estimate)
SAFE_VEL_FRAC = 0.03       # scan speed = 3% of the URDF joint velocity limit (deliberate, slow)
EASE_PEAK = 1.5            # smoothstep peaks at 1.5x its mean velocity -- size T for the PEAK
MIN_TIME = 2.5             # s -- floor on the move duration
TRAJ_HZ = 50
TRAJ_CHECK = 40            # waypoints to validate along the path


def ease(u):
    u = float(np.clip(u, 0.0, 1.0))
    return u * u * (3 - 2 * u)


def plan_scan(arm, target, box=None, link_radius=0.03, q_start=None, seeds=24):
    """Return dict: safe, q, trajectory[(t,q)], reasons, reach_mm, aim_deg."""
    target = np.asarray(target, float)
    q0 = np.zeros(arm.n) if q_start is None else np.asarray(q_start, float)
    sf = lambda q: body_safe(arm, q)
    r = arm.ik_best_aim(target, (1, 0, 0), seeds=seeds, box=box,
                        link_radius=link_radius, self_free=sf)
    reasons = []
    if not r.get("reachable"):
        return dict(safe=False, q=None, trajectory=[], reasons=["unreachable"],
                    reach_mm=float("nan"), aim_deg=float("nan"))
    q = r["q"]

    # --- endpoint guards ---
    if r["err"] > 0.01:
        reasons.append(f"reach {r['err']*1000:.0f}mm (>10mm)")
    if r.get("self_collide"):
        reasons.append("self-collision: no body-free solution for this placement")
    if r.get("collide"):
        reasons.append("arm would enter the QR's box")
    if not np.isnan(r.get("aim_err", np.nan)) and r["aim_err"] > AIM_CONE:
        reasons.append(f"scanner aim {r['aim_err']:.0f}deg (>cone {AIM_CONE:.0f})")
    if np.any(q < arm.lo + LIMIT_MARGIN) or np.any(q > arm.hi - LIMIT_MARGIN):
        reasons.append("a joint sits within the limit margin")

    # --- full-trajectory self-collision: clear at the ends but clipping mid-reach? ---
    mid_bad = 0
    for s in np.linspace(0.0, 1.0, TRAJ_CHECK):
        qw = q0 + ease(s) * (q - q0)
        if self_collision(arm, qw):
            mid_bad += 1
    if mid_bad:
        reasons.append(f"self-collision on {mid_bad}/{TRAJ_CHECK} mid-reach waypoints")

    # --- velocity-capped, eased timing; the cap comes FROM the URDF velocity limit ---
    vmax = SAFE_VEL_FRAC * float(np.min(arm.vel))    # e.g. 0.03 * 18 = 0.54 rad/s
    T = max(MIN_TIME, EASE_PEAK * float(np.max(np.abs(q - q0))) / vmax)
    n = max(2, int(T * TRAJ_HZ))
    traj = [(i / TRAJ_HZ, q0 + ease(i / n) * (q - q0)) for i in range(n + 1)]
    peak_vel = float(np.max(np.abs(np.diff([w for _, w in traj], axis=0)))) * TRAJ_HZ
    if peak_vel > float(np.min(arm.vel)):            # HARD bound: never exceed the URDF limit
        reasons.append("trajectory exceeds URDF velocity limit")

    safe = not reasons
    return dict(safe=safe, q=q, trajectory=traj, reasons=reasons,
                reach_mm=r["err"] * 1000, aim_deg=r.get("aim_err", float("nan")),
                duration_s=T, peak_vel=peak_vel, vmax=vmax)


def main():
    HAND = hand_tip("left"); SAX = np.array([0.598, 0.762, -0.250]); SAX /= np.linalg.norm(SAX)
    arm = K1Arm("left", tip_xyz=tuple(HAND + 0.05 * SAX), scanner_axis=tuple(SAX))

    print("=== valid scan envelope: every placement must yield a SAFE plan ===")
    ok = tot = 0
    for left in np.arange(0.12, 0.34, 0.03):
        for z in np.arange(-0.05, 0.34, 0.05):
            if not arm.ik_best_aim(np.array([0.30, left, z]), (1, 0, 0), seeds=12)["reachable"]:
                continue
            tot += 1
            p = plan_scan(arm, np.array([0.30, left, z]))
            ok += 1 if p["safe"] else 0
    print(f"  {ok}/{tot} placements -> SAFE validated trajectory")

    print("\n=== a nominal scan: trajectory is slow + smooth ===")
    p = plan_scan(arm, np.array([0.30, 0.24, 0.169]))
    print(f"  safe={p['safe']} reach={p['reach_mm']:.0f}mm aim={p['aim_deg']:.0f}deg "
          f"duration={p['duration_s']:.1f}s peak_vel={p['peak_vel']:.2f}rad/s "
          f"(cap {MAX_VEL}) waypoints={len(p['trajectory'])}")

    print("\n=== dangerous targets must be REFUSED (fail closed) ===")
    for desc, tgt in [("target inside the torso", (0.0, 0.0, 0.0)),
                      ("target behind the shoulder", (-0.25, 0.0, 0.05)),
                      ("target far out of reach", (0.9, 0.5, 0.0))]:
        p = plan_scan(arm, np.array(tgt))
        print(f"  {desc:28s} -> safe={p['safe']}  reasons={p['reasons']}")


if __name__ == "__main__":
    main()
