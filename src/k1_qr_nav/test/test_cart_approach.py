#!/usr/bin/env python3
"""Closed-loop simulation test for cart_approach's PURE control law (approach_cmd).

No ROS spin, no robot: we place a cart in the world, drive a unicycle robot with the SAME
integration booster_bridge uses (x += vx*cos th, y += vx*sin th, th += vyaw), feed the
body-relative (bearing, range) -- through an EMA + a simulated camera FOV + optional
measurement noise -- into approach_cmd, and assert the robot reaches the standoff facing the
cart, turns the correct way, and does NOT move toward a cart it cannot see.

Run:  python3 test/test_cart_approach.py     (with ROS2 sourced so the module imports)
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from k1_qr_nav.cart_approach_law import approach_cmd

PARAMS = dict(standoff=1.0, k_yaw=1.2, k_fwd=0.35, vyaw_cap=0.30, vx_cap=0.18,
              center_thresh=0.20, yaw_sign=-1.0, min_approach=0.06)
ARRIVE_TOL = 0.10                 # node latches 'arrived' within standoff+this
FOV_HALF = math.radians(48.0)     # ~96.8 deg camera -> +-48 deg visible
DT = 0.1


def run(cart, start=(0.0, 0.0, 0.0), noise=0.0, seed=0, steps=4000):
    random.seed(seed)
    x, y, th = start
    cx, cy = cart
    f_b = f_r = None
    arrived = False
    turned_left_first = None      # record initial turn direction for sign checks
    for i in range(steps):
        dx, dy = cx - x, cy - y
        forward = dx * math.cos(th) + dy * math.sin(th)
        right = dx * math.sin(th) - dy * math.cos(th)
        rng_true = math.hypot(dx, dy)
        bear_true = math.atan2(right, forward)
        seen = forward > 0.0 and abs(bear_true) < FOV_HALF
        if seen:
            r_meas = rng_true * (1.0 + random.uniform(-noise, noise))
            b_meas = bear_true + random.uniform(-noise, noise) * 0.3
            f_b = b_meas if f_b is None else 0.6 * f_b + 0.4 * b_meas
            f_r = r_meas if f_r is None else 0.6 * f_r + 0.4 * r_meas

        if f_r is None:                       # never seen the cart -> stop (search off)
            vx, vyaw = 0.0, 0.0
        elif f_r <= PARAMS["standoff"] + ARRIVE_TOL:
            arrived = True
            vx, vyaw = 0.0, 0.0
        else:
            vx, vyaw = approach_cmd(f_b, f_r, **PARAMS)
        if turned_left_first is None and abs(vyaw) > 1e-6:
            turned_left_first = vyaw > 0.0

        x += vx * math.cos(th) * DT
        y += vx * math.sin(th) * DT
        th += vyaw * DT
        if arrived and rng_true <= PARAMS["standoff"] + 0.20:
            return dict(status="ARRIVED", rng=rng_true, t=i * DT, x=x, y=y, th=th,
                        turned_left_first=turned_left_first)
    return dict(status="TIMEOUT", rng=math.hypot(cx - x, cy - y), t=steps * DT, x=x, y=y, th=th,
                turned_left_first=turned_left_first)


def check(name, ok, detail):
    mark = "PASS" if ok else "FAIL"
    print(f"  [{mark}] {name}: {detail}")
    return ok


def main():
    allok = True
    print("cart_approach closed-loop control-law tests (standoff=1.0 m):")

    # 1) straight ahead -> arrives at ~1 m
    r = run((3.0, 0.0))
    allok &= check("straight-ahead", r["status"] == "ARRIVED" and 0.9 <= r["rng"] <= 1.2,
                   f"{r['status']} final_range={r['rng']:.2f} m in {r['t']:.0f}s")

    # 2) cart to the RIGHT (within FOV) -> turns RIGHT first (vyaw<0), then arrives
    r = run((3.0, -2.0))
    allok &= check("cart-right -> turns right", r["status"] == "ARRIVED"
                   and r["turned_left_first"] is False and 0.9 <= r["rng"] <= 1.2,
                   f"{r['status']} turned_left_first={r['turned_left_first']} final={r['rng']:.2f} m")

    # 3) cart to the LEFT (within FOV) -> turns LEFT first (vyaw>0), then arrives
    r = run((3.0, 2.0))
    allok &= check("cart-left -> turns left", r["status"] == "ARRIVED"
                   and r["turned_left_first"] is True and 0.9 <= r["rng"] <= 1.2,
                   f"{r['status']} turned_left_first={r['turned_left_first']} final={r['rng']:.2f} m")

    # 4) NOISY detections (30% range noise) -> EMA still converges to ~1 m
    r = run((4.0, -1.0), noise=0.30, seed=7)
    allok &= check("noisy-30pct", r["status"] == "ARRIVED" and 0.85 <= r["rng"] <= 1.25,
                   f"{r['status']} final_range={r['rng']:.2f} m in {r['t']:.0f}s")

    # 5) far + off to the side but in FOV -> converges
    r = run((6.0, -3.0))
    allok &= check("far-off-axis", r["status"] == "ARRIVED" and 0.9 <= r["rng"] <= 1.25,
                   f"{r['status']} final_range={r['rng']:.2f} m in {r['t']:.0f}s")

    # 6) cart OUTSIDE the FOV (cannot be seen) -> robot must NOT wander (stays ~put)
    r = run((1.0, 3.0), steps=300)
    wander = math.hypot(r["x"], r["y"])
    allok &= check("out-of-FOV -> no motion", r["status"] == "TIMEOUT" and wander < 0.05,
                   f"{r['status']} robot moved {wander:.3f} m (want ~0)")

    print("ALL PASS" if allok else "SOME FAILED")
    return 0 if allok else 1


if __name__ == "__main__":
    sys.exit(main())
