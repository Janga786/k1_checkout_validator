#!/usr/bin/env python3
"""Pure-Python closed-loop test for cart_track's DRIFT-SAFETY fixes.

Reproduces the 2026-06-30 real-robot failure: cart_track approached well, then near the cart it
LOST sight, kept driving on a DRIFTED odom pin, did a ~180 and walked away ("turned the right way
but didn't stop turning"). Verifies the new behavior STOPS instead:
  * lost-target timeout  -> STOP if the cart hasn't been seen for `lost_timeout` s (don't drive on drift)
  * camera-preferred ctrl -> use the live (drift-free) bearing/range when fresh
  * spin guard            -> STOP if it turns a lot with no forward progress

No ROS, no robot. Replicates cart_track._det_cb + _tick decision logic (keep in sync with the node).
Run:  python3 test/test_cart_track.py   (ROS2 sourced just so cart_approach_law imports)
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from k1_qr_nav.cart_approach_law import approach_cmd

DT = 0.1
FOV = math.radians(48.0)


class Track:
    """Mirror of cart_track's per-tick decision (the parts under test)."""
    def __init__(self, yaw_sign=-1.0, lost_timeout=2.0, turn_limit=3.5, cam_fresh=0.5):
        self.standoff = 1.0; self.arrive_tol = 0.12; self.alpha = 0.3
        self.establish_n = 3; self.establish_spread = 1.0; self.reject_dist = 1.5
        self.lost_timeout = lost_timeout; self.turn_limit = turn_limit; self.cam_fresh = cam_fresh
        self.yaw_sign = yaw_sign
        self.cart = None; self.cam_bear = None; self.cam_rng = None; self.last_fix_t = None
        self._turn_acc = 0.0; self.state = "LOCALIZING"; self._fixes = []

    def det(self, forward, right, rx, ry, rth, t):
        if forward <= 0:
            return
        a = self.alpha
        cb = math.atan2(right, forward); crng = math.hypot(right, forward)
        self.cam_bear = cb if self.cam_bear is None else (1 - a) * self.cam_bear + a * cb
        self.cam_rng = crng if self.cam_rng is None else (1 - a) * self.cam_rng + a * crng
        self.last_fix_t = t
        cx = rx + forward * math.cos(rth) + right * math.sin(rth)
        cy = ry + forward * math.sin(rth) - right * math.cos(rth)
        if self.cart is None:
            self._fixes.append((cx, cy, t)); recent = self._fixes[-self.establish_n:]
            if len(recent) >= self.establish_n:
                mx = sum(x for x, _, _ in recent) / len(recent); my = sum(y for _, y, _ in recent) / len(recent)
                if max(math.hypot(x - mx, y - my) for x, y, _ in recent) <= self.establish_spread:
                    self.cart = (mx, my); self.state = "APPROACH"; self._fixes = []
        else:
            if math.hypot(cx - self.cart[0], cy - self.cart[1]) <= self.reject_dist:
                self.cart = ((1 - a) * self.cart[0] + a * cx, (1 - a) * self.cart[1] + a * cy)

    def tick(self, rx, ry, rth, t):
        if self.cart is None or self.state == "ARRIVED":
            return 0.0, 0.0, "hold"
        dx = self.cart[0] - rx; dy = self.cart[1] - ry
        o_bear = math.atan2(dx * math.sin(rth) - dy * math.cos(rth), dx * math.cos(rth) + dy * math.sin(rth))
        o_rng = math.hypot(dx, dy)
        age = (t - self.last_fix_t) if self.last_fix_t is not None else 1e9
        if age > self.lost_timeout:
            self._turn_acc = 0.0
            return 0.0, 0.0, "lost-stop"
        if age < self.cam_fresh and self.cam_bear is not None:
            bear, rng = self.cam_bear, self.cam_rng
        else:
            bear, rng = o_bear, o_rng
        if self.state == "APPROACH" and rng <= self.standoff + self.arrive_tol:
            self.state = "ARRIVED"
            return 0.0, 0.0, "arrived"
        vx, vyaw = approach_cmd(bear, rng, standoff=self.standoff, k_yaw=1.2, k_fwd=0.35, vyaw_cap=0.30,
                                vx_cap=0.18, center_thresh=0.20, yaw_sign=self.yaw_sign, min_approach=0.06)
        self._turn_acc = 0.0 if vx > 1e-3 else self._turn_acc + abs(vyaw) * DT
        if self._turn_acc > self.turn_limit:
            self._turn_acc = 0.0
            return 0.0, 0.0, "spin-stop"
        return vx, vyaw, "approach"


def sim(cart, yaw_sign=-1.0, lose_at=None, drift_rate=0.0, steps=900):
    """True pose drives the camera; odom pose (with heading drift) drives cart_track."""
    tx = ty = tth = 0.0; ox = oy = oth = 0.0
    cx, cy = cart; trk = Track(yaw_sign=yaw_sign)
    last_det = -1.0; rec = []
    for i in range(steps):
        t = i * DT
        dx, dy = cx - tx, cy - ty
        fwd = dx * math.cos(tth) + dy * math.sin(tth); rgt = dx * math.sin(tth) - dy * math.cos(tth)
        rng_true = math.hypot(dx, dy)
        visible = fwd > 0 and abs(math.atan2(rgt, fwd)) < FOV and (lose_at is None or t < lose_at)
        if visible and (t - last_det) >= 0.11:
            trk.det(fwd, rgt, ox, oy, oth, t); last_det = t      # fused against the DRIFTED odom pose
        vx, vyaw, mode = trk.tick(ox, oy, oth, t)
        tx += vx * math.cos(tth) * DT; ty += vx * math.sin(tth) * DT; tth += vyaw * DT
        ox += vx * math.cos(oth) * DT; oy += vx * math.sin(oth) * DT
        oth += (vyaw + drift_rate) * DT
        rec.append((t, rng_true, mode, tx, ty))
        if mode == "arrived":
            return dict(status="ARRIVED", rng=rng_true, t=t, rec=rec)
    return dict(status="END", rng=rng_true, t=t, rec=rec)


def ck(name, ok, detail):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    return ok


def main():
    allok = True
    print("cart_track drift-safety tests:")

    # 1) normal (no drift, cart stays visible) -> still reaches the standoff
    r = sim((3.0, 0.6))
    allok &= ck("normal-approach", r["status"] == "ARRIVED" and 0.85 <= r["rng"] <= 1.25,
               f"{r['status']} final={r['rng']:.2f} m in {r['t']:.0f}s")

    # 2) THE BUG: lose sight near the cart + odom heading-drift -> must STOP, not drive away
    r = sim((3.0, 0.0), lose_at=9.0, drift_rate=0.06)
    at_loss = next((x[1] for x in r["rec"] if x[0] >= 9.0), r["rec"][-1][1])
    after = [x[1] for x in r["rec"] if x[0] >= 9.0]
    max_after = max(after) if after else at_loss
    saw_loststop = any(x[2] == "lost-stop" for x in r["rec"])
    # success = it did NOT run away (never got much farther from the cart than at the moment of loss)
    allok &= ck("lost-near-cart -> STOP (no drive-away)",
               saw_loststop and max_after <= at_loss + 0.6,
               f"dist@loss={at_loss:.2f} max_after={max_after:.2f} lost_stop={saw_loststop} "
               f"(old code would keep growing as it walked away)")

    # 3) spin guard: wrong turn sign makes it turn away -> must STOP (not spin/walk off forever)
    r = sim((3.0, 0.9), yaw_sign=+1.0)
    stopped = any(x[2] in ("lost-stop", "spin-stop") for x in r["rec"])
    final_xy = math.hypot(r["rec"][-1][3], r["rec"][-1][4])
    allok &= ck("wrong-turn -> STOP (spin/lost guard)", stopped and final_xy < 2.0,
               f"stopped={stopped} robot_traveled={final_xy:.2f} m (bounded, didn't walk off)")

    print("ALL PASS" if allok else "SOME FAILED")
    return 0 if allok else 1


if __name__ == "__main__":
    sys.exit(main())
