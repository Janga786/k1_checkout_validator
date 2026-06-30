#!/usr/bin/env python3
"""Visualise cart_approach: top-down closed-loop simulation of the REAL control law.

Drives a unicycle robot (the same integration booster_bridge uses) with cart_approach_law's
approach_cmd through a simulated camera (FOV gate + EMA filter + optional measurement noise),
exactly as the node would, and renders:
  * cart_approach_paths.png       -- 4 scenarios, top-down trajectories to the standoff
  * cart_approach_timeseries.png  -- range / vx / vyaw vs time for the main scenario
  * cart_approach.gif             -- animated approach (robot, heading, FOV cone, standoff ring)

Run with the navila conda python (has matplotlib/numpy/PIL):
  /home/boosterk1/miniconda3/envs/navila/bin/python viz_cart_approach.py
"""
import math
import os
import random
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle, Polygon
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "src", "k1_qr_nav"))
from k1_qr_nav.cart_approach_law import approach_cmd          # the SAME law the robot runs

PARAMS = dict(standoff=1.0, k_yaw=1.2, k_fwd=0.35, vyaw_cap=0.30, vx_cap=0.18,
              center_thresh=0.20, yaw_sign=-1.0, min_approach=0.06)
STANDOFF = PARAMS["standoff"]
ARRIVE_TOL, REARM, EMA_A, LOST_TO = 0.10, 0.40, 0.4, 1.0
FOV_HALF = math.radians(48.4)        # ~96.8 deg head camera
DT = 0.1
OUT = os.path.expanduser("~/Desktop")
os.makedirs(OUT, exist_ok=True)


def simulate(cart, start=(0.0, 0.0, 0.0), noise=0.0, seed=1, steps=900):
    """Closed-loop sim mirroring cart_approach._det_cb + _tick. Returns recorded arrays."""
    random.seed(seed)
    x, y, th = start
    cx, cy = cart
    f_b = f_r = None
    last_seen = None
    arrived = False
    rec = {k: [] for k in ("t", "x", "y", "th", "vx", "vyaw", "rng", "bear", "fr", "seen", "state")}
    for i in range(steps):
        dx, dy = cx - x, cy - y
        forward = dx * math.cos(th) + dy * math.sin(th)
        right = dx * math.sin(th) - dy * math.cos(th)
        rng = math.hypot(dx, dy)
        bear = math.atan2(right, forward)
        seen = forward > 0.0 and abs(bear) < FOV_HALF
        if seen:                                   # _det_cb: gate(trivial) + EMA
            r_meas = rng * (1.0 + random.uniform(-noise, noise))
            b_meas = bear + random.uniform(-noise, noise) * 0.3
            f_b = b_meas if f_b is None else (1 - EMA_A) * f_b + EMA_A * b_meas
            f_r = r_meas if f_r is None else (1 - EMA_A) * f_r + EMA_A * r_meas
            last_seen = i

        fresh = last_seen is not None and (i - last_seen) * DT < LOST_TO
        vx = vyaw = 0.0
        state = "lost"
        if fresh and f_r is not None:              # _tick
            if arrived:
                if f_r > STANDOFF + REARM:
                    arrived = False
            if not arrived:
                if f_r <= STANDOFF + ARRIVE_TOL:
                    arrived = True
                    state = "arrived"
                else:
                    vx, vyaw = approach_cmd(f_b, f_r, **PARAMS)
                    state = "walking" if vx > 1e-6 else "turning"
            else:
                state = "arrived"

        for k, val in (("t", i * DT), ("x", x), ("y", y), ("th", th), ("vx", vx),
                       ("vyaw", vyaw), ("rng", rng), ("bear", bear), ("fr", f_r if f_r else 0.0),
                       ("seen", seen), ("state", state)):
            rec[k].append(val)

        x += vx * math.cos(th) * DT
        y += vx * math.sin(th) * DT
        th += vyaw * DT
        if state == "arrived" and len(rec["t"]) > 8 and rec["state"][-8] == "arrived":
            break                                  # hold a moment after arriving, then stop the sim
    rec = {k: (np.array(v) if k != "state" else v) for k, v in rec.items()}
    return rec, cart


def robot_tri(x, y, th, s=0.18):
    pts = [(s, 0), (-s * 0.6, s * 0.6), (-s * 0.6, -s * 0.6)]
    c, sn = math.cos(th), math.sin(th)
    return [(x + px * c - py * sn, y + px * sn + py * c) for px, py in pts]


def draw_scene(ax, rec, cart, title):
    cx, cy = cart
    ax.plot(rec["x"], rec["y"], "-", color="#1f77b4", lw=2, zorder=3, label="robot path")
    # heading arrows along the path
    for i in range(0, len(rec["x"]), max(1, len(rec["x"]) // 12)):
        ax.annotate("", xy=(rec["x"][i] + 0.16 * math.cos(rec["th"][i]),
                            rec["y"][i] + 0.16 * math.sin(rec["th"][i])),
                    xytext=(rec["x"][i], rec["y"][i]),
                    arrowprops=dict(arrowstyle="->", color="#1f77b4", alpha=0.5, lw=1))
    ax.add_patch(Circle((cx, cy), STANDOFF, fill=False, ls="--", ec="#2ca02c", lw=1.5, zorder=2))
    ax.plot([cx], [cy], "X", color="k", ms=12, mew=2, zorder=4, label="cart")
    ax.text(cx, cy + 0.18, "cart", ha="center", fontsize=8)
    ax.plot([rec["x"][0]], [rec["y"][0]], "o", color="#2ca02c", ms=8, zorder=5, label="start")
    ax.add_patch(Polygon(robot_tri(rec["x"][-1], rec["y"][-1], rec["th"][-1]),
                         closed=True, fc="#d62728", ec="k", zorder=6))
    final = rec["rng"][-1]
    ax.set_title(f"{title}\nstops {final:.2f} m from cart in {rec['t'][-1]:.0f}s", fontsize=10)
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")


def main():
    scenarios = [
        ("Cart straight ahead", (3.0, 0.0), 0.0),
        ("Cart to the LEFT (demo)", (2.5, 0.6), 0.0),
        ("Cart to the RIGHT", (2.5, -0.8), 0.0),
        ("Far, off-axis, +25% range noise", (5.0, -2.0), 0.25),
    ]
    runs = [(name, *simulate(cart, noise=n)) for name, cart, n in scenarios]

    # ---- Figure 1: 4 top-down trajectories ----
    fig, axes = plt.subplots(2, 2, figsize=(11, 10))
    for ax, (name, rec, cart) in zip(axes.ravel(), runs):
        draw_scene(ax, rec, cart, name)
    axes.ravel()[0].legend(loc="upper left", fontsize=8)
    fig.suptitle("Booster K1 cart_approach — reactive visual-servo (top-down sim, real control law)",
                 fontsize=13, y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    p1 = os.path.join(OUT, "cart_approach_paths.png")
    fig.savefig(p1, dpi=120); plt.close(fig)
    print("wrote", p1)

    # ---- Figure 2: time-series for the LEFT demo ----
    name, rec, cart = runs[1]
    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    ax[0].plot(rec["t"], rec["rng"], color="#1f77b4"); ax[0].axhline(STANDOFF, ls="--", color="#2ca02c")
    ax[0].set_ylabel("range (m)"); ax[0].text(rec["t"][-1], STANDOFF + 0.05, " standoff 1.0 m", color="#2ca02c", fontsize=8)
    ax[0].grid(alpha=0.3); ax[0].set_title(f"{name}: range, forward speed, turn rate vs time")
    ax[1].plot(rec["t"], rec["vx"], color="#d62728"); ax[1].set_ylabel("vx (m/s)"); ax[1].grid(alpha=0.3)
    ax[1].axhline(PARAMS["vx_cap"], ls=":", color="gray"); ax[1].text(0, PARAMS["vx_cap"] + 0.005, " cap 0.18", fontsize=8, color="gray")
    ax[2].plot(rec["t"], rec["vyaw"], color="#9467bd"); ax[2].set_ylabel("vyaw (rad/s)"); ax[2].set_xlabel("time (s)"); ax[2].grid(alpha=0.3)
    fig.tight_layout()
    p2 = os.path.join(OUT, "cart_approach_timeseries.png")
    fig.savefig(p2, dpi=120); plt.close(fig)
    print("wrote", p2)

    # ---- Figure 3: animated GIF of the LEFT demo ----
    name, rec, cart = runs[1]
    cx, cy = cart
    idx = list(range(0, len(rec["t"]), 2))          # ~2x subsample
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-0.5, cx + 1.2); ax.set_ylim(min(-0.5, cy - 1.5), max(1.8, cy + 1.5))
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.add_patch(Circle((cx, cy), STANDOFF, fill=False, ls="--", ec="#2ca02c", lw=1.5))
    ax.plot([cx], [cy], "X", color="k", ms=13, mew=2); ax.text(cx, cy + 0.18, "cart", ha="center", fontsize=9)
    ax.set_title("cart_approach: turn to the cart, walk in, stop at 1 m")
    trail, = ax.plot([], [], "-", color="#1f77b4", lw=1.5, alpha=0.7)
    fov = Wedge((0, 0), 3.0, 0, 0, color="#ffcc00", alpha=0.18)
    ax.add_patch(fov)
    tri = Polygon(robot_tri(0, 0, 0), closed=True, fc="#d62728", ec="k", zorder=6)
    ax.add_patch(tri)
    txt = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=10,
                  bbox=dict(boxstyle="round", fc="white", alpha=0.8))

    def upd(fi):
        i = idx[fi]
        x, y, th = rec["x"][i], rec["y"][i], rec["th"][i]
        trail.set_data(rec["x"][:i + 1], rec["y"][:i + 1])
        tri.set_xy(robot_tri(x, y, th))
        fov.set_center((x, y))
        fov.set_theta1(math.degrees(th - FOV_HALF)); fov.set_theta2(math.degrees(th + FOV_HALF))
        st = rec["state"][i]
        txt.set_text(f"t={rec['t'][i]:4.1f}s  range={rec['rng'][i]:.2f} m\n"
                     f"vx={rec['vx'][i]:.2f}  vyaw={rec['vyaw'][i]:+.2f}\nstate: {st.upper()}")
        return trail, tri, fov, txt

    anim = FuncAnimation(fig, upd, frames=len(idx), interval=120, blit=False)
    p3 = os.path.join(OUT, "cart_approach.gif")
    anim.save(p3, writer=PillowWriter(fps=10)); plt.close(fig)
    print("wrote", p3)
    print("DONE — final ranges:", {n: round(float(r["rng"][-1]), 2) for n, r, _ in runs})


if __name__ == "__main__":
    main()
