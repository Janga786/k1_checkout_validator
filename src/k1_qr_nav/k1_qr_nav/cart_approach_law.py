#!/usr/bin/env python3
"""Pure (no-ROS) control law for cart_approach -- the SINGLE SOURCE OF TRUTH.

Kept free of rclpy/ROS imports so the exact same law can be imported by:
  * the node          (cart_approach.py)
  * the unit test     (test/test_cart_approach.py)
  * the visualiser    (viz_cart_approach.py, runs in a conda env without rclpy)
"""


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def approach_cmd(bear, rng, *, standoff, k_yaw, k_fwd, vyaw_cap, vx_cap,
                 center_thresh, yaw_sign, min_approach=0.0):
    """PURE control law.

    bear: bearing to the cart (rad), + = cart to the robot's RIGHT.
    rng : range to the cart (m).
    Returns (vx, vyaw). Turn toward the cart always; walk forward only when roughly centred
    and still outside the standoff. The forward term is proportional (k_fwd*(rng-standoff)) so it
    tapers near the goal, but is floored at min_approach so it does NOT creep infinitely slowly --
    the node latches 'arrived' within a tolerance band, so this floor stops cleanly. yaw_sign sets
    the turn convention (-1 => +bearing(right) -> negative vyaw, i.e. turn right under REP-103
    CCW-positive); flip to +1 if the robot turns the wrong way.
    """
    vyaw = _clamp(yaw_sign * k_yaw * bear, -vyaw_cap, vyaw_cap)
    vx = 0.0
    if abs(bear) < center_thresh and rng > standoff:
        vx = _clamp(k_fwd * (rng - standoff), 0.0, vx_cap)
        if 0.0 < vx < min_approach:
            vx = min_approach
    return vx, vyaw
