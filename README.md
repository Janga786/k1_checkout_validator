# k1_qr_nav — Booster K1 cart-seek (reactive visual servo)

A ROS 2 (Humble) stack for the **Booster K1 humanoid**: detect a utility cart with the head
camera, walk to it and stop ~1 m away, using the Booster SDK velocity command (`Move`). Built
to be safe to supervise (gated start, hard-clamped speeds, stops that keep the robot standing).

## How it works

```
head camera ──HTTP──▶ cart_detect_server (YOLO-World)  ──▶  cart_locate  ──▶  cart_track  ──▶  booster_bridge ──SDK──▶ K1
              (:8080/frame.jpg)        /detections.json     /cart_detection      /cmd_vel        Move(vx,vy,vyaw)
```

- **`cart_detect_server.py`** — open-vocab YOLO-World "cart" detector (runs in its own torch env), serves boxes over HTTP.
- **`cart_locate`** — turns boxes into a body-relative bearing + monocular range; gates stale/frozen frames.
- **`cart_track`** — odom-frame tracker: locks the cart in the robot's `/odom` frame from several agreeing
  detections, then drives to the standoff preferring the live (drift-free) camera bearing/range, with a
  **lost-target STOP** and a **spin guard**. (`cart_approach` is the simpler instantaneous-servo fallback.)
- **`booster_bridge`** — the only SDK-facing node: `/cmd_vel → Move`, `/head_cmd → RotateHead`, leg odom → `/odom`.
  Gated (no motion until `/start_walking`), hard speed/joint clamps, and a watcher that **never autonomously
  changes the robot's mode** (won't stand a damped robot back up).

## Run it (real robot)

```bash
./go_cart.sh        # bring up camera + detector + stack — robot stays IDLE (no motion)
./walk.sh           # start walking to the cart (≤0.20 m/s)  — hand on the strap
./estop.sh          # stop; robot stays standing, never limp
```

The head-camera HTTP bridge (`/booster_video_stream → :8080`) comes up via `bringup_camera.sh`
(lives in the `k1-vlm-navigation` deploy tree on the robot host).

## Tests (offline, no robot)

```bash
python3 src/k1_qr_nav/test/test_cart_approach.py   # control-law closed-loop sim
python3 src/k1_qr_nav/test/test_cart_track.py      # drift-safety: lost-stop + spin guard
```

See `DEBUG_REPORT.md` for the full design audit and `SEEK_CART.md` for details.
