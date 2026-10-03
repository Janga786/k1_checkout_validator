# k1_checkout_validator — Booster K1 cart-seek (reactive visual servo)

A ROS 2 (Humble) stack for the **Booster K1 humanoid**: detect a utility cart with the head
camera, walk to it and stop ~1 m away, using the Booster SDK velocity command (`Move`). An optional
`orbit` mode then circles the cart at a fixed standoff (off by default). Built to be safe to
supervise (gated start, hard-clamped speeds, stops that keep the robot standing).

The ROS package is `src/k1_qr_nav`. It started as a QR/barcode-seeking project, so the repo also
holds earlier modules (QR/barcode goal seeking, arm reach and scan, Gazebo and Nav2 simulation
launches) that the cart-seek path does not depend on.

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

Orbit after arriving: `./run_seek_cart_reactive.sh orbit:=true` (tangential speed capped at 0.10 m/s).

## Configuration

Scripts and defaults reference the robot host's layout (`/home/boosterk1/...`, the K1 camera at
`192.168.10.102`). Adjust the paths and addresses for your own setup before running.

## Tests (offline, no robot)

```bash
python3 src/k1_qr_nav/test/test_cart_approach.py   # control-law closed-loop sim
python3 src/k1_qr_nav/test/test_cart_track.py      # drift-safety: lost-stop + spin guard
```

See `DEBUG_REPORT.md` for the full design audit and `SEEK_CART.md` for details.
