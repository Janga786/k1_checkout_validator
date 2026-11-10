# k1_checkout_validator — Booster K1 cart-seek (reactive visual servo)

A ROS 2 (Humble) stack for the **Booster K1 humanoid**: the head camera finds a utility cart with
YOLO-World, the robot walks to it and stops about 1 m away, and it can then circle the cart at a
fixed standoff. It drives the robot only through the Booster SDK velocity command (`Move`), so the
walking itself is Booster's built-in walker.

**What I built:** the cart detector service, the perception-to-command nodes (`cart_locate`,
`cart_track`, `cart_approach`), the gated SDK bridge (`booster_bridge`), the launch and safety
scripts, and the offline tests. Fort Lewis College, summer 2026 (advisor: Dr. Yiyan Li).

## What ran on the real robot

- **Demonstrated on hardware:** approach, stop at the ~1 m standoff, orbit around the cart, and a
  QR-code read on the cart. About 25 operator-supervised live runs in July 2026.
- **Evidence in this repo:** the code, the offline tests, and the design audit. Run logs and video
  are not in this repository; there is no success rate claimed here.
- **QR read:** no QR decoder is included in this repository, so the code that decoded the QR code
  during those runs is not here. This repo covers detection, approach, stop and orbit.

## Default configuration

`go_cart.sh` launches `seek_cart_reactive.launch.py` with the odom-frame tracker
(`tracker:=odom`, i.e. `cart_track`), a 1.0 m standoff, and **orbit off** (`orbit:=false`). The
robot comes up idle and does not move until `walk.sh` is run. Orbit is enabled with
`orbit:=true` (tangential speed capped at 0.10 m/s):

```bash
./run_seek_cart_reactive.sh net:=eno1 orbit:=true
```

## How it works

```
head camera ──HTTP──▶ cart_detect_server (YOLO-World)  ──▶  cart_locate  ──▶  cart_track  ──▶  booster_bridge ──SDK──▶ K1
              (:8080/frame.jpg)        /detections.json     /cart_detection      /cmd_vel        Move(vx,vy,vyaw)
```

- **`cart_detect_server.py`**: open-vocabulary YOLO-World "cart" detector (runs in its own torch
  env), serves boxes over HTTP.
- **`cart_locate`**: turns boxes into a body-relative bearing and a monocular range; gates stale
  or frozen frames.
- **`cart_track`** (default): odom-frame tracker. It locks the cart in the robot's `/odom` frame
  from several agreeing detections, then drives to the standoff, preferring the live camera
  bearing and range when they are fresh. It has a **lost-target stop** and a **spin guard**, and
  runs the optional orbit. (`cart_approach` is the simpler instantaneous-servo fallback,
  `tracker:=servo`.)
- **`booster_bridge`**: the only SDK-facing node: `/cmd_vel → Move`, `/head_cmd → RotateHead`, leg
  odometry → `/odom`. No motion until `/start_walking`, hard speed and joint clamps, and a watcher
  that never changes the robot's mode on its own (it will not stand a damped robot back up).

## Run it (real robot)

```bash
./go_cart.sh        # bring up camera + detector + stack; robot stays IDLE (no motion)
./walk.sh           # start walking to the cart (≤0.20 m/s); hand on the strap
./estop.sh          # stop; robot stays standing, never limp
```

The head-camera HTTP bridge (`/booster_video_stream → :8080`) comes up via `bringup_camera.sh`
in the [`k1-vlm-navigation`](https://github.com/Janga786/k1-vlm-navigation) repository.

## Tests (offline, no robot, no ROS needed)

```bash
python3 src/k1_qr_nav/test/test_cart_approach.py   # control-law closed-loop sim (6 cases)
python3 src/k1_qr_nav/test/test_cart_track.py      # drift safety: lost-target stop + spin guard (3 cases)
```

## Configuration

Scripts and defaults reference the robot host's layout (`/home/boosterk1/...`, the K1 camera at
`192.168.10.102`). Adjust the paths and addresses for your own setup before running.

## Repository contents and history

The ROS package is `src/k1_qr_nav`. It started as a QR/barcode-seeking project, so the repo also
holds earlier modules (QR/barcode goal seeking, arm reach and scan, Gazebo and Nav2 simulation
launches) that the reactive cart-seek path does not use.

- [`DEBUG_REPORT.md`](DEBUG_REPORT.md): design audit of the earlier Nav2-based cart seek and the
  reasons for moving to the reactive controller.
- [`SEEK_CART.md`](SEEK_CART.md): the earlier Nav2/SLAM cart-seek design and its offline dry-run
  check.
- [`src/k1_qr_nav/SEEK_BARCODE.md`](src/k1_qr_nav/SEEK_BARCODE.md): the earlier barcode-seek stack.

## Limitations

- Live sessions were operator-supervised (hand on the strap, e-stop ready) at low speed caps.
- Range to the cart is monocular, from the cart's apparent height, so it depends on the assumed
  cart height and camera pitch.
- Walking is Booster's built-in controller through `Move`; no learned walking policy runs here.

## License

MIT. See [`LICENSE`](LICENSE).
