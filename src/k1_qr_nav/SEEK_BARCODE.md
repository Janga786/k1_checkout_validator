# Booster K1 — detect a barcode, then walk to it

SLAM/Nav2 driving the **real K1**: the head camera finds a barcode, the robot **walks**
to a standoff in front of it, turning its **head** to search and keep the barcode in view.
Built on the existing `k1_qr_nav` SLAM+Nav2 stack; the new layer is the robot itself.

```
 head cam (:8080/frame.jpg)
        │
   detect_server.py  (.venv, YOLO+temporal gate)  ──HTTP──►  :8090/detections.json
        │                                                          │
        │                                              barcode_locate  (rclpy, torch-free)
        │                                                  │  (u,v)+intrinsics → 3D
        │                                       /qr_detection      /barcode_bearing
        │                                                  │              │
        │                                              qr_goal        head_search
        │                                          standoff goal      sweep→track→hold
        │                                                  │              │
        │                                                Nav2 ──/cmd_vel──┤  /head_cmd
        │                                                                 ▼
        └────────────────────────────────────  booster_bridge  ◄─────────┘
                          Move(vx,vy,vyaw)=WALK   RotateHead(pitch,yaw)=HEAD
                          leg odom (rt/odometer_state) → /odom → SLAM/Nav2
```

## The new nodes

| node | what it does | SDK call |
|------|--------------|----------|
| `booster_bridge` | `/cmd_vel`→walk, `/head_cmd`→head, leg-odom→`/odom`; clamps + watchdog + e-stop | `Move`, `RotateHead`, `B1OdometerStateSubscriber`, `ChangeMode` |
| `barcode_locate` | detect-server boxes → `/qr_detection` (3D) + `/barcode_bearing` | — (torch-free; reads HTTP) |
| `head_search` | SWEEP to find a barcode → TRACK to keep it framed → HOLD on arrival | publishes `/head_cmd` |

Reused unchanged: `odom_tf`, `qr_goal` (standoff-facing goal), Nav2, optional `slam_toolbox`,
optional `depthimage_to_laserscan`. The detector stays in the **receipt_validator .venv**
(torch+CUDA) and is reached over HTTP — it is *not* a ROS node, because `rclpy` is system
python 3.10 and torch is not. The SDK binding is `cpython-310`, so it *does* share the
`rclpy` interpreter (one process holds both).

## Run it

```bash
# Self-contained DEMO, NO robot + NO detector: the bridge sims /odom and the robot walks
# to a VIRTUAL barcode at (target_x,target_y). Add rviz:=true to watch it. Best first run.
./run_seek_barcode.sh dry_run:=true fake_target:=true rviz:=true

# Workstation, NO robot: the bridge integrates /cmd_vel into a simulated /odom so the
# whole graph runs. (Detector idles without a feed; that's expected.)
./run_seek_barcode.sh dry_run:=true

# REAL robot, odom-frame nav (drift-free over the short walk to a visible barcode):
./run_seek_barcode.sh net:=<iface>            # iface = the DDS net interface to the robot

# + depth->scan obstacle avoidance, + slam_toolbox mapping, + RViz:
./run_seek_barcode.sh net:=<iface> use_scan:=true slam:=true rviz:=true
```

`SRC=<url>` overrides the head-cam feed for the detector (default `http://192.168.10.102:8080/frame.jpg`);
`DETECT=0` skips auto-starting it.

## Real-robot bring-up — SAFETY

The Booster SDK does **no** reachability/limit checking; `booster_bridge` is the safety
authority and runs an explicit mode state machine. **Launching the stack does NOT move the
robot** — on the real robot it comes up `IDLE` and waits for you.

```
IDLE --/start_walking--> PREPARING --(2 s settle)--> WALKING --/cmd_vel--> walk
WALKING --/soft_stop--> HALTED  (stops, stays in kWalking, KEEPS balancing) --/start_walking--> WALKING
any --/estop--> ESTOP   (Move 0 + kPrepare -- stops & STANDS, balancing; NEVER limp)
any --/damp---> DAMPED  (Move 0 + kDamping = LIMP/sag -- ONLY when held/hoisted)
```

**Stops never drop the robot.** `kDamping` makes a biped go limp and fall, so it is **not** the
emergency stop. `/estop` halts the gait (the controller balances in place) and settles to the
`kPrepare` standing stance — upright. `kDamping` is reachable only via the explicit `/damp`, for
a robot already held/hoisted. Shutdown also leaves it standing in `kPrepare`, never limp.

1. **Clear, flat, obstacle-free space**, + a person on the stop. In the default mode there is
   no depth, so it walks **blind** (see "Range / depth").
2. **The stops:**
   ```bash
   ros2 topic pub --once /soft_stop std_msgs/msg/Bool "{data: true}"   # pause: stay in kWalking, balancing
   ros2 topic pub --once /estop     std_msgs/msg/Bool "{data: true}"   # EMERGENCY: stop + kPrepare (stands, balancing)
   ros2 topic pub --once /damp      std_msgs/msg/Bool "{data: true}"   # LIMP (kDamping) -- ONLY when held/hoisted
   ```
   `/estop` is the safe emergency — it stays upright. `/damp` is the only thing that makes it limp.
3. **Start is GATED.** The robot stays `IDLE` until:
   ```bash
   ros2 topic pub --once /start_walking std_msgs/msg/Bool "{data: true}"
   ```
   That runs `kPrepare`→(settle)→`kWalking` **with return-code checks** — a failed `ChangeMode`
   aborts and stays put. `auto_walk:=true` removes the gate (not advised on hardware).
4. **Speed caps** — low by default, raise once verified: `max_vx 0.20`, back `0.10`, `vy 0.10`,
   `vyaw 0.30`. Set at launch: `max_vx:=0.15`. Lower Nav2's `desired_linear_vel` in
   `nav2_params_booster.yaml` to match for smooth motion.
5. **Geofence:** auto `/soft_stop` after `max_travel` (default 5 m) from the start pose — a
   runaway / bad-localization guard. `-p max_travel:=…` (0 disables).
6. **Watchdog:** zeroes velocity if `/cmd_vel` is stale > 0.5 s. **Head limits:** pitch
   [-0.40, 1.00], |yaw| ≤ 0.95 rad. **Monitor:** `ros2 topic echo /booster_state` (latched).

First bring-up, in order — **do not skip the sign check:**
(a) `dry_run:=true fake_target:=true rviz:=true` — watch it walk to a virtual barcode, no robot.
(b) On the robot, launch with `net:=<iface>` (+ `DETECT=0`); it comes up `IDLE`. Hand-send tiny
`/cmd_vel` and **watch `/odom` to confirm the sign conventions + that it steps the way you
expect** — *before* `/start_walking`.
(c) Enable the detector; let `head_search` sweep with the **body stationary** to confirm
detection + tracking.
(d) `/start_walking` for a short autonomous seek at `max_vx:=0.15` in the clear space.
(e) Add `use_scan:=true` (depth → obstacle layer) before any cluttered area.

## Range / depth

`barcode_locate` always has an exact **bearing**; for **range** it uses `assumed_range`
(default 1.5 m) and lets re-detection refine the goal as the robot approaches (`qr_goal`
re-issues when the target moves). Wire metric depth by enabling `use_scan:=true` with a real
depth topic (`depth_topic`, `info_topic`) — that also feeds Nav2's obstacle layer and SLAM.

## Verified (2026-06-26, dry-run on the workstation, no robot)

- `booster_bridge`: `/cmd_vel` → `/odom` integration (x reached 0.60 m on a 0.4 m/s command);
  `/head_cmd [0.2,0.5]` → `RotateHead(+0.20,+0.50)`; clamps + watchdog + dry-run banner.
- `barcode_locate`: a stand-in detect-server box at (800,360) in a 1280×720 frame →
  `/qr_detection` `(0.42, 0.00, 1.50)` in `head_camera_optical`, `/barcode_bearing` `u=+0.25`.
- `head_search`: SWEEP (yaw sweeps at pitch 0.25) → TRACK on a bearing → HOLD on `/nav_arrived`.
- **Closed loop:** truth target at world (2.5, 0.6) → `drive_to_qr` → `booster_bridge` walk →
  **REACHED at range 0.75 m**, odom (1.78, 0.40) = the standoff point. (The Nav2 path uses the
  same `/cmd_vel`↔`/odom` seam, already proven end-to-end in the gz `run_gazebo.sh` demo.)

On-robot items (need the K1): the real SDK `Move`/`RotateHead`/leg-odom over DDS, the live
detector feed, and depth wiring. The bridge is the only piece that talks to the SDK.
```
