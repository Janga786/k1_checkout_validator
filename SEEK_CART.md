# Booster K1 — detect the cart, then walk to it (stop 1 m away)

SLAM/Nav2 driving the **real K1** with the **SDK velocity command** (`B1LocoClient.Move`): the head
camera finds the utility cart (YOLO-World, zero-shot), and the robot **walks** to a **1 m standoff**
in front of it, turning its **head** to keep the cart in view. Built on the `k1_qr_nav` SLAM+Nav2
stack — the same as seek-barcode, with a cart detector and a 1 m standoff.

```
 head cam (:8080/frame.jpg)
        │
   cart_detect_server.py  (navila env, YOLO-World "cart" + temporal gate)  ──HTTP──► :8090/detections.json
        │                                                                        │
        │                                                  cart_locate  (rclpy, torch-free)
        │                                                      │  (u,v)+intrinsics, range from cart height → 3D
        │                                           /cart_detection      /cart_bearing
        │                                                      │              │
        │                                                  qr_goal        head_search
        │                                             1 m standoff goal   sweep→track→hold
        │                                                      │              │
        │                                                    Nav2 ──/cmd_vel──┤  /head_cmd
        │                                                                     ▼
        └────────────────────────────────────────  booster_bridge  ◄─────────┘
                          Move(vx,vy,vyaw)=WALK (SDK velocity)   RotateHead(pitch,yaw)=HEAD
                          leg odom (rt/odometer_state) → /odom → SLAM/Nav2
```

## What's new vs seek-barcode
| piece | role |
|------|------|
| `cart_detect_server.py` | YOLO-World open-vocab **"cart"** on the head cam → :8090/detections.json (bbox/score/stable), annotated /frame.jpg. Runs in the **navila env** (torch/CUDA), NOT a ROS node. |
| `cart_locate` | poll the detect-server → best **stable** cart box → bearing (exact) + **range from the cart's apparent height** (view-stable for a known object) → `/cart_detection` (3D) + `/cart_bearing` |
| `seek_cart.launch.py` | wires it with **`standoff:=1.0`**; remaps `qr_goal`←`/cart_detection`, `head_search`←`/cart_bearing` |

**Reused unchanged:** `booster_bridge` (the **SDK `Move` velocity** driver + clamps/watchdog/e-stop FSM),
`qr_goal` (standoff goal, re-issues as the cart refines), `head_search`, `odom_tf`, Nav2, optional
`slam_toolbox`, optional `depthimage_to_laserscan`. Driving is **only** `Move(vx,vy,vyaw)` — the
built-in walker — never the trained policy or low-level joints.

## Run it
```bash
cd ~/k1_qr_nav_ws

# Self-contained DEMO, NO robot + NO detector: walk to a VIRTUAL cart at (target_x,target_y),
# stopping 1 m away. Best first run. (rviz:=true to watch.)
./run_seek_cart.sh dry_run:=true fake_target:=true rviz:=true

# Workstation, NO robot, REAL detector on a feed (SRC overrides the head-cam URL):
SRC=file:///home/boosterk1/Desktop/Cart.jpeg ./run_seek_cart.sh dry_run:=true

# REAL robot, odom-frame nav (drift-free over the short walk to a visible cart):
./run_seek_cart.sh net:=eno1

# + depth->scan obstacle avoidance + slam_toolbox mapping + RViz:
./run_seek_cart.sh net:=eno1 use_scan:=true slam:=true rviz:=true
```
Env knobs: `DETECT=0` skips the detector · `SRC=<url>` head-cam feed · `standoff:=1.0` stop distance
(m) · `real_height:=0.90` cart height for monocular ranging · `max_vx:=0.15` speed cap.

## Real-robot bring-up — SAFETY (same FSM as seek-barcode)
Launching does **NOT** move the robot — it comes up **IDLE** and waits. `booster_bridge` is the safety
authority (the SDK does no limit checking).
```
IDLE --/start_walking--> PREPARING --(2 s)--> WALKING --/cmd_vel--> walk
any  --/estop--> ESTOP  (Move 0 + kPrepare — stops & STANDS, balancing; NEVER limp)
any  --/damp---> DAMPED (kDamping = LIMP — ONLY when held/hoisted)
```
Order: (a) dry-run `fake_target:=true rviz:=true` first; (b) on the robot, `net:=eno1` (+`DETECT=0`),
hand-send tiny `/cmd_vel` and watch `/odom` for the sign convention BEFORE `/start_walking`;
(c) enable the detector, let `head_search` sweep **body-stationary** to confirm cart detection +
tracking; (d) `/start_walking` for a short seek at `max_vx:=0.15` in clear space; (e) add
`use_scan:=true` before any clutter. Speed caps default low (`max_vx 0.20`); geofence auto-stops after
`max_travel` (5 m); watchdog zeroes velocity if `/cmd_vel` is stale >0.5 s.

## Range / how the 1 m standoff converges
`cart_locate` always has an **exact bearing** (box centre). For **range** it uses the cart's apparent
**height** vs `real_height` (0.9 m) — height is far more view-stable than width for a cart. As the
robot approaches, the box grows → range shrinks → `qr_goal` re-issues the 1 m standoff, so the final
stop distance is self-correcting. Wire true metric **depth** (`use_scan:=true` + a depth topic) for a
one-shot exact range and Nav2 obstacle avoidance.

## Verified (2026-06-29, dry-run on the workstation, no robot)
- **Detector:** YOLO-World on the cart photo → cart @ **score 0.89, stable**, bbox [18,489,1344,1386],
  10.6 fps; correctly above the background clutter (0.11–0.13). Zero-shot, no training.
- **Closed loop:** virtual cart at map (2.5, 0.6) → `qr_goal` computed the goal at (1.53, 0.37)
  = **exactly 1.00 m in front of the cart, facing it** → Nav2 drove (`/cmd_vel`→`booster_bridge`
  simulated `/odom`) → **REACHED**. No node errors.

On-robot items (need the K1): the real SDK `Move`/`RotateHead`/leg-odom over DDS (`net:=eno1`), the
live head-cam feed for the detector, and (optional) depth wiring. The bridge is the only piece that
talks to the SDK.
