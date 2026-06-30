#!/usr/bin/env bash
# Booster K1: DETECT the utility cart with the head cam, then WALK to it, stopping 1 m away.
# SLAM/Nav2 + the SDK velocity command (Move) and head-turn (RotateHead).
#
#   ./run_seek_cart.sh dry_run:=true fake_target:=true rviz:=true  # NO robot/detector (best first run)
#   ./run_seek_cart.sh dry_run:=true                               # NO robot; detector idles w/o a feed
#   ./run_seek_cart.sh net:=eno1                                   # REAL robot, odom-frame nav
#   ./run_seek_cart.sh net:=eno1 use_scan:=true slam:=true rviz:=true
#
# Env knobs:
#   DETECT=0       don't auto-start the YOLO-World cart detect-server (default 1)
#   SRC=<url>      robot head-cam feed (default robot :8080/frame.jpg). Offline: file:///path/Cart.jpeg
#   DET_PY=<py>    python for the detector (default navila env: has ultralytics YOLO-World)
#   SDK=<path>     booster SDK build dir (default ~/booster_robotics_sdk/build)
#
# The detector (YOLO-World/torch) runs in the navila conda env as its OWN process, reached over
# HTTP at :8090 -- NOT a ROS node (torch in conda, rclpy in system python 3.10). Driving uses the
# SDK velocity command: Nav2 /cmd_vel -> booster_bridge -> B1LocoClient.Move(vx,vy,vyaw).
#
# SAFETY (REAL robot): comes up IDLE; gate with /start_walking. E-stop stays upright (kPrepare):
#   ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"
SDK="${SDK:-/home/boosterk1/booster_robotics_sdk/build}"
SRC="${SRC:-http://192.168.10.102:8080/frame.jpg}"
DET_PY="${DET_PY:-/home/boosterk1/miniconda3/envs/navila/bin/python}"
DET_SRV="/home/boosterk1/k1_qr_nav_ws/cart_detect_server.py"
PIDFILE=/tmp/cart_detect_server.pid

cleanup(){ [ -f "$PIDFILE" ] && kill "$(cat "$PIDFILE")" 2>/dev/null; pkill -f cart_detect_server.py 2>/dev/null; rm -f "$PIDFILE"; }
trap cleanup EXIT INT TERM

# normalise True/False -> true/false; detect if running the virtual-target demo
ARGS=(); FAKE=0
for a in "$@"; do
  # tolerate the common ':' typo for ':=' (fix only the first colon; skip args already using :=)
  case "$a" in *:=*) : ;; [A-Za-z_]*:*) echo "[seek-cart] note: '$a' -> '${a/:/:=}' (use := for launch args)"; a="${a/:/:=}" ;; esac
  a="${a/:=True/:=true}"; a="${a/:=False/:=false}"
  a="${a/:=TRUE/:=true}"; a="${a/:=FALSE/:=false}"
  [ "$a" = "fake_target:=true" ] && FAKE=1
  ARGS+=("$a")
done
DEFAULT_DETECT=1; [ "$FAKE" = "1" ] && DEFAULT_DETECT=0   # virtual cart -> no detector needed

# 1) cart detector (YOLO-World/torch) -- its OWN interpreter (navila env), independent of ROS python
if [ "${DETECT:-$DEFAULT_DETECT}" = "1" ]; then
  pkill -f cart_detect_server.py 2>/dev/null; sleep 0.3
  if [ -x "$DET_PY" ]; then
    echo "[seek-cart] cart detect-server (navila)  src=$SRC  -> http://127.0.0.1:8090/detections.json"
    "$DET_PY" "$DET_SRV" --src "$SRC" --port 8090 >/tmp/cart_detect_server.log 2>&1 &
    echo $! > "$PIDFILE"
  else
    echo "[seek-cart] WARN: $DET_PY missing; start cart_detect_server yourself, or run with DETECT=0"
  fi
fi

# 2) ROS stack -- system python 3.10, conda stripped, SDK on PYTHONPATH for the bridge
unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'conda|miniconda' | paste -sd:)"
hash -r
source /opt/ros/humble/setup.bash
cd /home/boosterk1/k1_qr_nav_ws
source install/setup.bash
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$SDK"

# Pre-launch hygiene: kill any ORPHANED nav/TF processes left by a prior run, then
# clear stale Fast-DDS shared memory + restart the daemon. Orphaned duplicates
# (esp. an old use_sim_time robot_state_publisher, or 6x duplicate static TF
# publishers) poison /tf and stop Nav2 activating -- the exact failure we hit.
# See nav_cleanup.sh for the full story. Safe: only touches host processes.
echo "[seek-cart] pre-launch hygiene (kill stale nav procs + clean DDS) ..."
bash /home/boosterk1/k1_qr_nav_ws/nav_cleanup.sh

echo "[seek-cart] ros2 launch k1_qr_nav seek_cart.launch.py ${ARGS[*]}"
ros2 launch k1_qr_nav seek_cart.launch.py "${ARGS[@]}"
