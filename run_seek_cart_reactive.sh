#!/usr/bin/env bash
# Booster K1: REACTIVE cart-seek -- detect the cart, WALK to it, stop `standoff` m away.
# Pure visual servo (cart_approach): head held FORWARD, turn body to centre the cart + walk in.
# NO Nav2 / NO SLAM / NO map-TF. Driving is the SDK Move via booster_bridge (GATED + hard-clamped).
#
#   ./run_seek_cart_reactive.sh dry_run:=true fake_target:=true     # offline CLOSED LOOP (no robot/detector)
#   ./run_seek_cart_reactive.sh net:=eno1                           # REAL robot, comes up IDLE (gated)
#   ./run_seek_cart_reactive.sh net:=eno1 standoff:=1.0 search_when_lost:=true
#
# Env knobs:
#   DETECT=0    don't auto-start the YOLO-World cart detect-server (default 1; auto-0 with fake_target)
#   SRC=<url>   robot head-cam feed (default robot :8080/frame.jpg)
#   DET_PY=<py> python for the detector (default navila env: has ultralytics YOLO-World)
#   SDK=<path>  booster SDK build dir (default ~/booster_robotics_sdk/build)
#
# SAFETY: comes up IDLE; gate with /start_walking (walk.sh). E-stop stays UPRIGHT (estop.sh).
SDK="${SDK:-/home/boosterk1/booster_robotics_sdk/build}"
SRC="${SRC:-http://192.168.10.102:8080/frame.jpg}"
DET_PY="${DET_PY:-/home/boosterk1/miniconda3/envs/navila/bin/python}"
DET_SRV="/home/boosterk1/k1_qr_nav_ws/cart_detect_server.py"
PIDFILE=/tmp/cart_detect_server.pid
HERE="$(cd "$(dirname "$0")" && pwd)"

cleanup(){ [ -f "$PIDFILE" ] && kill "$(cat "$PIDFILE")" 2>/dev/null; pkill -f cart_detect_server.py 2>/dev/null; rm -f "$PIDFILE"; }
trap cleanup EXIT INT TERM

# normalise launch args (tolerate the ':' typo and True/False), detect the virtual-cart demo
ARGS=(); FAKE=0
for a in "$@"; do
  case "$a" in *:=*) : ;; [A-Za-z_]*:*) echo "[reactive] note: '$a' -> '${a/:/:=}'"; a="${a/:/:=}" ;; esac
  a="${a/:=True/:=true}"; a="${a/:=False/:=false}"; a="${a/:=TRUE/:=true}"; a="${a/:=FALSE/:=false}"
  [ "$a" = "fake_target:=true" ] && FAKE=1
  ARGS+=("$a")
done
DEFAULT_DETECT=1; [ "$FAKE" = "1" ] && DEFAULT_DETECT=0     # virtual cart -> no detector needed

# ROS env: system python 3.10, conda stripped, SDK on PYTHONPATH for the bridge
unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'conda|miniconda' | paste -sd:)"
hash -r
source /opt/ros/humble/setup.bash
cd /home/boosterk1/k1_qr_nav_ws
source install/setup.bash
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$SDK"

# 1) HYGIENE FIRST -- kill orphaned nav procs + clean DDS BEFORE starting the detector.
#    (nav_cleanup's PAT matches cart_detect_server.py, so running it AFTER the detector would
#     immediately kill the detector -- the launch-1 blocker. Order matters: hygiene, THEN detector.)
echo "[reactive] pre-launch hygiene (kill stale procs + clean DDS) ..."
bash "$HERE/nav_cleanup.sh"

# 2) cart detector (YOLO-World/torch) in its OWN interpreter (navila env), started AFTER hygiene
if [ "${DETECT:-$DEFAULT_DETECT}" = "1" ]; then
  pkill -f cart_detect_server.py 2>/dev/null; sleep 0.3
  if [ -x "$DET_PY" ]; then
    echo "[reactive] cart detect-server (navila)  src=$SRC  -> http://127.0.0.1:8090/detections.json"
    "$DET_PY" "$DET_SRV" --src "$SRC" --port 8090 >/tmp/cart_detect_server.log 2>&1 &
    echo $! > "$PIDFILE"
  else
    echo "[reactive] WARN: $DET_PY missing; start cart_detect_server yourself, or run with DETECT=0"
  fi
fi

# 3) reactive stack (bridge + cart_locate + cart_approach). Comes up IDLE on the real robot.
echo "[reactive] ros2 launch k1_qr_nav seek_cart_reactive.launch.py ${ARGS[*]}"
ros2 launch k1_qr_nav seek_cart_reactive.launch.py "${ARGS[@]}"
