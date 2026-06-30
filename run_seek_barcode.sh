#!/usr/bin/env bash
# Booster K1: DETECT a barcode with the head cam, then WALK to it.
# SLAM/Nav2 + the SDK walk (Move) and head-turn (RotateHead).
#
#   ./run_seek_barcode.sh dry_run:=true            # workstation, NO robot (bridge sims /odom)
#   ./run_seek_barcode.sh net:=eth0                # REAL robot, odom-frame nav
#   ./run_seek_barcode.sh net:=eth0 use_scan:=true # + depth->scan obstacle avoidance
#   ./run_seek_barcode.sh net:=eth0 slam:=true use_scan:=true rviz:=true
#
# Env knobs:
#   DETECT=0       don't auto-start the YOLO detect-server (default 1)
#   SRC=<url>      robot head-cam feed for the detector (default robot :8080/frame.jpg)
#   SDK=<path>     booster SDK build dir (default ~/booster_robotics_sdk/build)
#
# The detector (YOLO/torch) runs in the receipt_validator .venv as its OWN process and is
# reached over HTTP at :8090 -- it is NOT a ROS node (torch lives in the venv, rclpy in
# system python 3.10; they can't share an interpreter). Everything else is ROS 2 Humble.
#
# NOTE: no `set -u` -- ROS's setup.bash references unbound vars (AMENT_TRACE_SETUP_FILES).
#
# SAFETY: on the REAL robot, keep the e-stop ready:  ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"
#         (latches kDamping). The bridge also zeroes velocity if /cmd_vel goes stale (watchdog).
SDK="${SDK:-/home/boosterk1/booster_robotics_sdk/build}"
SRC="${SRC:-http://192.168.10.102:8080/frame.jpg}"
VENV_PY="/home/boosterk1/receipt_validator/.venv/bin/python"
PIDFILE=/tmp/detect_server.pid

# clean up the detector on exit/Ctrl-C -- registered BEFORE we start anything
cleanup(){ [ -f "$PIDFILE" ] && kill "$(cat "$PIDFILE")" 2>/dev/null; pkill -f detect_server.py 2>/dev/null; rm -f "$PIDFILE"; }
trap cleanup EXIT INT TERM

# normalise True/False -> true/false so ros2 launch's IfCondition accepts them
ARGS=(); FAKE=0
for a in "$@"; do
  a="${a/:=True/:=true}"; a="${a/:=False/:=false}"
  a="${a/:=TRUE/:=true}"; a="${a/:=FALSE/:=false}"
  [ "$a" = "fake_target:=true" ] && FAKE=1
  ARGS+=("$a")
done
DEFAULT_DETECT=1; [ "$FAKE" = "1" ] && DEFAULT_DETECT=0   # virtual target -> no detector needed

# 1) detector (YOLO/torch) -- its OWN interpreter (.venv), independent of the ROS python
if [ "${DETECT:-$DEFAULT_DETECT}" = "1" ]; then
  pkill -f detect_server.py 2>/dev/null; sleep 0.3      # free :8090 from a previous run
  if [ -x "$VENV_PY" ]; then
    echo "[seek] detect-server (.venv)  src=$SRC  -> http://127.0.0.1:8090/detections.json"
    "$VENV_PY" /home/boosterk1/receipt_validator/detect_server.py --src "$SRC" --port 8090 \
        >/tmp/detect_server.log 2>&1 &
    echo $! > "$PIDFILE"
  else
    echo "[seek] WARN: $VENV_PY missing; start detect_server yourself, or run with DETECT=0"
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

echo "[seek] ros2 launch k1_qr_nav seek_barcode.launch.py ${ARGS[*]}"
ros2 launch k1_qr_nav seek_barcode.launch.py "${ARGS[@]}"   # foreground: the trap fires on exit
