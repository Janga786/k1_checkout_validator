#!/usr/bin/env bash
# Full pipeline: SLAM + Nav2 navigate the K1 to the QR, then its arm scans it.
# Strips conda so ROS2 uses system python 3.10 (numpy 1.21 runs the IK fine).
#   ./run_sim_nav_scan.sh                 # RViz
#   ./run_sim_nav_scan.sh rviz:=false     # headless (logs the phases)
unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'conda|miniconda' | paste -sd:)"
export DISPLAY="${DISPLAY:-:1}"
source /opt/ros/humble/setup.bash
source /home/boosterk1/k1_qr_nav_ws/install/setup.bash
exec ros2 launch k1_qr_nav sim_nav_scan.launch.py "$@"
