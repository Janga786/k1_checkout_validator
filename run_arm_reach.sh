#!/usr/bin/env bash
# Watch the K1 left arm reach to a barcode in RViz (full articulated URDF + IK).
# Strips conda so ROS2 uses system python 3.10 (numpy 1.21 runs the IK fine).
#   ./run_arm_reach.sh                 # RViz
#   ./run_arm_reach.sh rviz:=false     # headless (logs the IK reaches)
unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'conda|miniconda' | paste -sd:)"
export DISPLAY="${DISPLAY:-:1}"
source /opt/ros/humble/setup.bash
source /home/boosterk1/k1_qr_nav_ws/install/setup.bash
exec ros2 launch k1_qr_nav arm_reach.launch.py "$@"
