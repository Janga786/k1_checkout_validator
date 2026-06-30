#!/usr/bin/env bash
# Reliable Gazebo demo: the K1 drives to the QR panel (direct pursuit, no Nav2).
#   ./run_gazebo_drive.sh             # Gazebo GUI
#   ./run_gazebo_drive.sh gui:=false  # headless
unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'conda|miniconda' | paste -sd:)"
export DISPLAY="${DISPLAY:-:1}"
source /opt/ros/humble/setup.bash
source /home/boosterk1/k1_qr_nav_ws/install/setup.bash
exec ros2 launch k1_qr_nav gazebo_drive.launch.py "$@"
