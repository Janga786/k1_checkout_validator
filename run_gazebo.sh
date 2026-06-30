#!/usr/bin/env bash
# Gazebo physics demo: the K1 navigates to a QR panel with the REAL Nav2 stack
# (NavFn planner + RPP controller + costmaps), localized on the drift-free gz
# odometry instead of SLAM. Strips conda so ROS2 uses system python 3.10.
#   ./run_gazebo.sh                          # Gazebo GUI + RViz
#   ./run_gazebo.sh gui:=false rviz:=false   # headless
# Stop with Ctrl-C; if any nodes linger afterwards, run ./nuke_ros.sh
unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'conda|miniconda' | paste -sd:)"
export DISPLAY="${DISPLAY:-:1}"
source /opt/ros/humble/setup.bash
source /home/boosterk1/k1_qr_nav_ws/install/setup.bash
exec ros2 launch k1_qr_nav gazebo_nav.launch.py "$@"
