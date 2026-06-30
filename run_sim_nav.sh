#!/usr/bin/env bash
# Run the SLAM + Nav2 navigate-to-QR sim. Strips conda so ROS2 uses system
# python 3.10 (rclpy is built for it). Pass-through args, e.g.:
#   ./run_sim_nav.sh                 # full demo with RViz + K1 model
#   ./run_sim_nav.sh rviz:=false     # headless
#   ./run_sim_nav.sh model:=false    # skip the URDF model
unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'conda|miniconda' | paste -sd:)"
export DISPLAY="${DISPLAY:-:1}"
source /opt/ros/humble/setup.bash
source /home/boosterk1/k1_qr_nav_ws/install/setup.bash
exec ros2 launch k1_qr_nav sim_nav.launch.py "$@"
