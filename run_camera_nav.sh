#!/usr/bin/env bash
# Camera-only RGBD-SLAM nav in Gazebo (K1 has no LiDAR, just the head camera).
#   ./run_camera_nav.sh                          gz GUI + RViz
#   ./run_camera_nav.sh gui:=false rviz:=false   headless
#   ./run_camera_nav.sh visual_odom:=true        pure camera odometry
unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'conda|miniconda' | paste -sd:)"
hash -r
source /opt/ros/humble/setup.bash
cd /home/boosterk1/k1_qr_nav_ws
source install/setup.bash
exec ros2 launch k1_qr_nav camera_nav.launch.py "$@"
