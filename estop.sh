#!/usr/bin/env bash
# EMERGENCY STOP. Halts the gait immediately: Move(0,0,0) then kPrepare.
# The robot STOPS and STAYS STANDING (the controller balances) -- it does NOT go limp.
# (To deliberately make it limp -- only when it's held/hoisted -- use:  ... /damp ...)
unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'conda|miniconda' | paste -sd:)"
source /opt/ros/humble/setup.bash 2>/dev/null
echo ">>> E-STOP: stop + kPrepare (robot stays UPRIGHT, balancing)"
ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"
