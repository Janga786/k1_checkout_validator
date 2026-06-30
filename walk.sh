#!/usr/bin/env bash
# Tell the K1 to START WALKING to the cart. Run AFTER go_cart.sh is up.
# Keep your hand on the strap; estop.sh is ready in another terminal.
# The bridge runs kPrepare -> (2 s settle) -> kWalking, then seeks the cart at <=0.20 m/s.
unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'conda|miniconda' | paste -sd:)"
source /opt/ros/humble/setup.bash 2>/dev/null
echo ">>> START WALKING (kPrepare -> kWalking -> seek cart, hard-capped <=0.20 m/s)"
ros2 topic pub --once /start_walking std_msgs/msg/Bool "{data: true}"
