#!/usr/bin/env bash
# Kill EVERYTHING from a gz/Nav2 run -- gz server, bridge, and all Nav2 + helper
# nodes. killall by process name (gz/ruby) plus pkill by install path for the
# python nodes. Run as a *file* so the pkill patterns never match this shell.
for p in gz-sim-server ruby gz parameter_bridge controller_server planner_server \
         smoother_server behavior_server bt_navigator waypoint_follower \
         velocity_smoother lifecycle_manager rtabmap rtabmap_viz rgbd_odometry component_container_isolated rviz2; do
  killall -9 "$p" 2>/dev/null
done
pkill -9 -f "ros2 launch k1_qr_nav" 2>/dev/null
pkill -9 -f "navigation_launch"     2>/dev/null
pkill -9 -f "install/k1_qr_nav"     2>/dev/null
pkill -9 -f "static_transform_publisher" 2>/dev/null
exit 0
