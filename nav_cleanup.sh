#!/usr/bin/env bash
# ============================================================================
#  nav_cleanup.sh -- robust pre-launch hygiene for the K1 cart-seek stack.
#
#  WHY THIS EXISTS (root cause of the 2026-06-29 "didn't walk" failure):
#  A session of relaunches left a GRAVEYARD of orphaned host processes on the
#  ROS graph -- 6x duplicate static_transform_publishers, an OLD Gazebo
#  robot_state_publisher running on use_sim_time (timestamps frozen in the past),
#  and 11x depthimage_to_laserscan. That pile -- NOT the real robot's /tf --
#  poisoned TF (duplicate `base_link`/`map->odom` edges + stale `use_sim_time`
#  stamps => the `TF_OLD_DATA ... data from the past` flood) and starved Nav2's
#  activation/discovery, so `qr_goal` sat forever on:
#       waiting for TF to map ...
#       waiting for Nav2 navigate_to_pose action ...
#  The REAL robot roots its tree at `trunk_link` and names its camera
#  `head_color_optical_frame`; it does NOT use base_link/odom/map/Trunk, so our
#  nav frames never actually collided. The fix is simply: never let zombies pile up.
#
#  HOW: kill every prior host nav/TF/detector process by EXPLICIT PID (never a
#  bare `pkill -f <pattern>`, which matches this very shell's command line and
#  kills it mid-cleanup -- that bug is why earlier cleanups silently failed).
#  Then clear stale Fast-DDS shared memory + restart the daemon for clean discovery.
#
#  Safe: only touches HOST processes. The robot's own nodes run on the robot and
#  are never matched. Run this with NO stack up -- it does not move the robot.
# ============================================================================
set -u

# Source ROS if the caller hasn't (needed only for the `ros2 daemon` step).
command -v ros2 >/dev/null 2>&1 || source /opt/ros/humble/setup.bash 2>/dev/null || true

ME=$$
# Host binaries this stack spawns, matched against /proc cmdline. Anchored on the
# install path (e.g. opt/ros/humble/lib/nav2_*) so it can't match unrelated procs.
PAT='opt/ros/humble/lib/(nav2|tf2_ros|robot_state_publisher)|k1_qr_nav/lib/k1_qr_nav|depthimage_to_laserscan|slam_toolbox|component_container|rviz2|cart_detect_server\.py'

# --- graceful stop of any LIVE booster_bridge BEFORE the kill -9 sweep ---
# booster_bridge is the SDK-facing driver. A bare kill -9 orphans it mid-walk with no
# safe_stop(), so the robot is never commanded to a standing stop. First send /estop (Move0+
# kPrepare via the callback while it's alive) and SIGINT (which trips the bridge's
# `finally: safe_stop()` -> Move(0,0,0)+kPrepare, robot STAYS STANDING). Only then kill -9.
BRIDGE_PIDS=$(ps -eo pid,cmd | grep -E "k1_qr_nav/lib/k1_qr_nav/booster_bridge" | grep -v grep | awk -v me="$ME" '$1!=me{print $1}')
if [ -n "$BRIDGE_PIDS" ]; then
  echo "[nav_cleanup] live booster_bridge ($(echo $BRIDGE_PIDS|tr '\n' ' ')) -> /estop + SIGINT (safe_stop: stays STANDING) before kill -9"
  timeout 6 ros2 topic pub --once /estop std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true
  kill -INT $BRIDGE_PIDS 2>/dev/null || true     # trips the bridge's finally: safe_stop()
  for _ in 1 2 3 4 5 6; do                        # wait up to ~3s for a clean stop+exit
    sleep 0.5
    kill -0 $BRIDGE_PIDS 2>/dev/null || break
  done
fi

killed=0
for _ in 1 2 3; do
  # collect PIDs first, exclude THIS shell ($ME) and the grep, THEN kill by PID
  PIDS=$(ps -eo pid,cmd | grep -E "$PAT" | grep -v grep | awk -v me="$ME" '$1!=me{print $1}')
  [ -z "$PIDS" ] && break
  kill -9 $PIDS 2>/dev/null
  killed=$((killed + $(echo "$PIDS" | wc -w)))
  sleep 1
done

# Stale Fast-DDS shared memory + daemon -> clean discovery. timeout-guarded: the
# daemon can wedge on a polluted graph, so it must never block the launch.
timeout 12 ros2 daemon stop  >/dev/null 2>&1 || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fast_datasharing_* 2>/dev/null || true
timeout 12 ros2 daemon start >/dev/null 2>&1 || true
sleep 1

left=$(ps -eo pid,cmd | grep -E "$PAT" | grep -v grep | awk -v me="$ME" '$1!=me' | wc -l)
echo "[nav_cleanup] killed $killed stale host process(es); $left nav/TF process(es) remain (want 0)"
[ "$left" -eq 0 ]
