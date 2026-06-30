"""Gazebo (Harmonic) physics demo: the K1 navigates to a QR panel with the REAL
Nav2 stack -- NavFn planner + Regulated Pure Pursuit controller + rolling
costmaps + recovery behaviors. This is the full navigation brain (it plans a
path *around* the obstacle), not the direct-pursuit fallback (gazebo_drive).

Localization is the drift-free Gazebo odometry rather than SLAM:
  * gz OdometryPublisher -> /odom            (ground-truth, no drift)
  * odom_tf  re-broadcasts a CLEAN odom->base_link from /odom
        (the bridged gz /tf has out-of-order stamps that break every tf2 lookup)
  * static map->odom identity gives Nav2 its global 'map' frame for free
        (valid precisely because gz odom doesn't drift: map and odom coincide)
Dropping SLAM also removes the heaviest node, which is what made Nav2's
lifecycle bring-up flaky (bond timeouts under CPU load) in earlier attempts.

  gz sim (qr_room + k1_glider) -> /scan /odom /cmd_vel
  ros_gz_bridge -> ROS 2 ;  Nav2 -> /cmd_vel
  gz_qr_truth -> /qr_detection ;  qr_goal -> NavigateToPose

Run:  ./run_gazebo.sh                          (Gazebo GUI + RViz)
      ./run_gazebo.sh gui:=false rviz:=false   (headless)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("k1_qr_nav")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    world = os.path.join(pkg, "worlds", "qr_room.sdf")
    models = os.path.join(pkg, "models")
    bridge_cfg = os.path.join(pkg, "config", "gz_bridge.yaml")
    nav2_params = os.path.join(pkg, "config", "nav2_params_gz.yaml")
    rviz_cfg = os.path.join(nav2_bringup, "rviz", "nav2_default_view.rviz")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    sim = {"use_sim_time": True}

    gz_server = ExecuteProcess(cmd=["gz", "sim", "-s", "-r", world], output="screen")
    gz_gui = ExecuteProcess(cmd=["gz", "sim", "-g"], output="log", condition=IfCondition(gui))

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  parameters=[{"config_file": bridge_cfg}, sim])

    # --- clean TF tree (replaces the broken bridged gz /tf) ---
    odom_tf = Node(package="k1_qr_nav", executable="odom_tf", name="odom_tf",
                   output="screen", parameters=[sim])
    # gz odom is drift-free, so map == odom: a static identity gives Nav2 a 'map' frame
    map_odom = Node(package="tf2_ros", executable="static_transform_publisher", parameters=[sim],
                    arguments=["0", "0", "0", "0", "0", "0", "map", "odom"])
    # the bridge rewrites gz '::' to '/', so the scan frame is 'k1_glider/base_link/lidar'
    scan_tf = Node(package="tf2_ros", executable="static_transform_publisher", parameters=[sim],
                   arguments=["0", "0", "0.35", "0", "0", "0", "base_link",
                              "k1_glider/base_link/lidar"])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, "launch", "navigation_launch.py")),
        launch_arguments={"use_sim_time": "true", "params_file": nav2_params,
                          "autostart": "true"}.items())
    qr_truth = Node(package="k1_qr_nav", executable="gz_qr_truth", name="gz_qr_truth",
                    output="screen", parameters=[sim, {"qr_x": 6.85, "qr_y": 1.4, "range_m": 12.0}])
    qr_goal = Node(package="k1_qr_nav", executable="qr_goal", name="qr_goal",
                   output="screen", parameters=[sim, {"standoff": 0.7}])
    rviz_node = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
                     output="log", parameters=[sim], condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo GUI"),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models),
        # TF + bridge come up immediately so Nav2 finds a populated tree at bring-up
        gz_server, gz_gui, bridge, odom_tf, map_odom, scan_tf,
        TimerAction(period=10.0, actions=[nav2]),
        TimerAction(period=18.0, actions=[qr_truth, qr_goal, rviz_node]),
    ])
