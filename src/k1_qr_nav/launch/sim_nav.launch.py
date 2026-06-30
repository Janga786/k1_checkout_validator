"""Bring up the full SLAM + Nav2 navigate-to-QR demo in the lightweight 2-D sim.

  mini_sim  -> /scan /odom /qr_detection + TF odom->base_link
  slam_toolbox -> /map + TF map->odom
  Nav2 (nav2_bringup/navigation_launch) -> /cmd_vel
  qr_goal -> turns the QR detection into a Nav2 goal
  (+ optional K1 URDF model in RViz: model:=false to skip)

Run:  ros2 launch k1_qr_nav sim_nav.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

K1_URDF = "/home/boosterk1/booster_assets/robots/K1/K1_22dof-ZED.urdf"


def generate_launch_description():
    pkg = get_package_share_directory("k1_qr_nav")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")
    slam_params = os.path.join(pkg, "config", "slam_toolbox.yaml")
    rviz_cfg = os.path.join(nav2_bringup, "rviz", "nav2_default_view.rviz")

    robot_desc = ""
    if os.path.exists(K1_URDF):
        with open(K1_URDF) as fh:
            robot_desc = fh.read()

    use_model = LaunchConfiguration("model")

    sim = Node(package="k1_qr_nav", executable="mini_sim", name="mini_sim", output="screen")
    slam = Node(package="slam_toolbox", executable="async_slam_toolbox_node",
                name="slam_toolbox", output="screen", parameters=[slam_params])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, "launch", "navigation_launch.py")),
        launch_arguments={"use_sim_time": "false", "params_file": nav2_params,
                          "autostart": "true"}.items())
    qr_goal = Node(package="k1_qr_nav", executable="qr_goal", name="qr_goal", output="screen")
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2",
                arguments=["-d", rviz_cfg], output="log",
                condition=IfCondition(LaunchConfiguration("rviz")))

    # optional K1 model in RViz (cosmetic; navigation works without it)
    static_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                     arguments=["0", "0", "0", "0", "0", "0", "base_link", "Trunk"],
                     condition=IfCondition(use_model))
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               name="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_desc}],
               condition=IfCondition(use_model))
    jsp = Node(package="joint_state_publisher", executable="joint_state_publisher",
               name="joint_state_publisher", output="screen",
               condition=IfCondition(use_model))

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="true",
                              description="show the K1 URDF model in RViz (false to skip)"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="launch RViz (false for headless)"),
        sim, slam, static_tf, rsp, jsp,
        TimerAction(period=3.0, actions=[nav2]),            # slam + tf settle first
        TimerAction(period=6.0, actions=[qr_goal, rviz]),   # then goal + viz
    ])
