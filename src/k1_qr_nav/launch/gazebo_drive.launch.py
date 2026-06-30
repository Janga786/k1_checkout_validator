"""Reliable Gazebo demo: the K1 drives to the QR panel with a direct pursuit
controller (no Nav2/SLAM lifecycle). Use this to *see* the K1 perform in Gazebo.

  gz sim (qr_room + k1_glider) -> /odom /tf /scan /cmd_vel
  ros_gz_bridge -> ROS 2
  gz_qr_truth -> /qr_detection ;  drive_to_qr -> /cmd_vel (turn + approach + stop)

Run:  ./run_gazebo_drive.sh           (Gazebo GUI)
      ./run_gazebo_drive.sh gui:=false   (headless)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("k1_qr_nav")
    world = os.path.join(pkg, "worlds", "qr_room.sdf")
    models = os.path.join(pkg, "models")
    bridge_cfg = os.path.join(pkg, "config", "gz_bridge.yaml")
    gui = LaunchConfiguration("gui")
    sim = {"use_sim_time": True}

    gz_server = ExecuteProcess(cmd=["gz", "sim", "-s", "-r", world], output="screen")
    gz_gui = ExecuteProcess(cmd=["gz", "sim", "-g"], output="log", condition=IfCondition(gui))
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  parameters=[{"config_file": bridge_cfg}, sim])
    qr_truth = Node(package="k1_qr_nav", executable="gz_qr_truth", name="gz_qr_truth",
                    output="screen", parameters=[sim, {"qr_x": 6.85, "qr_y": 1.4, "range_m": 12.0}])
    drive = Node(package="k1_qr_nav", executable="drive_to_qr", name="drive_to_qr",
                 output="screen", parameters=[sim, {"standoff": 0.8}])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo GUI"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models),
        gz_server, gz_gui, bridge,
        TimerAction(period=6.0, actions=[qr_truth, drive]),
    ])
