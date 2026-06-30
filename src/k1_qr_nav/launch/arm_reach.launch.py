"""Watch the K1 left arm reach to a barcode in RViz (kinematic, full URDF).

  arm_reach        -> /joint_states (animated IK reach) + /arm_markers (barcode + aim)
  robot_state_publisher -> TF for every link (RViz RobotModel renders the meshes)

The URDF's relative mesh paths are rewritten to absolute file:// so the meshes
actually show. A static world->Trunk lifts the robot onto the ground plane.

Run:  ./run_arm_reach.sh           (RViz)
      ./run_arm_reach.sh rviz:=false   (headless, just logs the IK reaches)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

K1_URDF = "/home/boosterk1/booster_assets/robots/K1/K1_22dof-ZED.urdf"
MESH_DIR = "/home/boosterk1/booster_assets/robots/K1/meshes/"


def generate_launch_description():
    pkg = get_package_share_directory("k1_qr_nav")
    rviz_cfg = os.path.join(pkg, "rviz", "arm_reach.rviz")

    robot_desc = ""
    if os.path.exists(K1_URDF):
        with open(K1_URDF) as fh:
            robot_desc = fh.read()
        # relative 'meshes/X.STL' -> absolute file:// so RViz can find them
        robot_desc = robot_desc.replace('filename="meshes/', f'filename="file://{MESH_DIR}')

    rviz = LaunchConfiguration("rviz")

    static_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                     arguments=["0", "0", "0.62", "0", "0", "0", "world", "Trunk"])
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               name="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_desc}])
    reach = Node(package="k1_qr_nav", executable="arm_reach", name="arm_reach", output="screen")
    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2",
                     arguments=["-d", rviz_cfg], output="log",
                     condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="launch RViz"),
        static_tf, rsp, reach, rviz_node,
    ])
