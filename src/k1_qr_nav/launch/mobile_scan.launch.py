"""Full pipeline in RViz: the K1 drives to a QR panel and its arm scans it.

  mobile_scan -> world->base_link TF (moving base) + /joint_states + /scan_markers
  robot_state_publisher -> TF for every link (RViz RobotModel renders the meshes)

URDF relative mesh paths are rewritten to absolute file://. base_link->Trunk
lifts the body onto its legs; the node moves world->base_link as it drives.

Run:  ./run_mobile_scan.sh             (RViz)
      ./run_mobile_scan.sh rviz:=false  (headless, logs the phases)
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
    rviz_cfg = os.path.join(pkg, "rviz", "mobile_scan.rviz")

    robot_desc = ""
    if os.path.exists(K1_URDF):
        with open(K1_URDF) as fh:
            robot_desc = fh.read()
        robot_desc = robot_desc.replace('filename="meshes/', f'filename="file://{MESH_DIR}')

    rviz = LaunchConfiguration("rviz")

    base_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                   arguments=["0", "0", "0.62", "0", "0", "0", "base_link", "Trunk"])
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               name="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_desc}])
    scan = Node(package="k1_qr_nav", executable="mobile_scan", name="mobile_scan", output="screen")
    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2",
                     arguments=["-d", rviz_cfg], output="log",
                     condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="launch RViz"),
        base_tf, rsp, scan, rviz_node,
    ])
