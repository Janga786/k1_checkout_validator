"""Full pipeline in the lightweight sim: SLAM + Nav2 navigate the K1 to the QR,
then its arm scans it.

  mini_sim       -> /scan /odom /qr_detection + TF odom->base_link
  slam_toolbox   -> /map + TF map->odom
  Nav2           -> /cmd_vel
  qr_goal        -> NavigateToPose to a *scan pose* (QR lands in the arm's zone)
                    + /nav_arrived when parked
  arm_scan       -> on arrival, 4-DOF IK reaches the QR -> "SCANNED" (+ markers)
  robot_state_publisher renders the full articulated URDF

Run:  ./run_sim_nav_scan.sh            (RViz)
      ./run_sim_nav_scan.sh rviz:=false   (headless)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

K1_URDF = "/home/boosterk1/booster_assets/robots/K1/K1_22dof-ZED.urdf"
MESH_DIR = "/home/boosterk1/booster_assets/robots/K1/meshes/"
QR_X, QR_Y, QR_H = 5.3, 1.4, 0.789                 # QR world pose (height = arm zone)
SCAN_FWD, SCAN_LEFT, FACE_YAW = 0.30, 0.24, 0.0    # QR in robot frame (real-hand reach) + faces box
QR_BOX_CX, QR_BOX_CY, QR_BOX_SZ, QR_BOX_H = 5.5, 1.4, 0.4, 0.85   # box the QR is on (-x face = QR)


def generate_launch_description():
    pkg = get_package_share_directory("k1_qr_nav")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    nav2_params = os.path.join(pkg, "config", "nav2_params_scan.yaml")
    slam_params = os.path.join(pkg, "config", "slam_toolbox.yaml")
    rviz_cfg = os.path.join(pkg, "rviz", "sim_nav_scan.rviz")

    robot_desc = ""
    if os.path.exists(K1_URDF):
        with open(K1_URDF) as fh:
            robot_desc = fh.read()
        robot_desc = robot_desc.replace('filename="meshes/', f'filename="file://{MESH_DIR}')

    rviz = LaunchConfiguration("rviz")

    sim = Node(package="k1_qr_nav", executable="mini_sim", name="mini_sim", output="screen",
               parameters=[{"qr_x": QR_X, "qr_y": QR_Y, "qr_box_cx": QR_BOX_CX,
                            "qr_box_cy": QR_BOX_CY, "qr_box_size": QR_BOX_SZ,
                            "layout": ParameterValue(LaunchConfiguration("layout"), value_type=int)}])
    slam = Node(package="slam_toolbox", executable="async_slam_toolbox_node",
                name="slam_toolbox", output="screen", parameters=[slam_params])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, "launch", "navigation_launch.py")),
        launch_arguments={"use_sim_time": "false", "params_file": nav2_params,
                          "autostart": "true"}.items())
    qr_goal = Node(package="k1_qr_nav", executable="qr_goal", name="qr_goal", output="screen",
                   parameters=[{"scan_fwd": SCAN_FWD, "scan_left": SCAN_LEFT, "face_yaw": FACE_YAW}])

    base_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                   arguments=["0", "0", "0.62", "0", "0", "0", "base_link", "Trunk"])
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               name="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_desc}])
    arm = Node(package="k1_qr_nav", executable="arm_scan", name="arm_scan", output="screen",
               parameters=[{"qr_x": QR_X, "qr_y": QR_Y, "qr_height": QR_H,
                            "qr_box_cx": QR_BOX_CX, "qr_box_cy": QR_BOX_CY,
                            "qr_box_size": QR_BOX_SZ, "qr_box_h": QR_BOX_H}])
    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_cfg],
                     output="log", condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="launch RViz"),
        DeclareLaunchArgument("layout", default_value="0", description="obstacle layout 0..3"),
        sim, slam, base_tf, rsp, arm,
        TimerAction(period=3.0, actions=[nav2]),
        TimerAction(period=6.0, actions=[qr_goal, rviz_node]),
    ])
