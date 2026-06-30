"""Booster K1: DETECT a barcode with the head cam, then WALK to it.

Real-robot (or --dry-run) bring-up of the seek-barcode nav stack:

  booster_bridge   /cmd_vel -> Move(walk) ; /head_cmd -> RotateHead ; leg odom -> /odom
                   (--dry-run integrates /cmd_vel into /odom so the graph runs with no robot)
  odom_tf          clean odom->base_link from /odom
  static TFs       base_link->head_camera->head_camera_optical ; base_link->Trunk
  map->odom        static identity (odom-frame nav) | slam_toolbox provides it in --slam mode
  depth->scan      depthimage_to_laserscan -> /scan          (optional: needs a depth topic)
  Nav2             RegulatedPurePursuit -> /cmd_vel           (own subprocess, clean event loop)
  barcode_locate   detect-server boxes -> /qr_detection + /barcode_bearing
  head_search      sweep to find the barcode, then track it  -> /head_cmd
  qr_goal          /qr_detection -> standoff goal facing the barcode -> NavigateToPose

The detector itself (YOLO/torch) runs SEPARATELY in the receipt_validator .venv as
detect_server.py (see run_seek_barcode.sh) and is reached over HTTP -- it is NOT a ROS node.

  ros2 launch k1_qr_nav seek_barcode.launch.py dry_run:=true          # workstation, no robot
  ros2 launch k1_qr_nav seek_barcode.launch.py net:=eth0              # real robot, odom-frame
  ros2 launch k1_qr_nav seek_barcode.launch.py net:=eth0 slam:=true use_scan:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("k1_qr_nav")
    nav2_params = os.path.join(pkg, "config", "nav2_params_booster.yaml")
    slam_cfg = os.path.join(pkg, "config", "slam_toolbox.yaml")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    rviz_cfg = os.path.join(nav2_bringup, "rviz", "nav2_default_view.rviz")

    dry_run = LaunchConfiguration("dry_run")
    net = LaunchConfiguration("net")
    slam = LaunchConfiguration("slam")
    use_scan = LaunchConfiguration("use_scan")
    rviz = LaunchConfiguration("rviz")
    det_url = LaunchConfiguration("det_url")
    frame_url = LaunchConfiguration("frame_url")
    assumed_range = LaunchConfiguration("assumed_range")
    hfov_deg = LaunchConfiguration("hfov_deg")
    standoff = LaunchConfiguration("standoff")
    depth_topic = LaunchConfiguration("depth_topic")
    info_topic = LaunchConfiguration("info_topic")

    # --- the Booster bridge: walk + head + odom (dry-run or real, same node) ---
    # dry-run auto-walks (no robot, no risk -> the demo just runs); the REAL robot is GATED:
    # it stays IDLE until you send /start_walking (auto_walk:=true to override, not advised).
    bridge_dry = Node(package="k1_qr_nav", executable="booster_bridge", name="booster_bridge",
                      output="screen", arguments=["--dry-run", "--net", net],
                      parameters=[{"auto_walk": True}], condition=IfCondition(dry_run))
    bridge_real = Node(package="k1_qr_nav", executable="booster_bridge", name="booster_bridge",
                       output="screen", arguments=["--net", net],
                       parameters=[{"auto_walk": LaunchConfiguration("auto_walk"),
                                    "max_vx": LaunchConfiguration("max_vx")}],
                       condition=UnlessCondition(dry_run))

    odom_tf = Node(package="k1_qr_nav", executable="odom_tf", name="odom_tf", output="screen")

    # TF: base_link -> head_camera (ZED ~0.88 m up, slight down-pitch) -> optical ; base_link -> Trunk
    cam_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                  arguments=["0.08", "0", "0.88", "0", "0.175", "0", "base_link", "head_camera"])
    cam_opt_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                      arguments=["0", "0", "0", "-1.5708", "0", "-1.5708",
                                 "head_camera", "head_camera_optical"])
    trunk_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                    arguments=["0", "0", "0.62", "0", "0", "0", "base_link", "Trunk"])
    # map -> odom: static identity for odom-frame nav (slam_toolbox supplies this in --slam mode)
    map_odom_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                       arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                       condition=UnlessCondition(slam))

    depth_scan = Node(package="depthimage_to_laserscan",
                      executable="depthimage_to_laserscan_node", name="depthimage_to_laserscan",
                      output="screen", condition=IfCondition(use_scan),
                      parameters=[{"scan_height": 24, "range_min": 0.35, "range_max": 8.0,
                                   "output_frame": "head_camera_optical"}],
                      remappings=[("depth", depth_topic), ("depth_camera_info", info_topic),
                                  ("scan", "/scan")])

    slam_node = Node(package="slam_toolbox", executable="async_slam_toolbox_node",
                     name="slam_toolbox", output="screen", condition=IfCondition(slam),
                     parameters=[slam_cfg, {"use_sim_time": False}])

    nav2 = ExecuteProcess(
        cmd=["ros2", "launch", "nav2_bringup", "navigation_launch.py",
             "use_sim_time:=false", "params_file:=" + nav2_params, "autostart:=true"],
        output="screen")

    # real perception (barcode_locate) unless we're driving a virtual target for a dry-run demo
    fake_target = LaunchConfiguration("fake_target")
    locate = Node(package="k1_qr_nav", executable="barcode_locate", name="barcode_locate",
                  output="screen", condition=UnlessCondition(fake_target),
                  parameters=[{"det_url": det_url, "frame_url": frame_url,
                               "hfov_deg": hfov_deg, "assumed_range": assumed_range,
                               "camera_frame": "head_camera_optical"}])
    # self-contained demo: a virtual barcode at a known spot, "seen" from the robot's /odom pose
    fake = Node(package="k1_qr_nav", executable="gz_qr_truth", name="virtual_barcode",
                output="screen", condition=IfCondition(fake_target),
                parameters=[{"qr_x": LaunchConfiguration("target_x"),
                             "qr_y": LaunchConfiguration("target_y"),
                             "fov_deg": 270.0, "range_m": 30.0}])
    head = Node(package="k1_qr_nav", executable="head_search", name="head_search", output="screen")
    goal = Node(package="k1_qr_nav", executable="qr_goal", name="qr_goal", output="screen",
                parameters=[{"standoff": standoff}])
    rviz_node = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
                     output="log", condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="true = no robot/SDK; integrate /cmd_vel into /odom"),
        DeclareLaunchArgument("net", default_value="", description="DDS network interface for the SDK"),
        DeclareLaunchArgument("slam", default_value="false",
                              description="true = slam_toolbox (needs /scan); false = odom-frame nav"),
        DeclareLaunchArgument("use_scan", default_value="false",
                              description="true = depthimage_to_laserscan -> /scan (needs a depth topic)"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("det_url", default_value="http://127.0.0.1:8090/detections.json"),
        DeclareLaunchArgument("frame_url", default_value="http://127.0.0.1:8090/frame.jpg"),
        DeclareLaunchArgument("assumed_range", default_value="1.5",
                              description="m along the bearing until metric depth is wired"),
        DeclareLaunchArgument("hfov_deg", default_value="96.8"),
        DeclareLaunchArgument("standoff", default_value="0.6",
                              description="stop this far in front of the barcode, facing it"),
        DeclareLaunchArgument("depth_topic", default_value="/head_camera/depth_image"),
        DeclareLaunchArgument("info_topic", default_value="/head_camera/camera_info"),
        DeclareLaunchArgument("fake_target", default_value="false",
                              description="dry-run demo: walk to a VIRTUAL barcode (no robot/detector)"),
        DeclareLaunchArgument("target_x", default_value="2.5"),
        DeclareLaunchArgument("target_y", default_value="0.6"),
        DeclareLaunchArgument("auto_walk", default_value="false",
                              description="REAL robot: false = gated (send /start_walking); true = walk on launch"),
        DeclareLaunchArgument("max_vx", default_value="0.20",
                              description="REAL robot forward-speed cap (m/s); raise once verified"),
        bridge_dry, bridge_real, cam_tf, cam_opt_tf, trunk_tf, map_odom_tf,
        TimerAction(period=2.0, actions=[odom_tf]),
        TimerAction(period=3.0, actions=[depth_scan, slam_node]),
        TimerAction(period=6.0, actions=[nav2]),
        TimerAction(period=12.0, actions=[locate, fake, head, goal, rviz_node]),
    ])
