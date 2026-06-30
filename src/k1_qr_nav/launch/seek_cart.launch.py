"""Booster K1: DETECT the utility cart with the head cam, then WALK to it, stopping 1 m away.

Same SLAM+Nav2 stack as seek_barcode; the target is the cart (YOLO-World open-vocab) and the
standoff defaults to 1.0 m. Driving is the SDK velocity command: Nav2 -> /cmd_vel -> booster_bridge
-> Move(vx,vy,vyaw) (the built-in walker). Head turns keep the cart in view (RotateHead).

  booster_bridge   /cmd_vel -> Move(walk) ; /head_cmd -> RotateHead ; leg odom -> /odom
  cart_locate      cart detect-server boxes -> /cart_detection (3D) + /cart_bearing
  head_search      sweep to find the cart, then track it          -> /head_cmd
  qr_goal          /cart_detection -> standoff goal (1 m) facing the cart -> NavigateToPose
  Nav2             RegulatedPurePursuit -> /cmd_vel
  (optional) depth->scan obstacle layer, slam_toolbox mapping

The detector (YOLO-World/torch) runs SEPARATELY in the navila env as cart_detect_server.py
(see run_seek_cart.sh) and is reached over HTTP -- it is NOT a ROS node.

  ros2 launch k1_qr_nav seek_cart.launch.py dry_run:=true fake_target:=true rviz:=true  # no robot/detector
  ros2 launch k1_qr_nav seek_cart.launch.py net:=eno1                                    # real robot, odom-frame
  ros2 launch k1_qr_nav seek_cart.launch.py net:=eno1 slam:=true use_scan:=true rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
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
    real_height = LaunchConfiguration("real_height")
    hfov_deg = LaunchConfiguration("hfov_deg")
    standoff = LaunchConfiguration("standoff")
    depth_topic = LaunchConfiguration("depth_topic")
    info_topic = LaunchConfiguration("info_topic")
    fake_target = LaunchConfiguration("fake_target")

    # --- Booster bridge: walk + head + odom (SDK Move/RotateHead). Real robot is GATED (IDLE
    #     until /start_walking); dry-run integrates /cmd_vel into /odom so the graph runs no-robot.
    bridge_dry = Node(package="k1_qr_nav", executable="booster_bridge", name="booster_bridge",
                      output="screen", arguments=["--dry-run", "--net", net],
                      parameters=[{"auto_walk": True}], condition=IfCondition(dry_run))
    bridge_real = Node(package="k1_qr_nav", executable="booster_bridge", name="booster_bridge",
                       output="screen", arguments=["--net", net],
                       parameters=[{"auto_walk": LaunchConfiguration("auto_walk"),
                                    "max_vx": LaunchConfiguration("max_vx")}],
                       condition=UnlessCondition(dry_run))

    odom_tf = Node(package="k1_qr_nav", executable="odom_tf", name="odom_tf", output="screen")

    cam_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                  arguments=["0.08", "0", "0.88", "0", "0.175", "0", "base_link", "head_camera"])
    cam_opt_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                      arguments=["0", "0", "0", "-1.5708", "0", "-1.5708",
                                 "head_camera", "head_camera_optical"])
    trunk_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                    arguments=["0", "0", "0.62", "0", "0", "0", "base_link", "Trunk"])
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

    # real perception: cart_locate -> /cart_detection + /cart_bearing
    locate = Node(package="k1_qr_nav", executable="cart_locate", name="cart_locate",
                  output="screen", condition=UnlessCondition(fake_target),
                  parameters=[{"det_url": det_url, "frame_url": frame_url,
                               "hfov_deg": hfov_deg, "real_height": real_height,
                               "assumed_range": assumed_range,
                               "camera_frame": "head_camera_optical"}])
    # self-contained demo: a virtual cart at a known spot, "seen" from the robot's /odom pose
    fake = Node(package="k1_qr_nav", executable="gz_qr_truth", name="virtual_cart",
                output="screen", condition=IfCondition(fake_target),
                parameters=[{"qr_x": LaunchConfiguration("target_x"),
                             "qr_y": LaunchConfiguration("target_y"),
                             "fov_deg": 270.0, "range_m": 30.0}],
                remappings=[("qr_detection", "cart_detection")])
    # reused nodes, remapped onto the /cart_* topics
    head = Node(package="k1_qr_nav", executable="head_search", name="head_search", output="screen",
                remappings=[("barcode_bearing", "cart_bearing")])
    goal = Node(package="k1_qr_nav", executable="qr_goal", name="qr_goal", output="screen",
                parameters=[{"standoff": standoff}],
                remappings=[("qr_detection", "cart_detection")])
    rviz_node = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
                     output="log", condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="true = no robot/SDK; integrate /cmd_vel into /odom"),
        DeclareLaunchArgument("net", default_value="", description="DDS network interface for the SDK"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("use_scan", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("det_url", default_value="http://127.0.0.1:8090/detections.json"),
        DeclareLaunchArgument("frame_url", default_value="http://127.0.0.1:8090/frame.jpg"),
        DeclareLaunchArgument("assumed_range", default_value="1.5"),
        DeclareLaunchArgument("real_height", default_value="0.90",
                              description="cart height (m) for monocular range-from-size"),
        DeclareLaunchArgument("hfov_deg", default_value="96.8"),
        DeclareLaunchArgument("standoff", default_value="1.0",
                              description="stop this far in front of the cart, facing it"),
        DeclareLaunchArgument("depth_topic", default_value="/head_camera/depth_image"),
        DeclareLaunchArgument("info_topic", default_value="/head_camera/camera_info"),
        DeclareLaunchArgument("fake_target", default_value="false",
                              description="dry-run demo: walk to a VIRTUAL cart (no robot/detector)"),
        DeclareLaunchArgument("target_x", default_value="2.5"),
        DeclareLaunchArgument("target_y", default_value="0.6"),
        DeclareLaunchArgument("auto_walk", default_value="false",
                              description="REAL robot: false = gated (/start_walking); true = walk on launch"),
        DeclareLaunchArgument("max_vx", default_value="0.20",
                              description="REAL robot forward-speed cap (m/s)"),
        bridge_dry, bridge_real, cam_tf, cam_opt_tf, trunk_tf, map_odom_tf,
        TimerAction(period=2.0, actions=[odom_tf]),
        TimerAction(period=3.0, actions=[depth_scan, slam_node]),
        TimerAction(period=6.0, actions=[nav2]),
        TimerAction(period=12.0, actions=[locate, fake, head, goal, rviz_node]),
    ])
