"""CAMERA-ONLY (RGBD SLAM) nav in Gazebo -- the K1 has NO LiDAR, just the stereo
head camera. RTAB-Map maps from RGB+depth (replacing the LiDAR's mapping role),
Nav2 drives to a scan pose, the arm scans the QR.

  gz sim (camera_nav.sdf): diff-drive base + rgbd_camera -> /head_camera/* /odom /cmd_vel
  ros_gz_bridge  -> ROS
  odom_tf        -> clean odom->base_link from /odom        (K1 leg-odom analog)
  rtabmap        -> /map + map->odom   (RGBD visual SLAM, loop closure)
  depthimage_to_laserscan -> /scan (forward ~90 deg)        (Nav2 local obstacles)
  Nav2 (global=/map, local=/scan) -> /cmd_vel
  gz_qr_truth -> /qr_detection ;  qr_goal -> NavigateToPose(scan pose)
  arm_scan    -> on arrival, 4-DOF IK reach -> SCANNED

Run:  ./run_camera_nav.sh                          (gz GUI + RViz)
      ./run_camera_nav.sh gui:=false rviz:=false   (headless)
      ./run_camera_nav.sh visual_odom:=true        (PURE camera odometry, no /odom)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

K1_URDF = "/home/boosterk1/booster_assets/robots/K1/K1_22dof-ZED.urdf"
MESH_DIR = "/home/boosterk1/booster_assets/robots/K1/meshes/"
QR_X, QR_Y, QR_H = 5.3, 1.4, 0.789
SCAN_FWD, SCAN_LEFT, FACE_YAW = 0.30, 0.24, 0.0
QR_BOX_CX, QR_BOX_CY, QR_BOX_SZ, QR_BOX_H = 5.5, 1.4, 0.4, 0.85


def generate_launch_description():
    pkg = get_package_share_directory("k1_qr_nav")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    rtabmap_launch = get_package_share_directory("rtabmap_launch")
    world = os.path.join(pkg, "worlds", "camera_nav.sdf")
    models = os.path.join(pkg, "models")
    bridge_cfg = os.path.join(pkg, "config", "gz_bridge_rgbd.yaml")
    nav2_params = os.path.join(pkg, "config", "nav2_params_camera.yaml")
    rviz_cfg = os.path.join(nav2_bringup, "rviz", "nav2_default_view.rviz")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    visual_odom = LaunchConfiguration("visual_odom")
    sim = {"use_sim_time": True}

    robot_desc = ""
    if os.path.exists(K1_URDF):
        with open(K1_URDF) as fh:
            robot_desc = fh.read().replace('filename="meshes/', f'filename="file://{MESH_DIR}')

    gz_server = ExecuteProcess(cmd=["gz", "sim", "-s", "-r", "--headless-rendering", world],
                               output="screen")
    gz_gui = ExecuteProcess(cmd=["gz", "sim", "-g"], output="log", condition=IfCondition(gui))

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  parameters=[{"config_file": bridge_cfg}, sim])

    # --- TF tree:  map->odom (rtabmap) -> odom->base (odom_tf) -> base->camera (static) ; base->Trunk
    odom_tf = Node(package="k1_qr_nav", executable="odom_tf", name="odom_tf", output="screen",
                   parameters=[sim], condition=UnlessCondition(visual_odom))
    cam_tf = Node(package="tf2_ros", executable="static_transform_publisher", parameters=[sim],
                  arguments=["0.08", "0", "0.88", "0", "0.175", "0", "base_link", "head_camera"])
    cam_opt_tf = Node(package="tf2_ros", executable="static_transform_publisher", parameters=[sim],
                      arguments=["0", "0", "0", "-1.5708", "0", "-1.5708", "head_camera",
                                 "head_camera_optical"])
    trunk_tf = Node(package="tf2_ros", executable="static_transform_publisher", parameters=[sim],
                    arguments=["0", "0", "0.62", "0", "0", "0", "base_link", "Trunk"])

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch, "launch", "rtabmap.launch.py")),
        launch_arguments={
            "rgb_topic": "/head_camera/image",
            "depth_topic": "/head_camera/depth_image",
            "camera_info_topic": "/head_camera/camera_info",
            "frame_id": "base_link",
            "odom_topic": "/odom",
            "visual_odometry": visual_odom,
            "approx_sync": "true",
            "qos": "2",
            "use_sim_time": "true",
            "rtabmap_viz": "false",
            "rviz": "false",
            "rtabmap_args": "-d --Reg/Force3DoF true --Grid/RangeMax 5.0 --Grid/RayTracing true "
                            "--Grid/MaxObstacleHeight 1.2 --RGBD/OptimizeMaxError 3.0",
        }.items())

    d2s = Node(package="depthimage_to_laserscan", executable="depthimage_to_laserscan_node",
               name="depthimage_to_laserscan", output="screen",
               parameters=[sim, {"scan_height": 24, "scan_time": 0.066, "range_min": 0.35,
                                 "range_max": 8.0, "output_frame": "head_camera_optical"}],
               remappings=[("depth", "/head_camera/depth_image"),
                           ("depth_camera_info", "/head_camera/camera_info"),
                           ("scan", "/scan")])

    # Nav2 as its OWN subprocess (not a nested Include): navigation_launch's RewrittenYaml
    # param-rewrite races with the rest of this 12-node launch in the shared launch-service
    # event loop under RTAB-Map load and silently drops params (controller falls back to DWB
    # -> "No critics" -> abort). A dedicated `ros2 launch` subprocess gets a clean event loop.
    nav2 = ExecuteProcess(
        cmd=["ros2", "launch", "nav2_bringup", "navigation_launch.py",
             "use_sim_time:=true", "params_file:=" + nav2_params, "autostart:=true"],
        output="screen")

    qr_truth = Node(package="k1_qr_nav", executable="gz_qr_truth", name="gz_qr_truth", output="screen",
                    parameters=[sim, {"qr_x": QR_X, "qr_y": QR_Y, "fov_deg": 90.0, "range_m": 12.0}])
    qr_goal = Node(package="k1_qr_nav", executable="qr_goal", name="qr_goal", output="screen",
                   parameters=[sim, {"scan_fwd": SCAN_FWD, "scan_left": SCAN_LEFT, "face_yaw": FACE_YAW}])
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               name="robot_state_publisher", output="screen",
               parameters=[sim, {"robot_description": robot_desc}])
    arm = Node(package="k1_qr_nav", executable="arm_scan", name="arm_scan", output="screen",
               parameters=[sim, {"qr_x": QR_X, "qr_y": QR_Y, "qr_height": QR_H, "qr_box_cx": QR_BOX_CX,
                                 "qr_box_cy": QR_BOX_CY, "qr_box_size": QR_BOX_SZ, "qr_box_h": QR_BOX_H,
                                 "force_unsafe": ParameterValue(LaunchConfiguration("force_unsafe"),
                                                                value_type=bool)}])
    rviz_node = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], output="log",
                     parameters=[sim], condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo GUI"),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz"),
        DeclareLaunchArgument("visual_odom", default_value="false",
                              description="true = pure camera visual odometry (no wheel/leg odom)"),
        DeclareLaunchArgument("force_unsafe", default_value="false",
                              description="demo: inject a body-folding arm target -> safety gate refuses"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models),
        gz_server, gz_gui, bridge, cam_tf, cam_opt_tf, trunk_tf,
        TimerAction(period=2.0, actions=[odom_tf]),
        TimerAction(period=4.0, actions=[rtabmap, d2s]),
        TimerAction(period=12.0, actions=[nav2]),
        TimerAction(period=20.0, actions=[rsp, arm, qr_truth, qr_goal, rviz_node]),
    ])
