"""Booster K1: REACTIVE cart-seek -- detect the cart, walk to it, stop `standoff` m away.

A pure body-relative visual servo (cart_approach) replaces the whole Nav2/SLAM goal pipeline:
NO navigate_to_pose, NO costmaps, NO map/odom TF, NO head sweep. The head is held FORWARD so
the camera bearing == the body bearing, and the robot turns its body + walks straight in. This
sidesteps the audit's Nav2-stack blockers (lifecycle never activating, head-pan goal desync,
goal-behind-robot, map-TF timing). Driving is still the SDK Move via booster_bridge, which
GATES (no motion until /start_walking) and HARD-CLAMPS every command.

  booster_bridge   /cmd_vel -> Move(walk) ; /head_cmd -> RotateHead ; leg odom -> /odom
  cart_locate      cart detect-server boxes -> /cart_detection (3D) + /cart_bearing  (real)
  gz_qr_truth      virtual cart from /odom  -> /cart_detection                       (fake/dry-run)
  cart_approach    /cart_detection -> turn-to-centre + walk-to-standoff -> /cmd_vel + /head_cmd

  ros2 launch k1_qr_nav seek_cart_reactive.launch.py dry_run:=true fake_target:=true  # offline closed loop
  ros2 launch k1_qr_nav seek_cart_reactive.launch.py net:=eno1                        # real robot (gated IDLE)
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    dry_run = LaunchConfiguration("dry_run")
    net = LaunchConfiguration("net")
    fake_target = LaunchConfiguration("fake_target")
    standoff = LaunchConfiguration("standoff")

    bridge_dry = Node(package="k1_qr_nav", executable="booster_bridge", name="booster_bridge",
                      output="screen", arguments=["--dry-run", "--net", net],
                      parameters=[{"auto_walk": True}], condition=IfCondition(dry_run))
    bridge_real = Node(package="k1_qr_nav", executable="booster_bridge", name="booster_bridge",
                       output="screen", arguments=["--net", net],
                       parameters=[{"auto_walk": LaunchConfiguration("auto_walk"),
                                    "max_vx": LaunchConfiguration("max_vx")}],
                       condition=UnlessCondition(dry_run))

    # real perception: cart_locate -> /cart_detection (head_camera_optical) + /cart_bearing
    locate = Node(package="k1_qr_nav", executable="cart_locate", name="cart_locate",
                  output="screen", condition=UnlessCondition(fake_target),
                  parameters=[{"det_url": LaunchConfiguration("det_url"),
                               "frame_url": LaunchConfiguration("frame_url"),
                               "hfov_deg": LaunchConfiguration("hfov_deg"),
                               "real_height": LaunchConfiguration("real_height"),
                               "assumed_range": LaunchConfiguration("assumed_range"),
                               "min_score": LaunchConfiguration("locate_min_score"),
                               "camera_frame": "head_camera_optical"}])
    # offline closed loop: a virtual cart "seen" from the robot's integrated /odom pose
    fake = Node(package="k1_qr_nav", executable="gz_qr_truth", name="virtual_cart",
                output="screen", condition=IfCondition(fake_target),
                parameters=[{"qr_x": LaunchConfiguration("target_x"),
                             "qr_y": LaunchConfiguration("target_y"),
                             "fov_deg": 96.8, "range_m": 30.0}],
                remappings=[("qr_detection", "cart_detection")])

    tracker = LaunchConfiguration("tracker")
    common = {"standoff": standoff, "vx_cap": LaunchConfiguration("vx_cap"),
              "vyaw_cap": LaunchConfiguration("vyaw_cap"), "min_score": LaunchConfiguration("min_score"),
              "head_pitch": LaunchConfiguration("head_pitch"), "yaw_sign": LaunchConfiguration("yaw_sign")}
    # ODOM-frame tracker (DEFAULT): pin the cart in /odom from agreeing detections, drive on fast
    # odometry -- robust to this robot's ~0.4 fps camera; won't move until locked; supports orbit.
    track = Node(package="k1_qr_nav", executable="cart_track", name="cart_track", output="screen",
                 condition=IfCondition(PythonExpression(["'", tracker, "' == 'odom'"])),
                 parameters=[dict(common, orbit=LaunchConfiguration("orbit"))])
    # SERVO fallback: body-relative visual servo on the INSTANTANEOUS detection (fragile if camera is slow).
    approach = Node(package="k1_qr_nav", executable="cart_approach", name="cart_approach", output="screen",
                    condition=IfCondition(PythonExpression(["'", tracker, "' == 'servo'"])),
                    parameters=[dict(common, search_when_lost=LaunchConfiguration("search_when_lost"))])

    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("net", default_value="", description="DDS network interface for the SDK"),
        DeclareLaunchArgument("fake_target", default_value="false",
                              description="dry-run demo: walk to a VIRTUAL cart (no robot/detector)"),
        DeclareLaunchArgument("standoff", default_value="1.0",
                              description="stop this far in front of the cart, facing it (m)"),
        DeclareLaunchArgument("tracker", default_value="odom",
                              description="odom = cart_track (pin the cart in /odom, drive on odometry -- "
                                          "robust to the slow camera; supports orbit). servo = cart_approach "
                                          "(instantaneous visual servo; fragile if the camera lags)."),
        DeclareLaunchArgument("orbit", default_value="false",
                              description="cart_track only: after reaching the standoff, walk a circle around the cart"),
        DeclareLaunchArgument("vx_cap", default_value="0.18", description="approach forward-speed cap (m/s)"),
        DeclareLaunchArgument("vyaw_cap", default_value="0.30", description="approach turn-speed cap (rad/s)"),
        DeclareLaunchArgument("min_score", default_value="0.15",
                              description="cart_approach: drop detections below this confidence. "
                                          "yolov8x-world scores the live distant cart ~0.20-0.28 (bouncy) with "
                                          "next-best clutter ~0.05, so 0.15 keeps the cart locked with a 3x margin. "
                                          "Closer cart -> 0.5-0.9; raise this if a scene has cart-like clutter."),
        DeclareLaunchArgument("locate_min_score", default_value="0.12",
                              description="cart_locate: detector score floor for publishing"),
        DeclareLaunchArgument("head_pitch", default_value="0.20", description="rad down; head held forward"),
        DeclareLaunchArgument("yaw_sign", default_value="-1.0",
                              description="flip to 1.0 if the robot turns the WRONG way toward the cart"),
        DeclareLaunchArgument("search_when_lost", default_value="false",
                              description="true = slow body rotate to reacquire a lost cart; false = STOP"),
        DeclareLaunchArgument("det_url", default_value="http://127.0.0.1:8090/detections.json"),
        DeclareLaunchArgument("frame_url", default_value="http://127.0.0.1:8090/frame.jpg"),
        DeclareLaunchArgument("assumed_range", default_value="1.5"),
        DeclareLaunchArgument("real_height", default_value="0.90",
                              description="cart height (m) for monocular range-from-size"),
        DeclareLaunchArgument("hfov_deg", default_value="96.8"),
        DeclareLaunchArgument("target_x", default_value="2.5"),
        DeclareLaunchArgument("target_y", default_value="0.6"),
        DeclareLaunchArgument("auto_walk", default_value="false",
                              description="REAL robot: false = gated (/start_walking); true = walk on launch"),
        DeclareLaunchArgument("max_vx", default_value="0.20", description="REAL robot forward-speed cap (m/s)"),
        bridge_dry, bridge_real,
        # detector/perception + controller come up a beat after the bridge so /odom is flowing
        TimerAction(period=3.0, actions=[locate, fake, track, approach]),
    ])
