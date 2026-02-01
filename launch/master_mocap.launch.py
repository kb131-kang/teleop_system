"""Launch file for the master system with BVH motion capture replay.

Replaces dummy sinusoidal publishers with real BVH motion data.
Launches BVH replay publisher and all teleop processing nodes:
  - BVH Replay Publisher (replaces Dummy Tracker/Glove/HMD Publishers)
  - Arm Teleop Node (tracker -> IK -> joint commands)
  - Locomotion Node (foot tracker -> gait -> base velocity)
  - Hand Teleop Node (glove -> retargeting -> gripper commands)
  - Camera Teleop Node (HMD -> head pan/tilt commands)
  - (Optional) ROS2 RGB-D Viewer for first-person camera view

Usage:
    ros2 launch teleop_system master_mocap.launch.py bvh_file:=/path/to/file.bvh
    ros2 launch teleop_system master_mocap.launch.py bvh_file:=/path/to/file.bvh playback_speed:=0.5
    ros2 launch teleop_system master_mocap.launch.py bvh_file:=/path/to/file.bvh launch_viewer:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── BVH parameters ──
        DeclareLaunchArgument(
            "bvh_file",
            description="Path to BVH motion capture file (required)",
        ),
        DeclareLaunchArgument(
            "rate_hz", default_value="120.0",
            description="Replay publishing rate in Hz",
        ),
        DeclareLaunchArgument(
            "playback_speed", default_value="1.0",
            description="Playback speed multiplier",
        ),
        DeclareLaunchArgument(
            "loop", default_value="true",
            description="Loop playback when reaching end",
        ),
        DeclareLaunchArgument(
            "scale", default_value="0.056",
            description="BVH to meters scale factor",
        ),
        DeclareLaunchArgument(
            "normalize_mode", default_value="relative",
            description="Position normalization: relative or absolute",
        ),
        DeclareLaunchArgument(
            "launch_viewer", default_value="false",
            description="Launch RGB-D point cloud viewer (first-person camera)",
        ),
        DeclareLaunchArgument(
            "viewer_mode", default_value="ros2-viewer",
            description="Viewer mode: ros2-viewer (same machine, ROS2 topics) "
                        "or client (cross-machine, TCP stream from slave)",
        ),
        DeclareLaunchArgument(
            "viewer_host", default_value="localhost",
            description="TCP streaming host for viewer_mode=client (slave IP address)",
        ),
        DeclareLaunchArgument(
            "viewer_port", default_value="9876",
            description="TCP streaming port for viewer_mode=client",
        ),
        DeclareLaunchArgument(
            "launch_gui", default_value="true",
            description="Launch GUI control panel",
        ),
        DeclareLaunchArgument(
            "font_scale", default_value="0.0",
            description="GUI font scale (0.0 = auto-detect from screen resolution)",
        ),
        DeclareLaunchArgument(
            "auto_start", default_value="false",
            description="Start BVH playback immediately (false = wait for start command)",
        ),

        # ── BVH Replay Publisher ──
        # (replaces dummy_tracker_pub + dummy_glove_pub + dummy_hmd_pub)
        Node(
            package="teleop_system",
            executable="bvh_replay_pub",
            name="bvh_replay_pub",
            output="screen",
            parameters=[{
                "bvh_file": LaunchConfiguration("bvh_file"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                "playback_speed": LaunchConfiguration("playback_speed"),
                "loop": LaunchConfiguration("loop"),
                "scale": LaunchConfiguration("scale"),
                "normalize_mode": LaunchConfiguration("normalize_mode"),
                "auto_start": LaunchConfiguration("auto_start"),
            }],
        ),

        # ── Arm Teleop Node ──
        Node(
            package="teleop_system",
            executable="arm_teleop_node",
            name="arm_teleop",
            output="screen",
        ),

        # ── Locomotion Node ──
        Node(
            package="teleop_system",
            executable="locomotion_node",
            name="locomotion",
            output="screen",
            parameters=[{
                "rate_hz": 50.0,
                "deadzone": 0.02,
                "linear_scale": 1.0,
                "angular_scale": 1.0,
                "max_linear_velocity": 0.5,
                "max_angular_velocity": 1.0,
            }],
        ),

        # ── Hand Teleop Node ──
        Node(
            package="teleop_system",
            executable="hand_teleop_node",
            name="hand_teleop",
            output="screen",
            parameters=[{
                "rate_hz": 100.0,
                "max_joint_velocity": 5.0,
                "smoothing_alpha": 0.3,
            }],
        ),

        # ── Camera Teleop Node ──
        Node(
            package="teleop_system",
            executable="camera_teleop_node",
            name="camera_teleop",
            output="screen",
            parameters=[{
                "rate_hz": 30.0,
                "smoothing_alpha": 0.3,
                "max_angular_velocity": 2.0,
            }],
        ),

        # ── Calibration Node ──
        Node(
            package="teleop_system",
            executable="calibration_node",
            name="calibration_node",
            output="screen",
        ),

        # ── Optional: GUI Control Panel ──
        Node(
            package="teleop_system",
            executable="gui_control_panel",
            name="gui_control_panel",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_gui")),
            parameters=[{
                "viewer_host": LaunchConfiguration("viewer_host"),
                "viewer_port": LaunchConfiguration("viewer_port"),
                "font_scale": LaunchConfiguration("font_scale"),
            }],
        ),

        # ── Optional: RGB-D Point Cloud Viewer ──
        # viewer_mode: "ros2-viewer" for same-machine (ROS2 topics),
        #              "client" for cross-machine (TCP stream from slave)
        ExecuteProcess(
            cmd=[
                FindExecutable(name="python3"),
                "-u", "scripts/demo_rgbd_streaming.py",
                "--mode", LaunchConfiguration("viewer_mode"),
                "--host", LaunchConfiguration("viewer_host"),
                "--port", LaunchConfiguration("viewer_port"),
            ],
            name="rgbd_viewer",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_viewer")),
        ),
    ])
