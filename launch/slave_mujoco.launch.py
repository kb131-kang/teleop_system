"""Launch file for the slave system (MuJoCo simulation).

Launches the MuJoCo ROS2 Bridge which:
  - Runs MuJoCo physics simulation
  - Publishes joint states and sensor data
  - Subscribes to joint commands from the master side
  - Optionally publishes RGB-D camera images
  - Optionally starts a TCP streaming server for remote viewers

Usage:
    ros2 launch teleop_system slave_mujoco.launch.py
    ros2 launch teleop_system slave_mujoco.launch.py launch_viewer:=true
    ros2 launch teleop_system slave_mujoco.launch.py publish_camera:=true camera_fps:=30
    ros2 launch teleop_system slave_mujoco.launch.py publish_camera:=true launch_streaming:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Launch arguments ──
        DeclareLaunchArgument(
            "launch_viewer", default_value="true",
            description="Launch MuJoCo passive viewer",
        ),
        DeclareLaunchArgument(
            "publish_camera", default_value="false",
            description="Publish RGB-D from MuJoCo head camera as ROS2 topics",
        ),
        DeclareLaunchArgument(
            "camera_fps", default_value="15.0",
            description="Camera publishing rate (Hz)",
        ),
        DeclareLaunchArgument(
            "camera_name", default_value="head_camera",
            description="MuJoCo camera name in the XML model",
        ),
        DeclareLaunchArgument(
            "camera_width", default_value="640",
            description="Camera image width",
        ),
        DeclareLaunchArgument(
            "camera_height", default_value="480",
            description="Camera image height",
        ),
        DeclareLaunchArgument(
            "mjcf_path", default_value="models/rby1/model_teleop.xml",
            description="Path to MuJoCo XML model",
        ),
        DeclareLaunchArgument(
            "physics_rate_hz", default_value="500.0",
            description="Physics simulation rate (Hz)",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz", default_value="100.0",
            description="Joint state publish rate (Hz)",
        ),
        DeclareLaunchArgument(
            "launch_streaming", default_value="false",
            description="Launch TCP streaming server (ros2-server) for remote viewers",
        ),
        DeclareLaunchArgument(
            "streaming_port", default_value="9876",
            description="TCP streaming server port",
        ),

        # ── MuJoCo ROS2 Bridge ──
        Node(
            package="teleop_system",
            executable="mujoco_bridge",
            name="mujoco_bridge",
            output="screen",
            parameters=[{
                "mjcf_path": LaunchConfiguration("mjcf_path"),
                "physics_rate_hz": LaunchConfiguration("physics_rate_hz"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "viewer_rate_hz": 60.0,
                "launch_viewer": LaunchConfiguration("launch_viewer"),
                "publish_camera": LaunchConfiguration("publish_camera"),
                "camera_fps": LaunchConfiguration("camera_fps"),
                "camera_name": LaunchConfiguration("camera_name"),
                "camera_width": LaunchConfiguration("camera_width"),
                "camera_height": LaunchConfiguration("camera_height"),
            }],
        ),

        # ── Optional: TCP Streaming Server (ROS2 → TCP bridge) ──
        ExecuteProcess(
            cmd=[
                FindExecutable(name="python3"),
                "-u", "scripts/demo_rgbd_streaming.py",
                "--mode", "ros2-server",
                "--host", "0.0.0.0",
                "--port", LaunchConfiguration("streaming_port"),
            ],
            name="rgbd_streaming_server",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_streaming")),
        ),
    ])
