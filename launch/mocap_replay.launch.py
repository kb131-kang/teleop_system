"""ROS2 launch file for BVH motion capture replay.

Launches BVH replay publisher (replacing dummy publishers),
teleop processing nodes, and optionally MuJoCo bridge for
visual verification.

Usage:
    ros2 launch teleop_system mocap_replay.launch.py bvh_file:=/path/to/file.bvh
    ros2 launch teleop_system mocap_replay.launch.py bvh_file:=/path/to/file.bvh launch_bridge:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    bvh_file_arg = DeclareLaunchArgument(
        "bvh_file",
        description="Path to BVH motion capture file",
    )
    rate_hz_arg = DeclareLaunchArgument(
        "rate_hz", default_value="120.0",
        description="Replay publishing rate in Hz",
    )
    playback_speed_arg = DeclareLaunchArgument(
        "playback_speed", default_value="1.0",
        description="Playback speed multiplier",
    )
    loop_arg = DeclareLaunchArgument(
        "loop", default_value="true",
        description="Loop playback",
    )
    scale_arg = DeclareLaunchArgument(
        "scale", default_value="0.056",
        description="BVH to meters scale factor",
    )
    normalize_mode_arg = DeclareLaunchArgument(
        "normalize_mode", default_value="relative",
        description="Position normalization: relative or absolute",
    )
    launch_bridge_arg = DeclareLaunchArgument(
        "launch_bridge", default_value="false",
        description="Launch MuJoCo bridge for visual verification",
    )

    # BVH Replay Publisher (replaces dummy_tracker_pub + dummy_glove_pub + dummy_hmd_pub)
    bvh_replay = Node(
        package="teleop_system",
        executable="bvh_replay_pub",
        name="bvh_replay_pub",
        parameters=[{
            "bvh_file": LaunchConfiguration("bvh_file"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            "playback_speed": LaunchConfiguration("playback_speed"),
            "loop": LaunchConfiguration("loop"),
            "scale": LaunchConfiguration("scale"),
            "normalize_mode": LaunchConfiguration("normalize_mode"),
        }],
        output="screen",
    )

    # Arm teleop node
    arm_teleop = Node(
        package="teleop_system",
        executable="arm_teleop_node",
        name="arm_teleop_node",
        output="screen",
    )

    # Locomotion node
    locomotion = Node(
        package="teleop_system",
        executable="locomotion_node",
        name="locomotion_node",
        output="screen",
    )

    # Hand teleop node
    hand_teleop = Node(
        package="teleop_system",
        executable="hand_teleop_node",
        name="hand_teleop_node",
        output="screen",
    )

    # Camera teleop node
    camera_teleop = Node(
        package="teleop_system",
        executable="camera_teleop_node",
        name="camera_teleop_node",
        output="screen",
    )

    # Optional MuJoCo bridge
    mujoco_bridge = Node(
        package="teleop_system",
        executable="mujoco_bridge",
        name="mujoco_bridge",
        parameters=[{
            "launch_viewer": True,
            "publish_camera": False,
        }],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_bridge")),
    )

    return LaunchDescription([
        bvh_file_arg,
        rate_hz_arg,
        playback_speed_arg,
        loop_arg,
        scale_arg,
        normalize_mode_arg,
        launch_bridge_arg,
        bvh_replay,
        arm_teleop,
        locomotion,
        hand_teleop,
        camera_teleop,
        mujoco_bridge,
    ])
