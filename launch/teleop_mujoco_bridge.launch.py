"""Launch file for MuJoCo simulation teleoperation pipeline.

Launches:
  1. MuJoCo ROS2 Bridge (physics + viewer)
  2. Dummy Tracker Publisher (simulated tracker data)
  3. Arm Teleop Node (tracker -> IK -> joint commands)
  4. Locomotion Node (foot tracker -> gait detection -> base velocity)
  5. Dummy Glove Publisher (simulated glove data)
  6. Hand Teleop Node (glove -> retargeting -> gripper commands)
  7. Dummy HMD Publisher (simulated HMD orientation)
  8. Camera Teleop Node (HMD -> head pan/tilt commands)

Usage:
    source /opt/ros/jazzy/setup.bash
    MUJOCO_GL=glfw ros2 launch teleop_system teleop_mujoco_bridge.launch.py
    MUJOCO_GL=glfw ros2 launch teleop_system teleop_mujoco_bridge.launch.py launch_viewer:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


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
            "tracker_amplitude", default_value="0.08",
            description="Dummy tracker oscillation amplitude (meters)",
        ),
        DeclareLaunchArgument(
            "tracker_frequency", default_value="0.3",
            description="Dummy tracker oscillation frequency (Hz)",
        ),

        # ── MuJoCo ROS2 Bridge ──
        Node(
            package="teleop_system",
            executable="mujoco_bridge",
            name="mujoco_bridge",
            output="screen",
            parameters=[{
                "mjcf_path": "models/rby1/model_teleop.xml",
                "physics_rate_hz": 500.0,
                "publish_rate_hz": 100.0,
                "viewer_rate_hz": 60.0,
                "launch_viewer": LaunchConfiguration("launch_viewer"),
                "publish_camera": LaunchConfiguration("publish_camera"),
                "camera_fps": 15.0,
                "camera_name": "head_camera",
                "camera_width": 640,
                "camera_height": 480,
            }],
        ),

        # ── Dummy Tracker Publisher ──
        Node(
            package="teleop_system",
            executable="dummy_tracker_pub",
            name="dummy_tracker_pub",
            output="screen",
            parameters=[{
                "rate_hz": 100.0,
                "amplitude": LaunchConfiguration("tracker_amplitude"),
                "frequency": LaunchConfiguration("tracker_frequency"),
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

        # ── Dummy Glove Publisher ──
        Node(
            package="teleop_system",
            executable="dummy_glove_pub",
            name="dummy_glove_pub",
            output="screen",
            parameters=[{
                "rate_hz": 100.0,
                "frequency": 0.2,
                "max_angle": 1.4,
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

        # ── Dummy HMD Publisher ──
        Node(
            package="teleop_system",
            executable="dummy_hmd_pub",
            name="dummy_hmd_pub",
            output="screen",
            parameters=[{
                "rate_hz": 90.0,
                "amplitude": 0.3,
                "frequency": 0.15,
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
    ])
