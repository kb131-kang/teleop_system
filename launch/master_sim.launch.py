"""Launch file for the master system (simulated VR inputs).

Launches simulated sensor publishers and teleop processing nodes:
  - Dummy Tracker Publisher (simulated VR tracker data)
  - Dummy Glove Publisher (simulated hand glove data)
  - Dummy HMD Publisher (simulated head-mounted display orientation)
  - Arm Teleop Node (tracker -> IK -> joint commands)
  - Locomotion Node (foot tracker -> gait -> base velocity)
  - Hand Teleop Node (glove -> retargeting -> gripper commands)
  - Camera Teleop Node (HMD -> head pan/tilt commands)
  - (Optional) ROS2 RGB-D Viewer for first-person camera view

Usage:
    ros2 launch teleop_system master_sim.launch.py
    ros2 launch teleop_system master_sim.launch.py tracker_amplitude:=0.10
    ros2 launch teleop_system master_sim.launch.py launch_viewer:=true
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
            "tracker_amplitude", default_value="0.08",
            description="Dummy tracker oscillation amplitude (meters)",
        ),
        DeclareLaunchArgument(
            "tracker_frequency", default_value="0.3",
            description="Dummy tracker oscillation frequency (Hz)",
        ),
        DeclareLaunchArgument(
            "glove_frequency", default_value="0.2",
            description="Dummy glove oscillation frequency (Hz)",
        ),
        DeclareLaunchArgument(
            "glove_max_angle", default_value="1.4",
            description="Dummy glove maximum finger angle (rad)",
        ),
        DeclareLaunchArgument(
            "hmd_amplitude", default_value="0.3",
            description="Dummy HMD oscillation amplitude (rad)",
        ),
        DeclareLaunchArgument(
            "hmd_frequency", default_value="0.15",
            description="Dummy HMD oscillation frequency (Hz)",
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

        # ── Dummy Glove Publisher ──
        Node(
            package="teleop_system",
            executable="dummy_glove_pub",
            name="dummy_glove_pub",
            output="screen",
            parameters=[{
                "rate_hz": 100.0,
                "frequency": LaunchConfiguration("glove_frequency"),
                "max_angle": LaunchConfiguration("glove_max_angle"),
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
                "amplitude": LaunchConfiguration("hmd_amplitude"),
                "frequency": LaunchConfiguration("hmd_frequency"),
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
