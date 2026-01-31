"""MuJoCo ROS2 Bridge Node.

Runs MuJoCo physics simulation and bridges joint commands/states
via ROS2 topics. Subscribes to joint command topics from teleop modules
and publishes the simulated robot's joint states.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 -m teleop_system.simulators.mujoco_ros2_bridge --launch-viewer
"""

import os
import sys
from pathlib import Path

from teleop_system.utils.logger import get_logger

logger = get_logger("mujoco_ros2_bridge")

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Twist

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — MuJoCoROS2Bridge cannot be used")

if _ROS2_AVAILABLE:
    import numpy as np

    from teleop_system.simulators.mujoco_sim import MuJoCoSimulator
    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        TopicNames,
        get_qos_profile,
    )

    # ── MuJoCo actuator ctrl index mapping ──
    # From model_teleop.xml: 26 actuators total
    CTRL_LEFT_WHEEL = 0
    CTRL_RIGHT_WHEEL = 1
    CTRL_TORSO = slice(2, 8)        # 6 joints
    CTRL_RIGHT_ARM = slice(8, 15)   # 7 joints
    CTRL_LEFT_ARM = slice(15, 22)   # 7 joints
    CTRL_HEAD = slice(22, 24)       # 2 joints
    CTRL_RIGHT_GRIPPER = 24
    CTRL_LEFT_GRIPPER = 25

    class MuJoCoROS2Bridge(Node):
        """ROS2 node that runs MuJoCo physics and bridges joint commands/states.

        Subscribes to:
            /slave/arm/right/joint_cmd  (JointState) -> ctrl[8:15]
            /slave/arm/left/joint_cmd   (JointState) -> ctrl[15:22]
            /slave/torso/joint_cmd      (JointState) -> ctrl[2:8]
            /slave/base/cmd_vel         (Twist)      -> ctrl[0:2] (wheels)
            /slave/hand/right/joint_cmd (JointState) -> ctrl[24]
            /slave/hand/left/joint_cmd  (JointState) -> ctrl[25]

        Publishes:
            /mujoco/joint_states (JointState) at publish_rate_hz
        """

        def __init__(self, node_name: str = "mujoco_bridge"):
            super().__init__(node_name)

            # ── Parameters ──
            self.declare_parameter("mjcf_path", "models/rby1/model_teleop.xml")
            self.declare_parameter("physics_rate_hz", 500.0)
            self.declare_parameter("publish_rate_hz", 100.0)
            self.declare_parameter("viewer_rate_hz", 60.0)
            self.declare_parameter("launch_viewer", False)

            mjcf_path = self.get_parameter("mjcf_path").get_parameter_value().string_value
            physics_rate = self.get_parameter("physics_rate_hz").get_parameter_value().double_value
            publish_rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
            viewer_rate = self.get_parameter("viewer_rate_hz").get_parameter_value().double_value
            launch_viewer = self.get_parameter("launch_viewer").get_parameter_value().bool_value

            # ── Resolve model path relative to project root ──
            project_root = Path(__file__).resolve().parent.parent.parent
            full_path = str(project_root / mjcf_path)

            # ── Initialize MuJoCo Simulator ──
            self._sim = MuJoCoSimulator()
            success = self._sim.initialize({
                "mjcf_path": full_path,
                "timestep": 0.002,
                "render": False,
            })
            if not success:
                self.get_logger().error(f"Failed to initialize MuJoCo from {full_path}")
                raise RuntimeError("MuJoCo initialization failed")

            self._all_joint_names = self._sim.get_joint_names()
            self.get_logger().info(
                f"MuJoCo initialized: {len(self._all_joint_names)} joints, "
                f"physics={physics_rate}Hz, publish={publish_rate}Hz"
            )

            # ── Optional passive viewer ──
            if launch_viewer:
                self._sim.launch_passive_viewer()
                self.get_logger().info("MuJoCo passive viewer launched")

            # ── Subscribers for joint commands ──
            cmd_qos = get_qos_profile(QoSPreset.COMMAND)

            self.create_subscription(
                JointState, TopicNames.ARM_RIGHT_CMD,
                lambda msg: self._joint_cmd_callback(msg, CTRL_RIGHT_ARM),
                cmd_qos,
            )
            self.create_subscription(
                JointState, TopicNames.ARM_LEFT_CMD,
                lambda msg: self._joint_cmd_callback(msg, CTRL_LEFT_ARM),
                cmd_qos,
            )
            self.create_subscription(
                JointState, TopicNames.TORSO_CMD,
                lambda msg: self._joint_cmd_callback(msg, CTRL_TORSO),
                cmd_qos,
            )
            self.create_subscription(
                Twist, TopicNames.BASE_CMD_VEL,
                self._base_cmd_callback,
                cmd_qos,
            )
            self.create_subscription(
                JointState, TopicNames.HAND_RIGHT_CMD,
                lambda msg: self._gripper_cmd_callback(msg, CTRL_RIGHT_GRIPPER),
                cmd_qos,
            )
            self.create_subscription(
                JointState, TopicNames.HAND_LEFT_CMD,
                lambda msg: self._gripper_cmd_callback(msg, CTRL_LEFT_GRIPPER),
                cmd_qos,
            )
            self.create_subscription(
                JointState, TopicNames.CAMERA_PAN_TILT_CMD,
                lambda msg: self._joint_cmd_callback(msg, CTRL_HEAD),
                cmd_qos,
            )

            # ── Publisher for joint states ──
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)
            self._joint_state_pub = self.create_publisher(
                JointState, "/mujoco/joint_states", sensor_qos
            )

            # ── Timers ──
            self._physics_timer = self.create_timer(
                1.0 / physics_rate, self._physics_step
            )
            self._publish_timer = self.create_timer(
                1.0 / publish_rate, self._publish_joint_states
            )
            if launch_viewer:
                self._viewer_timer = self.create_timer(
                    1.0 / viewer_rate, self._sync_viewer
                )

            self.get_logger().info("MuJoCoROS2Bridge ready")

        def _joint_cmd_callback(self, msg: JointState, ctrl_slice: slice) -> None:
            """Apply incoming joint positions to MuJoCo ctrl at the given indices."""
            if not msg.position:
                return
            positions = np.array(msg.position)
            n_expected = ctrl_slice.stop - ctrl_slice.start
            n = min(len(positions), n_expected)
            self._sim._data.ctrl[ctrl_slice.start:ctrl_slice.start + n] = positions[:n]

        def _base_cmd_callback(self, msg: Twist) -> None:
            """Convert Twist to differential drive wheel velocities."""
            wheel_base = 0.35  # approximate wheel separation (meters)
            v_left = msg.linear.x - msg.angular.z * wheel_base / 2.0
            v_right = msg.linear.x + msg.angular.z * wheel_base / 2.0
            self._sim._data.ctrl[CTRL_LEFT_WHEEL] = v_left
            self._sim._data.ctrl[CTRL_RIGHT_WHEEL] = v_right

        def _gripper_cmd_callback(self, msg: JointState, ctrl_index: int) -> None:
            """Apply gripper command to a single ctrl index.

            Averages the incoming joint positions and scales by 20.0 to convert
            from joint angles (radians) to motor force (N) for visible gripper motion.
            """
            if msg.position:
                self._sim._data.ctrl[ctrl_index] = float(np.mean(msg.position)) * 20.0

        def _physics_step(self) -> None:
            """Step the MuJoCo simulation forward."""
            self._sim.step()

        def _publish_joint_states(self) -> None:
            """Publish current joint states from MuJoCo."""
            state = self._sim.get_state()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self._all_joint_names
            msg.position = state.joint_positions.tolist()
            msg.velocity = state.joint_velocities.tolist()
            self._joint_state_pub.publish(msg)

        def _sync_viewer(self) -> None:
            """Sync the passive viewer with simulation state."""
            self._sim.sync_viewer()

        def destroy_node(self):
            self._sim.shutdown()
            super().destroy_node()


def main(args=None):
    """Entry point for the MuJoCo ROS2 bridge node.

    Supports --launch-viewer flag from command line.
    """
    if not _ROS2_AVAILABLE:
        raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")

    # Parse --launch-viewer flag before passing args to rclpy
    argv = args if args is not None else sys.argv
    launch_viewer = "--launch-viewer" in argv
    filtered_args = [a for a in argv if a != "--launch-viewer"]

    rclpy.init(args=filtered_args)

    node = MuJoCoROS2Bridge()

    # Override launch_viewer parameter if CLI flag was passed
    if launch_viewer and not hasattr(node, '_viewer_timer'):
        node._sim.launch_passive_viewer()
        node._viewer_timer = node.create_timer(1.0 / 60.0, node._sync_viewer)
        node.get_logger().info("Passive viewer launched via --launch-viewer flag")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
