"""Dummy Glove Publisher ROS2 Node.

Wraps SimulatedHand instances in a ROS2 node that publishes
JointState messages on the standard glove topics for testing
without real Manus Glove hardware.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 -m teleop_system.simulators.dummy_glove_pub
"""

from teleop_system.utils.logger import get_logger

logger = get_logger("dummy_glove_pub")

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — DummyGlovePub cannot be used")

if _ROS2_AVAILABLE:
    from teleop_system.simulators.simulated_hand import SimulatedHand
    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        TopicNames,
        get_qos_profile,
    )

    class DummyGlovePub(Node):
        """ROS2 node that publishes simulated glove data for testing.

        Creates SimulatedHand instances for left and right hands,
        and publishes their joint states as JointState at a configurable rate.

        Publishes:
            /master/hand/left/joints  (JointState)
            /master/hand/right/joints (JointState)
        """

        def __init__(self, node_name: str = "dummy_glove_pub"):
            super().__init__(node_name)

            # Parameters
            self.declare_parameter("rate_hz", 100.0)
            self.declare_parameter("frequency", 0.2)
            self.declare_parameter("max_angle", 1.4)

            rate = self.get_parameter("rate_hz").get_parameter_value().double_value
            frequency = self.get_parameter("frequency").get_parameter_value().double_value
            max_angle = self.get_parameter("max_angle").get_parameter_value().double_value

            # Create simulated hands
            self._hands = {
                "left": SimulatedHand(side="left", frequency=frequency, max_angle=max_angle),
                "right": SimulatedHand(side="right", frequency=frequency, max_angle=max_angle),
            }
            for hand in self._hands.values():
                hand.initialize()

            # Create publishers
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)
            self._pubs = {
                "left": self.create_publisher(
                    JointState, TopicNames.HAND_LEFT_JOINTS, sensor_qos
                ),
                "right": self.create_publisher(
                    JointState, TopicNames.HAND_RIGHT_JOINTS, sensor_qos
                ),
            }

            # Timer
            self._timer = self.create_timer(1.0 / rate, self._publish_all)

            self.get_logger().info(
                f"DummyGlovePub started: 2 hands at {rate} Hz "
                f"(frequency={frequency}, max_angle={max_angle})"
            )

        def _publish_all(self) -> None:
            """Read each simulated hand and publish as JointState."""
            for side, hand in self._hands.items():
                state = hand.get_joint_state()
                if not state.valid:
                    continue
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.position = state.joint_angles.tolist()
                self._pubs[side].publish(msg)

        def destroy_node(self):
            for hand in self._hands.values():
                hand.shutdown()
            super().destroy_node()


def main(args=None):
    """Entry point for the dummy glove publisher."""
    if not _ROS2_AVAILABLE:
        raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")

    rclpy.init(args=args)
    node = DummyGlovePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
