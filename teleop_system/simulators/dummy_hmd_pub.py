"""Dummy HMD Publisher ROS2 Node.

Wraps a SimulatedTracker(HEAD) in a ROS2 node that publishes
QuaternionStamped messages on /master/hmd/orientation for testing
without real VR HMD hardware.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 -m teleop_system.simulators.dummy_hmd_pub
"""

from teleop_system.utils.logger import get_logger

logger = get_logger("dummy_hmd_pub")

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import QuaternionStamped

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — DummyHMDPub cannot be used")

if _ROS2_AVAILABLE:
    from teleop_system.interfaces.master_device import TrackerRole
    from teleop_system.simulators.simulated_tracker import SimulatedTracker
    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        TopicNames,
        get_qos_profile,
    )

    class DummyHMDPub(Node):
        """ROS2 node that publishes simulated HMD orientation data for testing.

        Creates a SimulatedTracker(HEAD) internally and publishes only the
        orientation component as QuaternionStamped on /master/hmd/orientation.

        Publishes:
            /master/hmd/orientation (QuaternionStamped)
        """

        def __init__(self, node_name: str = "dummy_hmd_pub"):
            super().__init__(node_name)

            # Parameters
            self.declare_parameter("rate_hz", 90.0)
            self.declare_parameter("amplitude", 1.3)
            self.declare_parameter("frequency", 0.15)

            rate = self.get_parameter("rate_hz").get_parameter_value().double_value
            amplitude = self.get_parameter("amplitude").get_parameter_value().double_value
            frequency = self.get_parameter("frequency").get_parameter_value().double_value

            # Create simulated HMD tracker
            self._tracker = SimulatedTracker(
                role=TrackerRole.HEAD,
                amplitude=amplitude,
                frequency=frequency,
            )
            self._tracker.initialize()

            # Create publisher
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)
            self._pub = self.create_publisher(
                QuaternionStamped, TopicNames.HMD_ORIENTATION, sensor_qos
            )

            # Timer
            self._timer = self.create_timer(1.0 / rate, self._publish_hmd)

            self.get_logger().info(
                f"DummyHMDPub started at {rate} Hz "
                f"(amplitude={amplitude}, frequency={frequency})"
            )

        def _publish_hmd(self) -> None:
            """Read SimulatedTracker(HEAD) orientation, publish as QuaternionStamped."""
            pose = self._tracker.get_pose()
            if not pose.valid:
                return

            msg = QuaternionStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "hmd"
            msg.quaternion.x = float(pose.orientation[0])
            msg.quaternion.y = float(pose.orientation[1])
            msg.quaternion.z = float(pose.orientation[2])
            msg.quaternion.w = float(pose.orientation[3])
            self._pub.publish(msg)

        def destroy_node(self):
            self._tracker.shutdown()
            super().destroy_node()


def main(args=None):
    """Entry point for the dummy HMD publisher."""
    if not _ROS2_AVAILABLE:
        raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")

    rclpy.init(args=args)
    node = DummyHMDPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
