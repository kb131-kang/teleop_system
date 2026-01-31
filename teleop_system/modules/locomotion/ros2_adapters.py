"""ROS2 adapter classes for LocomotionController integration.

Bridges ROS2 message-based communication and the IMobileBase interface
that LocomotionController expects. Allows LocomotionController to work
unchanged in a ROS2 context.
"""

import numpy as np

from teleop_system.interfaces.slave_robot import IMobileBase
from teleop_system.utils.logger import get_logger

logger = get_logger("locomotion_ros2_adapters")


class ROS2BaseCommandPublisher(IMobileBase):
    """Adapts IMobileBase.send_velocity() to publish Twist on a ROS2 topic.

    When LocomotionController calls send_velocity(), this adapter publishes
    a geometry_msgs/Twist message via the injected ROS2 publisher.
    """

    def __init__(self, publisher, clock):
        """Initialize the ROS2 base command publisher adapter.

        Args:
            publisher: rclpy Publisher for geometry_msgs/Twist.
            clock: rclpy Clock for timestamping (unused for Twist but kept for consistency).
        """
        self._publisher = publisher
        self._clock = clock
        self._last_cmd = (0.0, 0.0, 0.0)

    def send_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        """Publish velocity as a geometry_msgs/Twist message."""
        from geometry_msgs.msg import Twist

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self._publisher.publish(msg)
        self._last_cmd = (linear_x, linear_y, angular_z)

    def initialize(self) -> bool:
        return True

    def get_odometry(self) -> tuple[np.ndarray, np.ndarray]:
        # No feedback from ROS2 topic direction; return zeros
        return np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0])

    def stop(self) -> None:
        self.send_velocity(0.0, 0.0, 0.0)

    def is_connected(self) -> bool:
        return True

    def shutdown(self) -> None:
        self.stop()
