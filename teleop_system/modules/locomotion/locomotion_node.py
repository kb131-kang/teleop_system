"""ROS2 Lifecycle Node for locomotion teleoperation.

Subscribes to foot tracker poses, runs gait detection, and publishes
velocity commands to the mobile base.
"""

from teleop_system.utils.logger import get_logger

logger = get_logger("locomotion_node")

try:
    import rclpy
    from rclpy.lifecycle import Node as LifecycleNode
    from rclpy.callback_group import MutuallyExclusiveCallbackGroup
    from geometry_msgs.msg import PoseStamped, Twist

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — LocomotionNode cannot be used")


if _ROS2_AVAILABLE:
    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        TopicNames,
        get_qos_profile,
    )

    class LocomotionNode(LifecycleNode):
        """ROS2 Lifecycle Node for locomotion teleoperation.

        Subscribes to:
            - /master/tracker/left_foot (PoseStamped)
            - /master/tracker/right_foot (PoseStamped)
        Publishes:
            - /slave/base/cmd_vel (Twist)
        """

        def __init__(self, node_name: str = "locomotion_node"):
            super().__init__(node_name)

            self._cb_group = MutuallyExclusiveCallbackGroup()
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)
            cmd_qos = get_qos_profile(QoSPreset.COMMAND)

            self._latest_left_foot = None
            self._latest_right_foot = None

            self.create_subscription(
                PoseStamped,
                TopicNames.TRACKER_LEFT_FOOT,
                lambda msg: self._foot_callback("left", msg),
                sensor_qos,
                callback_group=self._cb_group,
            )
            self.create_subscription(
                PoseStamped,
                TopicNames.TRACKER_RIGHT_FOOT,
                lambda msg: self._foot_callback("right", msg),
                sensor_qos,
                callback_group=self._cb_group,
            )

            self._pub_cmd_vel = self.create_publisher(
                Twist, TopicNames.BASE_CMD_VEL, cmd_qos
            )

            self.get_logger().info("LocomotionNode created")

        def _foot_callback(self, foot_id: str, msg: PoseStamped):
            if foot_id == "left":
                self._latest_left_foot = msg
            else:
                self._latest_right_foot = msg

        def _publish_velocity(self, linear_x: float, linear_y: float, angular_z: float):
            msg = Twist()
            msg.linear.x = linear_x
            msg.linear.y = linear_y
            msg.angular.z = angular_z
            self._pub_cmd_vel.publish(msg)

else:
    class LocomotionNode:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")
