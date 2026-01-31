"""ROS2 Lifecycle Node for hand teleoperation.

Subscribes to glove joint topics, runs retargeting, and publishes
hand joint commands.
"""

from teleop_system.utils.logger import get_logger

logger = get_logger("hand_teleop_node")

try:
    import rclpy
    from rclpy.lifecycle import Node as LifecycleNode
    from rclpy.callback_group import MutuallyExclusiveCallbackGroup
    from sensor_msgs.msg import JointState

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — HandTeleopNode cannot be used")


if _ROS2_AVAILABLE:
    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        TopicNames,
        get_qos_profile,
    )

    class HandTeleopNode(LifecycleNode):
        """ROS2 Lifecycle Node for hand teleoperation.

        Subscribes to:
            - /master/hand/{left,right}/joints (JointState)
        Publishes:
            - /slave/hand/{left,right}/joint_cmd (JointState)
        """

        def __init__(self, node_name: str = "hand_teleop_node"):
            super().__init__(node_name)

            self._cb_group = MutuallyExclusiveCallbackGroup()
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)
            cmd_qos = get_qos_profile(QoSPreset.COMMAND)

            self._latest_left = None
            self._latest_right = None

            self.create_subscription(
                JointState,
                TopicNames.HAND_LEFT_JOINTS,
                lambda msg: self._glove_callback("left", msg),
                sensor_qos,
                callback_group=self._cb_group,
            )
            self.create_subscription(
                JointState,
                TopicNames.HAND_RIGHT_JOINTS,
                lambda msg: self._glove_callback("right", msg),
                sensor_qos,
                callback_group=self._cb_group,
            )

            self._pub_left = self.create_publisher(
                JointState, TopicNames.HAND_LEFT_CMD, cmd_qos
            )
            self._pub_right = self.create_publisher(
                JointState, TopicNames.HAND_RIGHT_CMD, cmd_qos
            )

            self.get_logger().info("HandTeleopNode created")

        def _glove_callback(self, side: str, msg: JointState):
            if side == "left":
                self._latest_left = msg
            else:
                self._latest_right = msg

        def _publish_hand_command(self, side: str, positions):
            publisher = self._pub_left if side == "left" else self._pub_right
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = positions.tolist()
            publisher.publish(msg)

else:
    class HandTeleopNode:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")
