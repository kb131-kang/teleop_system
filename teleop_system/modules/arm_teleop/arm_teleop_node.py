"""ROS2 Lifecycle Node for arm teleoperation.

Subscribes to master tracker poses, runs IK via ArmController,
and publishes joint commands to the slave robot.

This node can operate independently of other modules,
communicating only through ROS2 topics.
"""

from teleop_system.utils.logger import get_logger

logger = get_logger("arm_teleop_node")

try:
    import rclpy
    from rclpy.lifecycle import Node as LifecycleNode
    from rclpy.callback_group import MutuallyExclusiveCallbackGroup
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — ArmTeleopNode cannot be used")


if _ROS2_AVAILABLE:
    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        TopicNames,
        get_qos_profile,
        create_timer_rate,
    )

    class ArmTeleopNode(LifecycleNode):
        """ROS2 Lifecycle Node for arm teleoperation.

        Subscribes to:
            - /master/tracker/{left,right,waist} (PoseStamped)
        Publishes:
            - /slave/arm/{left,right}/joint_cmd (JointState)
            - /slave/torso/joint_cmd (JointState)
        """

        def __init__(self, node_name: str = "arm_teleop_node"):
            super().__init__(node_name)

            self._cb_group = MutuallyExclusiveCallbackGroup()

            # Latest tracker data (thread-safe via callback group)
            self._latest_poses = {
                "right": None,
                "left": None,
                "waist": None,
            }

            # Subscribers
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)

            self.create_subscription(
                PoseStamped,
                TopicNames.TRACKER_RIGHT_HAND,
                lambda msg: self._tracker_callback("right", msg),
                sensor_qos,
                callback_group=self._cb_group,
            )
            self.create_subscription(
                PoseStamped,
                TopicNames.TRACKER_LEFT_HAND,
                lambda msg: self._tracker_callback("left", msg),
                sensor_qos,
                callback_group=self._cb_group,
            )
            self.create_subscription(
                PoseStamped,
                TopicNames.TRACKER_WAIST,
                lambda msg: self._tracker_callback("waist", msg),
                sensor_qos,
                callback_group=self._cb_group,
            )

            # Publishers
            cmd_qos = get_qos_profile(QoSPreset.COMMAND)

            self._pub_right = self.create_publisher(
                JointState, TopicNames.ARM_RIGHT_CMD, cmd_qos
            )
            self._pub_left = self.create_publisher(
                JointState, TopicNames.ARM_LEFT_CMD, cmd_qos
            )
            self._pub_torso = self.create_publisher(
                JointState, TopicNames.TORSO_CMD, cmd_qos
            )

            self.get_logger().info("ArmTeleopNode created")

        def _tracker_callback(self, tracker_id: str, msg: PoseStamped):
            self._latest_poses[tracker_id] = msg

        def _publish_joint_command(self, publisher, positions, names=None):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = positions.tolist()
            if names:
                msg.name = names
            publisher.publish(msg)
else:
    # Stub for environments without ROS2
    class ArmTeleopNode:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")
