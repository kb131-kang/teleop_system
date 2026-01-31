"""ROS2 Lifecycle Node for hand teleoperation.

Subscribes to glove joint topics, runs retargeting via HandController,
and publishes hand joint commands.

This node can operate independently of other modules,
communicating only through ROS2 topics.
"""

from teleop_system.utils.logger import get_logger

logger = get_logger("hand_teleop_node")

try:
    import rclpy
    from rclpy.lifecycle import Node as LifecycleNode
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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
        create_timer_rate,
    )
    from teleop_system.modules.hand_teleop.ros2_adapters import (
        ROS2GloveAdapter,
        ROS2HandCommandPublisher,
    )
    from teleop_system.modules.hand_teleop.hand_controller import HandController
    from teleop_system.modules.hand_teleop.retargeting import HandRetargeting

    class HandTeleopNode(LifecycleNode):
        """ROS2 Lifecycle Node for hand teleoperation.

        Subscribes to:
            - /master/hand/left/joints  (JointState)
            - /master/hand/right/joints (JointState)
        Publishes:
            - /slave/hand/left/joint_cmd  (JointState)
            - /slave/hand/right/joint_cmd (JointState)

        Internally creates dual HandController instances (left + right),
        each wired through ROS2GloveAdapter (IHandInput) and
        ROS2HandCommandPublisher (ISlaveHand).
        """

        def __init__(self, node_name: str = "hand_teleop_node"):
            super().__init__(node_name)

            # ── Parameters ──
            self.declare_parameter("rate_hz", 100.0)
            self.declare_parameter("max_joint_velocity", 5.0)
            self.declare_parameter("smoothing_alpha", 0.3)

            rate = self.get_parameter("rate_hz").get_parameter_value().double_value
            max_vel = self.get_parameter("max_joint_velocity").get_parameter_value().double_value
            smoothing = self.get_parameter("smoothing_alpha").get_parameter_value().double_value

            self._cb_group = MutuallyExclusiveCallbackGroup()

            # ── ROS2 Subscribers ──
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)

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

            # ── ROS2 Publishers ──
            cmd_qos = get_qos_profile(QoSPreset.COMMAND)

            self._pub_left = self.create_publisher(
                JointState, TopicNames.HAND_LEFT_CMD, cmd_qos
            )
            self._pub_right = self.create_publisher(
                JointState, TopicNames.HAND_RIGHT_CMD, cmd_qos
            )

            # ── ROS2 Glove Adapters (IHandInput wrappers) ──
            self._glove_left = ROS2GloveAdapter(side="left")
            self._glove_right = ROS2GloveAdapter(side="right")

            self._glove_map = {
                "left": self._glove_left,
                "right": self._glove_right,
            }

            # ── ROS2 Hand Command Publishers (ISlaveHand wrappers) ──
            self._hand_pub_left = ROS2HandCommandPublisher(
                side="left",
                publisher=self._pub_left,
                clock=self.get_clock(),
            )
            self._hand_pub_right = ROS2HandCommandPublisher(
                side="right",
                publisher=self._pub_right,
                clock=self.get_clock(),
            )

            # ── HandRetargeting (pure mapping logic) ──
            left_rt = HandRetargeting(smoothing_alpha=smoothing)
            right_rt = HandRetargeting(smoothing_alpha=smoothing)

            # ── HandControllers (pure control logic) ──
            dt = 1.0 / rate
            self._left_controller = HandController(
                glove=self._glove_left,
                hand=self._hand_pub_left,
                retargeting=left_rt,
                max_joint_velocity=max_vel,
                dt=dt,
            )
            self._right_controller = HandController(
                glove=self._glove_right,
                hand=self._hand_pub_right,
                retargeting=right_rt,
                max_joint_velocity=max_vel,
                dt=dt,
            )

            # Enable controllers — they return None from update() until
            # the glove adapter reports is_connected() == True.
            self._left_controller.enable()
            self._right_controller.enable()

            # ── Control Loop Timer ──
            self._control_timer = self.create_timer(
                create_timer_rate(rate),
                self._control_loop,
                callback_group=self._cb_group,
            )

            self.get_logger().info(
                f"HandTeleopNode created with dual HandControllers "
                f"(rate={rate}Hz, max_vel={max_vel}, smoothing={smoothing})"
            )

        def _glove_callback(self, side: str, msg: JointState) -> None:
            """Feed incoming JointState into the ROS2GloveAdapter."""
            adapter = self._glove_map.get(side)
            if adapter is not None:
                adapter.update_joints(msg)

        def _control_loop(self) -> None:
            """Timer callback: run one HandController update for each side."""
            left_result = self._left_controller.update()
            right_result = self._right_controller.update()

            if left_result is not None or right_result is not None:
                self.get_logger().debug(
                    f"hand cmd: left={'OK' if left_result is not None else '--'} "
                    f"right={'OK' if right_result is not None else '--'}",
                    throttle_duration_sec=2.0,
                )

    def main(args=None):
        """Entry point for the hand teleop node."""
        rclpy.init(args=args)
        node = HandTeleopNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

else:
    # Stub for environments without ROS2
    class HandTeleopNode:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")

    def main(args=None):
        raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")


if __name__ == "__main__":
    main()
