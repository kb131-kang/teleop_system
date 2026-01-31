"""ROS2 Lifecycle Node for locomotion teleoperation.

Subscribes to foot tracker poses, runs gait detection via LocomotionController,
and publishes velocity commands to the mobile base.

This node can operate independently of other modules,
communicating only through ROS2 topics.
"""

from teleop_system.utils.logger import get_logger

logger = get_logger("locomotion_node")

try:
    import rclpy
    from rclpy.lifecycle import Node as LifecycleNode
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from geometry_msgs.msg import PoseStamped, Twist

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — LocomotionNode cannot be used")


if _ROS2_AVAILABLE:
    from teleop_system.interfaces.master_device import TrackerRole
    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        TopicNames,
        get_qos_profile,
        create_timer_rate,
    )
    from teleop_system.modules.arm_teleop.ros2_adapters import ROS2TrackerAdapter
    from teleop_system.modules.locomotion.ros2_adapters import ROS2BaseCommandPublisher
    from teleop_system.modules.locomotion.locomotion_controller import LocomotionController

    class LocomotionNode(LifecycleNode):
        """ROS2 Lifecycle Node for locomotion teleoperation.

        Subscribes to:
            - /master/tracker/left_foot (PoseStamped)
            - /master/tracker/right_foot (PoseStamped)
        Publishes:
            - /slave/base/cmd_vel (Twist)

        Internally creates ROS2 adapter classes to bridge ROS2 messages
        with LocomotionController's IMasterTracker/IMobileBase interfaces.
        """

        def __init__(self, node_name: str = "locomotion_node"):
            super().__init__(node_name)

            # ── Parameters ──
            self.declare_parameter("rate_hz", 50.0)
            self.declare_parameter("deadzone", 0.02)
            self.declare_parameter("linear_scale", 1.0)
            self.declare_parameter("angular_scale", 1.0)
            self.declare_parameter("max_linear_velocity", 0.5)
            self.declare_parameter("max_angular_velocity", 1.0)
            self.declare_parameter("smoothing_window", 5)

            rate = self.get_parameter("rate_hz").get_parameter_value().double_value
            deadzone = self.get_parameter("deadzone").get_parameter_value().double_value
            linear_scale = self.get_parameter("linear_scale").get_parameter_value().double_value
            angular_scale = self.get_parameter("angular_scale").get_parameter_value().double_value
            max_linear_vel = self.get_parameter("max_linear_velocity").get_parameter_value().double_value
            max_angular_vel = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
            smoothing_window = self.get_parameter("smoothing_window").get_parameter_value().integer_value

            self._cb_group = MutuallyExclusiveCallbackGroup()

            # ── ROS2 Subscribers ──
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)

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

            # ── ROS2 Publisher ──
            cmd_qos = get_qos_profile(QoSPreset.COMMAND)
            self._pub_cmd_vel = self.create_publisher(
                Twist, TopicNames.BASE_CMD_VEL, cmd_qos
            )

            # ── ROS2 Tracker Adapters (IMasterTracker wrappers) ──
            self._tracker_left = ROS2TrackerAdapter(TrackerRole.LEFT_FOOT)
            self._tracker_right = ROS2TrackerAdapter(TrackerRole.RIGHT_FOOT)

            self._tracker_map = {
                "left": self._tracker_left,
                "right": self._tracker_right,
            }

            # ── ROS2 Base Command Publisher (IMobileBase wrapper) ──
            self._base_pub = ROS2BaseCommandPublisher(
                publisher=self._pub_cmd_vel,
                clock=self.get_clock(),
            )

            # ── LocomotionController (pure control logic) ──
            self._controller = LocomotionController(
                left_foot_tracker=self._tracker_left,
                right_foot_tracker=self._tracker_right,
                base=self._base_pub,
                deadzone=deadzone,
                linear_scale=linear_scale,
                angular_scale=angular_scale,
                max_linear_velocity=max_linear_vel,
                max_angular_velocity=max_angular_vel,
                smoothing_window=smoothing_window,
            )
            # Don't enable yet — trackers have no data until ROS2 messages arrive.
            # The control loop will auto-calibrate once data is available.

            # ── Control Loop Timer ──
            self._control_timer = self.create_timer(
                create_timer_rate(rate),
                self._control_loop,
                callback_group=self._cb_group,
            )

            self.get_logger().info(
                f"LocomotionNode created with LocomotionController "
                f"(rate={rate}Hz, deadzone={deadzone}, "
                f"linear_scale={linear_scale}, angular_scale={angular_scale})"
            )

        def _foot_callback(self, foot_id: str, msg: PoseStamped) -> None:
            """Feed incoming PoseStamped into the ROS2TrackerAdapter."""
            adapter = self._tracker_map.get(foot_id)
            if adapter is not None:
                adapter.update_pose(msg)

        def _control_loop(self) -> None:
            """Timer callback: run one LocomotionController update cycle.

            Auto-calibrates once both foot trackers provide valid data.
            Then runs gait detection and dispatches velocity commands.
            """
            # Auto-calibrate once trackers have data
            if not self._controller.is_enabled:
                if self._tracker_left.is_connected() and self._tracker_right.is_connected():
                    ok = self._controller.enable()
                    if ok:
                        self.get_logger().info("LocomotionController auto-calibrated and enabled")
                    else:
                        self.get_logger().debug(
                            "Calibration attempt failed, will retry",
                            throttle_duration_sec=2.0,
                        )
                return

            cmd = self._controller.update()
            if cmd.linear_x != 0.0 or cmd.linear_y != 0.0 or cmd.angular_z != 0.0:
                self.get_logger().debug(
                    f"vel: x={cmd.linear_x:.3f} y={cmd.linear_y:.3f} "
                    f"yaw={cmd.angular_z:.3f}",
                    throttle_duration_sec=2.0,
                )

    def main(args=None):
        """Entry point for the locomotion node."""
        rclpy.init(args=args)
        node = LocomotionNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

else:
    # Stub for environments without ROS2
    class LocomotionNode:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")

    def main(args=None):
        raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")


if __name__ == "__main__":
    main()
