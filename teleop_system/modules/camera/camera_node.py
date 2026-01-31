"""ROS2 Lifecycle Node for camera/head teleoperation.

Subscribes to HMD orientation topic, runs CameraController to extract
pan/tilt angles, and publishes head joint commands.

This node can operate independently of other modules,
communicating only through ROS2 topics.
"""

from teleop_system.utils.logger import get_logger

logger = get_logger("camera_teleop_node")

try:
    import rclpy
    from rclpy.lifecycle import Node as LifecycleNode
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from geometry_msgs.msg import QuaternionStamped
    from sensor_msgs.msg import JointState

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — CameraTeleopNode cannot be used")


if _ROS2_AVAILABLE:
    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        TopicNames,
        get_qos_profile,
        create_timer_rate,
    )
    from teleop_system.modules.camera.ros2_adapters import (
        ROS2HMDAdapter,
        ROS2CameraAdapter,
    )
    from teleop_system.modules.camera.camera_controller import CameraController

    class CameraTeleopNode(LifecycleNode):
        """ROS2 Lifecycle Node for camera/head teleoperation.

        Subscribes to:
            /master/hmd/orientation  (QuaternionStamped) -- HMD head orientation

        Publishes:
            /slave/camera/pan_tilt_cmd  (JointState) -- head pan/tilt commands

        Internally creates a CameraController wired through
        ROS2HMDAdapter (IMasterTracker) and ROS2CameraAdapter (ICameraStream).
        """

        def __init__(self, node_name: str = "camera_teleop_node"):
            super().__init__(node_name)

            # ── Parameters ──
            self.declare_parameter("rate_hz", 30.0)
            self.declare_parameter("smoothing_alpha", 0.3)
            self.declare_parameter("max_angular_velocity", 2.0)

            rate = self.get_parameter("rate_hz").get_parameter_value().double_value
            smoothing = self.get_parameter("smoothing_alpha").get_parameter_value().double_value
            max_vel = self.get_parameter("max_angular_velocity").get_parameter_value().double_value

            self._cb_group = MutuallyExclusiveCallbackGroup()

            # ── ROS2 Subscriber ──
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)

            self.create_subscription(
                QuaternionStamped,
                TopicNames.HMD_ORIENTATION,
                self._hmd_callback,
                sensor_qos,
                callback_group=self._cb_group,
            )

            # ── ROS2 Publisher ──
            cmd_qos = get_qos_profile(QoSPreset.COMMAND)

            self._pan_tilt_pub = self.create_publisher(
                JointState, TopicNames.CAMERA_PAN_TILT_CMD, cmd_qos
            )

            # ── ROS2 HMD Adapter (IMasterTracker wrapper) ──
            self._hmd_adapter = ROS2HMDAdapter()

            # ── ROS2 Camera Adapter (ICameraStream wrapper) ──
            self._camera_adapter = ROS2CameraAdapter(
                publisher=self._pan_tilt_pub,
                clock=self.get_clock(),
            )

            # ── CameraController (pure control logic) ──
            dt = 1.0 / rate
            self._controller = CameraController(
                tracker=self._hmd_adapter,
                camera=self._camera_adapter,
                smoothing_alpha=smoothing,
                max_angular_velocity=max_vel,
                dt=dt,
            )

            # Enable controller — returns None from update() until
            # the HMD adapter reports is_connected() == True.
            self._controller.enable()

            # ── Control Loop Timer ──
            self._control_timer = self.create_timer(
                create_timer_rate(rate),
                self._control_loop,
                callback_group=self._cb_group,
            )

            self.get_logger().info(
                f"CameraTeleopNode created with CameraController "
                f"(rate={rate}Hz, smoothing={smoothing}, max_vel={max_vel})"
            )

        def _hmd_callback(self, msg: QuaternionStamped) -> None:
            """Feed incoming QuaternionStamped into the ROS2HMDAdapter."""
            self._hmd_adapter.update_orientation(msg)

        def _control_loop(self) -> None:
            """Timer callback: run one CameraController update cycle."""
            result = self._controller.update()

            if result is not None:
                self.get_logger().debug(
                    f"camera cmd: pan={result[0]:.3f} tilt={result[1]:.3f}",
                    throttle_duration_sec=2.0,
                )

    def main(args=None):
        """Entry point for the camera teleop node."""
        rclpy.init(args=args)
        node = CameraTeleopNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

else:
    # Stub for environments without ROS2
    class CameraTeleopNode:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")

    def main(args=None):
        raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")


if __name__ == "__main__":
    main()
