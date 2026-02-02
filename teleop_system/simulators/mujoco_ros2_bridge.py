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
    from sensor_msgs.msg import JointState, Image, CameraInfo
    from geometry_msgs.msg import Twist

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — MuJoCoROS2Bridge cannot be used")

if _ROS2_AVAILABLE:
    import numpy as np
    from rcl_interfaces.msg import ParameterDescriptor

    from teleop_system.simulators.mujoco_sim import MuJoCoSimulator
    from std_srvs.srv import Trigger

    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        ServiceNames,
        TopicNames,
        get_qos_profile,
    )

    # Allow launch files to pass int/string/double interchangeably
    _DYNAMIC_TYPE = ParameterDescriptor(dynamic_typing=True)

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

    def _get_float_param(node, name: str) -> float:
        """Get a ROS2 parameter as float, handling int/double/string types.

        ROS2 LaunchConfiguration passes all values as strings. Depending on
        the string content (e.g. "30" vs "30.0"), the parameter may be stored
        as integer, double, or string. This helper normalizes to float.
        """
        pv = node.get_parameter(name).get_parameter_value()
        if pv.type == 2:  # INTEGER
            return float(pv.integer_value)
        elif pv.type == 3:  # DOUBLE
            return pv.double_value
        elif pv.type == 4:  # STRING
            return float(pv.string_value)
        return pv.double_value

    def _get_bool_param(node, name: str) -> bool:
        """Get a ROS2 parameter as bool, handling string type from launch."""
        pv = node.get_parameter(name).get_parameter_value()
        if pv.type == 1:  # BOOL
            return pv.bool_value
        elif pv.type == 4:  # STRING
            return pv.string_value.lower() in ("true", "1", "yes")
        return pv.bool_value

    def _get_int_param(node, name: str) -> int:
        """Get a ROS2 parameter as int, handling string/double types from launch."""
        pv = node.get_parameter(name).get_parameter_value()
        if pv.type == 2:  # INTEGER
            return pv.integer_value
        elif pv.type == 3:  # DOUBLE
            return int(pv.double_value)
        elif pv.type == 4:  # STRING
            return int(pv.string_value)
        return pv.integer_value

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
            # Numeric/bool params use dynamic_typing so launch files can pass
            # int, double, or string interchangeably (e.g. camera_fps:=30 vs 30.0)
            self.declare_parameter("mjcf_path", "models/rby1/model_teleop.xml")
            self.declare_parameter("physics_rate_hz", 500.0, _DYNAMIC_TYPE)
            self.declare_parameter("publish_rate_hz", 100.0, _DYNAMIC_TYPE)
            self.declare_parameter("viewer_rate_hz", 60.0, _DYNAMIC_TYPE)
            self.declare_parameter("launch_viewer", False, _DYNAMIC_TYPE)
            self.declare_parameter("publish_camera", False, _DYNAMIC_TYPE)
            self.declare_parameter("camera_fps", 15.0, _DYNAMIC_TYPE)
            self.declare_parameter("camera_name", "head_camera")
            self.declare_parameter("camera_width", 640, _DYNAMIC_TYPE)
            self.declare_parameter("camera_height", 480, _DYNAMIC_TYPE)

            mjcf_path = self.get_parameter("mjcf_path").get_parameter_value().string_value
            physics_rate = _get_float_param(self, "physics_rate_hz")
            publish_rate = _get_float_param(self, "publish_rate_hz")
            viewer_rate = _get_float_param(self, "viewer_rate_hz")
            launch_viewer = _get_bool_param(self, "launch_viewer")
            publish_camera = _get_bool_param(self, "publish_camera")
            camera_fps = _get_float_param(self, "camera_fps")
            self._camera_name = self.get_parameter("camera_name").get_parameter_value().string_value
            camera_width = _get_int_param(self, "camera_width")
            camera_height = _get_int_param(self, "camera_height")

            # ── Resolve model path relative to project root ──
            project_root = Path(__file__).resolve().parent.parent.parent
            full_path = project_root / mjcf_path

            # Fallback: if running from a colcon install tree (not --symlink-install),
            # __file__ is inside install/ and the model won't be found.
            # Try the current working directory as a fallback.
            if not full_path.exists():
                cwd_path = Path.cwd() / mjcf_path
                if cwd_path.exists():
                    full_path = cwd_path
                    self.get_logger().info(
                        f"Model resolved via CWD: {full_path}"
                    )
                else:
                    self.get_logger().error(
                        f"Model not found: {full_path}\n"
                        f"  CWD fallback also failed: {cwd_path}\n"
                        f"  If using colcon build, rebuild with --symlink-install:\n"
                        f"    colcon build --packages-select teleop_system --symlink-install"
                    )
                    raise FileNotFoundError(f"MuJoCo model not found: {mjcf_path}")

            full_path = str(full_path)

            # ── Initialize MuJoCo Simulator ──
            # Enable rendering when camera publishing is requested
            self._sim = MuJoCoSimulator()
            success = self._sim.initialize({
                "mjcf_path": full_path,
                "timestep": 0.002,
                "render": publish_camera,
                "render_width": camera_width,
                "render_height": camera_height,
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

            # ── Optional camera RGB-D publishers ──
            self._publish_camera = publish_camera
            if publish_camera:
                self._camera_color_pub = self.create_publisher(
                    Image, TopicNames.CAMERA_COLOR_IMAGE, sensor_qos
                )
                self._camera_depth_pub = self.create_publisher(
                    Image, TopicNames.CAMERA_DEPTH_IMAGE, sensor_qos
                )
                self._camera_info_pub = self.create_publisher(
                    CameraInfo, TopicNames.CAMERA_INFO, sensor_qos
                )
                self._camera_timer = self.create_timer(
                    1.0 / camera_fps, self._publish_camera_frame
                )
                self.get_logger().info(
                    f"Camera publishing enabled: {self._camera_name} "
                    f"@ {camera_fps}Hz, {camera_width}x{camera_height}"
                )

            # ── E-stop flag: when True, ignore all command callbacks ──
            self._estop_active = False
            from std_msgs.msg import Bool
            status_qos = get_qos_profile(QoSPreset.STATUS)
            self.create_subscription(
                Bool, TopicNames.ESTOP_ACTIVE,
                self._estop_status_cb, status_qos,
            )

            # ── Initial pose configuration ──
            self._initial_pose = {
                "torso": [0.0] * 6,
                "right_arm": [0.0, -0.8, 0.0, -0.6, 0.0, 0.0, 0.0],
                "left_arm": [0.0, 0.8, 0.0, -0.6, 0.0, 0.0, 0.0],
                "head": [0.0, 0.0],
                "right_gripper": 0.0,
                "left_gripper": 0.0,
            }
            # Try loading from YAML config
            self._load_initial_pose_config(project_root)

            # ── Init pose service ──
            self._init_pose_srv = self.create_service(
                Trigger, ServiceNames.INIT_POSE,
                self._init_pose_callback,
            )

            self.get_logger().info("MuJoCoROS2Bridge ready")

        def _load_initial_pose_config(self, project_root: Path) -> None:
            """Load initial_pose from config/simulation/mujoco.yaml if available."""
            config_path = project_root / "config" / "simulation" / "mujoco.yaml"
            if not config_path.exists():
                return
            try:
                import yaml
                with open(config_path) as f:
                    cfg = yaml.safe_load(f)
                ip = cfg.get("mujoco", {}).get("initial_pose", {})
                if not ip:
                    return
                if "torso" in ip:
                    self._initial_pose["torso"] = list(ip["torso"])
                if "right_arm" in ip:
                    self._initial_pose["right_arm"] = list(ip["right_arm"])
                if "left_arm" in ip:
                    self._initial_pose["left_arm"] = list(ip["left_arm"])
                if "head" in ip:
                    self._initial_pose["head"] = list(ip["head"])
                if "right_gripper" in ip:
                    self._initial_pose["right_gripper"] = float(ip["right_gripper"])
                if "left_gripper" in ip:
                    self._initial_pose["left_gripper"] = float(ip["left_gripper"])
                self.get_logger().info(f"Loaded initial pose from {config_path}")
            except Exception as e:
                self.get_logger().warning(f"Failed to load initial pose config: {e}")

        def _init_pose_callback(self, request, response) -> object:
            """Set robot to initial A-pose with slow interpolation.

            Interpolates from current position to target over ~2 seconds
            (1000 physics steps at 500Hz) for real robot safety.
            """
            try:
                ctrl = self._sim._data.ctrl
                # Capture current ctrl values as start
                start_ctrl = ctrl.copy()

                # Build target ctrl array
                target_ctrl = ctrl.copy()
                target_ctrl[CTRL_TORSO] = self._initial_pose["torso"]
                target_ctrl[CTRL_RIGHT_ARM] = self._initial_pose["right_arm"]
                target_ctrl[CTRL_LEFT_ARM] = self._initial_pose["left_arm"]
                target_ctrl[CTRL_HEAD] = self._initial_pose["head"]
                target_ctrl[CTRL_RIGHT_GRIPPER] = self._initial_pose["right_gripper"]
                target_ctrl[CTRL_LEFT_GRIPPER] = self._initial_pose["left_gripper"]

                # Interpolate over 1000 steps (~2s at 500Hz)
                n_steps = 1000
                for i in range(n_steps):
                    alpha = (i + 1) / n_steps
                    # Smooth interpolation (ease-in-out)
                    alpha = 0.5 * (1.0 - np.cos(alpha * np.pi))
                    ctrl[:] = start_ctrl + alpha * (target_ctrl - start_ctrl)
                    self._sim.step()

                response.success = True
                response.message = "Initial pose set (smooth interpolation)"
                self.get_logger().info("Robot set to initial pose (2s interpolation)")
            except Exception as e:
                response.success = False
                response.message = str(e)
                self.get_logger().error(f"Failed to set initial pose: {e}")
            return response

        def _estop_status_cb(self, msg) -> None:
            """Handle estop active/inactive from GUI."""
            was_active = self._estop_active
            self._estop_active = msg.data
            if msg.data and not was_active:
                self.get_logger().warning("E-STOP active — ignoring command topics")
                # Zero all ctrl immediately
                self._sim._data.ctrl[:] = 0.0
            elif not msg.data and was_active:
                self.get_logger().info("E-STOP released — command topics re-enabled")

        def _joint_cmd_callback(self, msg: JointState, ctrl_slice: slice) -> None:
            """Apply incoming joint positions to MuJoCo ctrl at the given indices."""
            if self._estop_active:
                return  # Ignore commands during estop
            if not msg.position:
                return
            positions = np.array(msg.position)
            n_expected = ctrl_slice.stop - ctrl_slice.start
            n = min(len(positions), n_expected)
            self._sim._data.ctrl[ctrl_slice.start:ctrl_slice.start + n] = positions[:n]

        def _base_cmd_callback(self, msg: Twist) -> None:
            """Convert Twist to differential drive wheel velocities."""
            if self._estop_active:
                return  # Ignore commands during estop
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
            if self._estop_active:
                return  # Ignore commands during estop
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

        def _publish_camera_frame(self) -> None:
            """Render head camera and publish RGB-D as ROS2 Image messages.

            Runs on the main thread via rclpy timer (single-threaded executor),
            which is the same thread that created the EGL rendering context.
            """
            frame = self._sim.get_camera_rgbd(self._camera_name)
            stamp = self.get_clock().now().to_msg()

            # RGB Image (rgb8 encoding)
            rgb_msg = Image()
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = self._camera_name
            rgb_msg.height = frame.height
            rgb_msg.width = frame.width
            rgb_msg.encoding = "rgb8"
            rgb_msg.is_bigendian = False
            rgb_msg.step = frame.width * 3
            rgb_msg.data = frame.rgb.tobytes()
            self._camera_color_pub.publish(rgb_msg)

            # Depth Image (32FC1 encoding, meters)
            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self._camera_name
            depth_msg.height = frame.height
            depth_msg.width = frame.width
            depth_msg.encoding = "32FC1"
            depth_msg.is_bigendian = False
            depth_msg.step = frame.width * 4
            depth_msg.data = frame.depth.astype(np.float32).tobytes()
            self._camera_depth_pub.publish(depth_msg)

            # CameraInfo
            info_msg = CameraInfo()
            info_msg.header.stamp = stamp
            info_msg.header.frame_id = self._camera_name
            info_msg.height = frame.height
            info_msg.width = frame.width
            info_msg.distortion_model = "plumb_bob"
            info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            K = frame.intrinsics
            info_msg.k = [
                K[0, 0], K[0, 1], K[0, 2],
                K[1, 0], K[1, 1], K[1, 2],
                K[2, 0], K[2, 1], K[2, 2],
            ]
            info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info_msg.p = [
                K[0, 0], 0.0, K[0, 2], 0.0,
                0.0, K[1, 1], K[1, 2], 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]
            self._camera_info_pub.publish(info_msg)

        def destroy_node(self):
            self._sim.shutdown()
            super().destroy_node()


def main(args=None):
    """Entry point for the MuJoCo ROS2 bridge node.

    Supports --launch-viewer and --publish-camera flags from command line.
    """
    if not _ROS2_AVAILABLE:
        raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")

    # Parse CLI flags before passing args to rclpy
    argv = args if args is not None else sys.argv
    launch_viewer = "--launch-viewer" in argv
    publish_camera = "--publish-camera" in argv
    filtered_args = [a for a in argv if a not in ("--launch-viewer", "--publish-camera")]

    rclpy.init(args=filtered_args)

    # Pass publish_camera as an override parameter
    overrides = []
    if publish_camera:
        from rclpy.parameter import Parameter
        overrides.append(Parameter("publish_camera", Parameter.Type.BOOL, True))

    node = MuJoCoROS2Bridge()

    # Override launch_viewer parameter if CLI flag was passed
    if launch_viewer and not hasattr(node, '_viewer_timer'):
        node._sim.launch_passive_viewer()
        node._viewer_timer = node.create_timer(1.0 / 60.0, node._sync_viewer)
        node.get_logger().info("Passive viewer launched via --launch-viewer flag")

    # Override publish_camera if CLI flag was passed
    if publish_camera and not node._publish_camera:
        node.get_logger().warn(
            "--publish-camera flag requires render support. "
            "Set publish_camera:=true as a parameter instead."
        )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
