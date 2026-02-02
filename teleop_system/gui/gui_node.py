"""ROS2 node that integrates ControlPanel GUI with ROS2 topics/services.

Threading model:
- Dear PyGui runs on the **main thread** (required, not thread-safe)
- ROS2 executor spins in a **background daemon thread**
- Shared data in ControlPanel buffers protected by threading.Lock

Entry point: gui_control_panel
"""

from __future__ import annotations

import json
import threading
import time

import numpy as np

from teleop_system.gui.control_panel import ControlPanel
from teleop_system.utils.logger import get_logger

logger = get_logger("gui_node")


def main():
    """Entry point for the GUI control panel ROS2 node."""
    try:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped, QuaternionStamped, Twist
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger
    except ImportError:
        logger.error("ROS2 (rclpy) is required for GUINode")
        return

    from teleop_system.utils.ros2_helpers import (
        TopicNames,
        ServiceNames,
        QoSPreset,
        get_qos_profile,
    )

    class GUINode(Node):
        """ROS2 node bridging ControlPanel GUI with ROS2 topics."""

        def __init__(self, node_name: str = "gui_control_panel"):
            super().__init__(node_name)

            # Declare parameters
            self.declare_parameter("viewer_host", "localhost")
            self.declare_parameter("viewer_port", 9876)
            self.declare_parameter("font_scale", 0.0)  # 0.0 = auto-detect

            viewer_host = self.get_parameter("viewer_host").get_parameter_value().string_value
            viewer_port = self.get_parameter("viewer_port").get_parameter_value().integer_value
            font_scale_val = self.get_parameter("font_scale").get_parameter_value().double_value
            font_scale = font_scale_val if font_scale_val > 0 else None

            # Create control panel
            self._panel = ControlPanel(font_scale=font_scale)
            self._panel.set_tcp_viewer_config(viewer_host, viewer_port)

            # Register modules
            self._panel.add_module("Arm Teleop")
            self._panel.add_module("Locomotion")
            self._panel.add_module("Hand Teleop")
            self._panel.add_module("Camera")

            # Set callbacks
            self._panel.set_calibrate_callback(self._request_calibration)
            self._panel.set_emergency_stop_callback(self._toggle_emergency_stop)
            self._panel.set_start_playback_callback(self._request_start_playback)
            self._panel.set_init_pose_callback(self._request_init_pose)
            self._panel.set_soft_stop_callback(self._soft_stop)
            self._panel.set_resume_callback(self._resume)

            # E-stop timer (created on demand)
            self._estop_timer = None
            # Soft-stop timer (ramp-down then hold)
            self._soft_stop_timer = None
            self._soft_stop_ticks = 0  # counts 50Hz ticks for 0.5s ramp

            # ROS2 subscriptions
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)
            status_qos = get_qos_profile(QoSPreset.STATUS)
            cmd_qos = get_qos_profile(QoSPreset.COMMAND)

            # Tracker pose subscriptions (including HEAD for BVH replay)
            tracker_topics = {
                "right_hand": TopicNames.TRACKER_RIGHT_HAND,
                "left_hand": TopicNames.TRACKER_LEFT_HAND,
                "waist": TopicNames.TRACKER_WAIST,
                "right_foot": TopicNames.TRACKER_RIGHT_FOOT,
                "left_foot": TopicNames.TRACKER_LEFT_FOOT,
                "head": TopicNames.TRACKER_HEAD,
            }

            for role_name, topic in tracker_topics.items():
                self.create_subscription(
                    PoseStamped, topic,
                    lambda msg, r=role_name: self._tracker_cb(r, msg),
                    sensor_qos,
                )

            # HMD subscription (fallback for head position when no tracker data)
            self.create_subscription(
                QuaternionStamped, TopicNames.HMD_ORIENTATION,
                self._hmd_cb, sensor_qos,
            )

            # Joint command subscriptions
            self.create_subscription(
                JointState, TopicNames.ARM_LEFT_CMD,
                lambda msg: self._joint_cmd_cb("left_arm", msg),
                cmd_qos,
            )
            self.create_subscription(
                JointState, TopicNames.ARM_RIGHT_CMD,
                lambda msg: self._joint_cmd_cb("right_arm", msg),
                cmd_qos,
            )
            self.create_subscription(
                JointState, TopicNames.TORSO_CMD,
                lambda msg: self._joint_cmd_cb("torso", msg),
                cmd_qos,
            )

            # Joint state feedback (MuJoCo)
            self.create_subscription(
                JointState, TopicNames.MUJOCO_JOINT_STATES,
                self._joint_states_cb, sensor_qos,
            )

            # Hand joint input subscriptions
            self.create_subscription(
                JointState, TopicNames.HAND_LEFT_JOINTS,
                lambda msg: self._hand_input_cb("left", msg),
                sensor_qos,
            )
            self.create_subscription(
                JointState, TopicNames.HAND_RIGHT_JOINTS,
                lambda msg: self._hand_input_cb("right", msg),
                sensor_qos,
            )

            # Hand command output subscriptions (for skeleton overlay)
            self.create_subscription(
                JointState, TopicNames.HAND_LEFT_CMD,
                lambda msg: self._hand_cmd_cb("left", msg),
                cmd_qos,
            )
            self.create_subscription(
                JointState, TopicNames.HAND_RIGHT_CMD,
                lambda msg: self._hand_cmd_cb("right", msg),
                cmd_qos,
            )

            # Locomotion output (for module activity monitoring)
            self.create_subscription(
                Twist, TopicNames.BASE_CMD_VEL,
                self._base_vel_cb, cmd_qos,
            )

            # Calibration state subscription
            self.create_subscription(
                String, TopicNames.CALIBRATION_STATE,
                self._cal_state_cb, status_qos,
            )

            # Calibration offsets subscription
            self.create_subscription(
                String, TopicNames.CALIBRATION_OFFSETS,
                self._cal_offsets_cb, status_qos,
            )

            # Emergency stop publishers (all command topics)
            self._estop_arm_left = self.create_publisher(
                JointState, TopicNames.ARM_LEFT_CMD, cmd_qos,
            )
            self._estop_arm_right = self.create_publisher(
                JointState, TopicNames.ARM_RIGHT_CMD, cmd_qos,
            )
            self._estop_base = self.create_publisher(
                Twist, TopicNames.BASE_CMD_VEL, cmd_qos,
            )
            self._estop_hand_left = self.create_publisher(
                JointState, TopicNames.HAND_LEFT_CMD, cmd_qos,
            )
            self._estop_hand_right = self.create_publisher(
                JointState, TopicNames.HAND_RIGHT_CMD, cmd_qos,
            )

            # E-stop status publisher (tells MuJoCo bridge to ignore commands)
            self._estop_status_pub = self.create_publisher(
                Bool, TopicNames.ESTOP_ACTIVE, status_qos,
            )

            # Calibration service client
            self._cal_client = self.create_client(
                Trigger, ServiceNames.CALIBRATE,
            )

            # Playback state subscription
            self.create_subscription(
                String, TopicNames.PLAYBACK_STATE,
                self._playback_state_cb, status_qos,
            )

            # Start playback service client
            self._playback_client = self.create_client(
                Trigger, ServiceNames.START_PLAYBACK,
            )

            # Init pose service client
            self._init_pose_client = self.create_client(
                Trigger, ServiceNames.INIT_POSE,
            )

            # Track start time for relative timestamps in plots
            self._start_time = time.monotonic()

            self.get_logger().info("GUI node ready")

        @property
        def panel(self) -> ControlPanel:
            return self._panel

        # ----------------------------------------------------------
        # ROS2 callbacks (run in executor thread)
        # ----------------------------------------------------------

        def _mark_module_active(self, module_name: str) -> None:
            """Update last_activity timestamp for a module (must hold lock)."""
            if module_name in self._panel._modules:
                self._panel._modules[module_name].last_activity = time.monotonic()

        def _tracker_cb(self, role_name: str, msg: PoseStamped) -> None:
            pos = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ])
            with self._panel.lock:
                self._panel.tracker_data.positions[role_name] = pos

        def _hmd_cb(self, msg: QuaternionStamped) -> None:
            # HMD only provides orientation, not position.
            # Only set fallback head position if no tracker data for head.
            with self._panel.lock:
                if "head" not in self._panel.tracker_data.positions:
                    self._panel.tracker_data.positions["head"] = np.array([
                        0.0, 0.0, 1.55,
                    ])
                self._mark_module_active("Camera")

        def _joint_cmd_cb(self, group: str, msg: JointState) -> None:
            t = time.monotonic() - self._start_time
            positions = list(msg.position) if msg.position else []
            if not positions:
                return

            with self._panel.lock:
                jd = self._panel.joint_data
                if group == "left_arm":
                    jd.left_arm_cmd.append((t, positions))
                    self._mark_module_active("Arm Teleop")
                elif group == "right_arm":
                    jd.right_arm_cmd.append((t, positions))
                    self._mark_module_active("Arm Teleop")
                elif group == "torso":
                    jd.torso_cmd.append((t, positions))
                    self._mark_module_active("Arm Teleop")

        def _hand_input_cb(self, side: str, msg: JointState) -> None:
            positions = np.array(msg.position) if msg.position else None
            if positions is None or len(positions) == 0:
                return

            with self._panel.lock:
                hd = self._panel.hand_data
                if side == "left":
                    hd.left_joints = positions
                else:
                    hd.right_joints = positions
                self._mark_module_active("Hand Teleop")

        def _hand_cmd_cb(self, side: str, msg: JointState) -> None:
            positions = np.array(msg.position) if msg.position else None
            if positions is None or len(positions) == 0:
                return

            with self._panel.lock:
                hd = self._panel.hand_data
                if side == "left":
                    hd.left_cmd = positions
                else:
                    hd.right_cmd = positions

        def _joint_states_cb(self, msg: JointState) -> None:
            t = time.monotonic() - self._start_time
            positions = list(msg.position) if msg.position else []
            if not positions:
                return
            with self._panel.lock:
                self._panel.joint_data.joint_states.append((t, positions))

        def _base_vel_cb(self, msg: Twist) -> None:
            with self._panel.lock:
                self._mark_module_active("Locomotion")

        def _cal_state_cb(self, msg: String) -> None:
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return

            with self._panel.lock:
                td = self._panel.tracker_data
                td.calibration_state = data.get("state", "IDLE")
                td.calibration_progress = data.get("progress", 0.0)
                td.calibration_countdown = data.get("countdown", 0.0)
                td.calibration_error = data.get("error", "")

        def _cal_offsets_cb(self, msg: String) -> None:
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return

            with self._panel.lock:
                td = self._panel.tracker_data
                for name, offset_list in data.items():
                    td.offsets[name] = np.array(offset_list)

        def _playback_state_cb(self, msg: String) -> None:
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return

            with self._panel.lock:
                self._panel._playback_state = data.get("state", "UNKNOWN")

        # ----------------------------------------------------------
        # Action callbacks (called from GUI main thread)
        # ----------------------------------------------------------

        def _request_start_playback(self) -> None:
            if not self._playback_client.service_is_ready():
                self.get_logger().warning(
                    "Start playback service not available "
                    "(BVH replay publisher may not be running)"
                )
                return

            request = Trigger.Request()
            future = self._playback_client.call_async(request)
            future.add_done_callback(self._playback_response_cb)
            self.get_logger().info("Start playback request sent")

        def _playback_response_cb(self, future) -> None:
            try:
                result = future.result()
                if result.success:
                    self.get_logger().info(f"Playback: {result.message}")
                else:
                    self.get_logger().warning(
                        f"Start playback failed: {result.message}"
                    )
            except Exception as e:
                self.get_logger().error(f"Playback service error: {e}")

        def _request_calibration(self) -> None:
            if not self._cal_client.service_is_ready():
                self.get_logger().warning(
                    "Calibration service not available"
                )
                return

            request = Trigger.Request()
            future = self._cal_client.call_async(request)
            future.add_done_callback(self._cal_response_cb)
            self.get_logger().info("Calibration request sent")

        def _cal_response_cb(self, future) -> None:
            try:
                result = future.result()
                if result.success:
                    self.get_logger().info(f"Calibration: {result.message}")
                else:
                    self.get_logger().warning(
                        f"Calibration failed: {result.message}"
                    )
            except Exception as e:
                self.get_logger().error(f"Calibration service error: {e}")

        def _request_init_pose(self) -> None:
            """Call /teleop/init_pose service to set robot to initial A-pose."""
            if not self._init_pose_client.service_is_ready():
                self.get_logger().warning(
                    "Init pose service not available "
                    "(MuJoCo bridge may not be running)"
                )
                return

            request = Trigger.Request()
            future = self._init_pose_client.call_async(request)
            future.add_done_callback(self._init_pose_response_cb)
            self.get_logger().info("Init pose request sent")

        def _init_pose_response_cb(self, future) -> None:
            try:
                result = future.result()
                if result.success:
                    self.get_logger().info(f"Init pose: {result.message}")
                else:
                    self.get_logger().warning(
                        f"Init pose failed: {result.message}"
                    )
            except Exception as e:
                self.get_logger().error(f"Init pose service error: {e}")

        def _soft_stop(self) -> None:
            """Gradually ramp commands to zero over 0.5s, then hold."""
            self.get_logger().warning("SOFT STOP — ramping to zero")
            self._soft_stop_ticks = 0
            # Tell bridge to ignore normal commands
            self._publish_estop_status(True)
            # Publish zeros immediately
            self._publish_estop_zeros()
            # Create timer for continuous zero publishing at 50Hz
            if self._soft_stop_timer is None:
                self._soft_stop_timer = self.create_timer(
                    0.02, self._soft_stop_tick,
                )

        def _soft_stop_tick(self) -> None:
            """50Hz tick during soft stop: publish zero commands."""
            self._soft_stop_ticks += 1
            self._publish_estop_zeros()
            # After 0.5s ramp (25 ticks at 50Hz), keep holding zeros
            # Timer continues until resume is called

        def _resume(self) -> None:
            """Cancel soft stop, allow normal command flow."""
            self.get_logger().info("Resuming from soft stop")
            if self._soft_stop_timer is not None:
                self._soft_stop_timer.cancel()
                self.destroy_timer(self._soft_stop_timer)
                self._soft_stop_timer = None
            self._soft_stop_ticks = 0
            # Tell bridge to re-enable normal commands
            self._publish_estop_status(False)

        def _toggle_emergency_stop(self, active: bool) -> None:
            """Toggle continuous zero-command publishing on all output topics."""
            if active:
                self.get_logger().warning("EMERGENCY STOP ACTIVATED")
                # Tell bridge to ignore normal commands
                self._publish_estop_status(True)
                # Publish zeros immediately
                self._publish_estop_zeros()
                # Create timer for continuous zero publishing at 50Hz
                if self._estop_timer is None:
                    self._estop_timer = self.create_timer(
                        0.02, self._publish_estop_zeros,
                    )
            else:
                self.get_logger().info("Emergency stop released")
                if self._estop_timer is not None:
                    self._estop_timer.cancel()
                    self.destroy_timer(self._estop_timer)
                    self._estop_timer = None
                # Tell bridge to re-enable normal commands
                self._publish_estop_status(False)

        def _publish_estop_status(self, active: bool) -> None:
            """Publish estop active/inactive to MuJoCo bridge."""
            msg = Bool()
            msg.data = active
            self._estop_status_pub.publish(msg)

        def _publish_estop_zeros(self) -> None:
            """Publish zero commands to all output topics."""
            zero_arm = JointState()
            zero_arm.position = [0.0] * 7
            self._estop_arm_left.publish(zero_arm)
            self._estop_arm_right.publish(zero_arm)

            zero_hand = JointState()
            zero_hand.position = [0.0] * 20
            self._estop_hand_left.publish(zero_hand)
            self._estop_hand_right.publish(zero_hand)

            zero_vel = Twist()
            self._estop_base.publish(zero_vel)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    rclpy.init()
    node = GUINode()

    # Spin ROS2 in background thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Dear PyGui on main thread
    if not node.panel.setup():
        logger.error(
            "Failed to initialize GUI. "
            "Install Dear PyGui: pip install dearpygui>=2.1.1"
        )
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        while node.panel.render_frame():
            pass
    except KeyboardInterrupt:
        pass
    finally:
        node.panel.shutdown()
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
