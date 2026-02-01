"""ROS2 RGB-D subscriber implementing ICameraStream.

Subscribes to standard ROS2 camera topics (sensor_msgs/Image, CameraInfo)
and provides RGBDFrame data. Also subscribes to joint states for head
orientation tracking. Designed to consume output from the MuJoCo bridge's
camera publisher or any ROS2 camera driver.

Pattern reference: teleop_system/devices/realsense_camera.py
"""

from __future__ import annotations

import numpy as np

from teleop_system.interfaces.camera_stream import ICameraStream, RGBDFrame
from teleop_system.utils.logger import get_logger

logger = get_logger("ros2_rgbd_subscriber")

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo, JointState

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False


class ROS2RGBDSubscriber(ICameraStream):
    """Subscribes to ROS2 RGB-D topics and provides ICameraStream interface.

    Subscribes to:
        color_topic (sensor_msgs/Image, rgb8 or bgr8)
        depth_topic (sensor_msgs/Image, 32FC1 or 16UC1)
        info_topic  (sensor_msgs/CameraInfo)
        joint_states_topic (sensor_msgs/JointState) — for head orientation

    Publishes (optional):
        pan_tilt_topic (sensor_msgs/JointState) — when set_orientation() called

    Args:
        color_topic: RGB image topic name.
        depth_topic: Depth image topic name.
        info_topic: Camera info topic name.
        joint_states_topic: Joint states topic for head orientation tracking.
        pan_tilt_topic: Topic to publish pan/tilt commands to.
        head_joint_names: Names of head joints in joint_states.
        node: Existing rclpy Node, or None to create one.
    """

    def __init__(
        self,
        color_topic: str = "/slave/camera/color/image_raw",
        depth_topic: str = "/slave/camera/depth/image_raw",
        info_topic: str = "/slave/camera/camera_info",
        joint_states_topic: str = "/mujoco/joint_states",
        pan_tilt_topic: str = "/slave/camera/pan_tilt_cmd",
        head_joint_names: tuple[str, str] = ("head_0", "head_1"),
        node: object | None = None,
    ):
        self._color_topic = color_topic
        self._depth_topic = depth_topic
        self._info_topic = info_topic
        self._joint_states_topic = joint_states_topic
        self._pan_tilt_topic = pan_tilt_topic
        self._head_joint_names = head_joint_names
        self._ext_node = node
        self._node = None
        self._connected = False

        self._latest_rgb: np.ndarray | None = None
        self._latest_depth: np.ndarray | None = None
        self._intrinsics = np.eye(3)
        self._width = 640
        self._height = 480

        self._pan = 0.0
        self._tilt = 0.0
        self._pan_tilt_pub = None

    def initialize(self) -> bool:
        if not _ROS2_AVAILABLE:
            logger.error("ROS2 not available")
            return False

        try:
            from teleop_system.utils.ros2_helpers import (
                QoSPreset,
                get_qos_profile,
            )

            if self._ext_node is not None:
                self._node = self._ext_node
            else:
                self._node = rclpy.create_node("ros2_rgbd_subscriber")

            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)
            cmd_qos = get_qos_profile(QoSPreset.COMMAND)

            self._node.create_subscription(
                Image, self._color_topic, self._rgb_callback, sensor_qos
            )
            self._node.create_subscription(
                Image, self._depth_topic, self._depth_callback, sensor_qos
            )
            self._node.create_subscription(
                CameraInfo, self._info_topic, self._info_callback, sensor_qos
            )

            if self._joint_states_topic:
                self._node.create_subscription(
                    JointState,
                    self._joint_states_topic,
                    self._joint_states_callback,
                    sensor_qos,
                )

            if self._pan_tilt_topic:
                self._pan_tilt_pub = self._node.create_publisher(
                    JointState, self._pan_tilt_topic, cmd_qos
                )

            self._connected = True
            logger.info(
                f"ROS2RGBDSubscriber initialized: "
                f"color={self._color_topic}, depth={self._depth_topic}"
            )
            return True
        except Exception as e:
            logger.error(f"ROS2RGBDSubscriber init failed: {e}")
            return False

    def _rgb_callback(self, msg) -> None:
        """Process incoming RGB image."""
        h, w = msg.height, msg.width
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if msg.encoding == "rgb8":
            self._latest_rgb = data.reshape(h, w, 3).copy()
        elif msg.encoding == "bgr8":
            self._latest_rgb = data.reshape(h, w, 3)[:, :, ::-1].copy()
        else:
            self._latest_rgb = data.reshape(h, w, -1)[:, :, :3].copy()

        self._width = w
        self._height = h

    def _depth_callback(self, msg) -> None:
        """Process incoming depth image."""
        h, w = msg.height, msg.width
        if msg.encoding == "16UC1":
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            self._latest_depth = raw.astype(np.float32) * 0.001
        elif msg.encoding == "32FC1":
            self._latest_depth = (
                np.frombuffer(msg.data, dtype=np.float32).reshape(h, w).copy()
            )

    def _info_callback(self, msg) -> None:
        """Extract intrinsics from CameraInfo."""
        self._intrinsics = np.array(msg.k).reshape(3, 3)
        self._width = msg.width
        self._height = msg.height

    def _joint_states_callback(self, msg) -> None:
        """Extract head pan/tilt from joint states."""
        for i, name in enumerate(msg.name):
            if name == self._head_joint_names[0] and i < len(msg.position):
                self._pan = float(msg.position[i])
            elif name == self._head_joint_names[1] and i < len(msg.position):
                self._tilt = float(msg.position[i])

    def get_rgbd(self) -> RGBDFrame:
        if self._latest_rgb is None or self._latest_depth is None:
            return RGBDFrame()
        return RGBDFrame(
            rgb=self._latest_rgb.copy(),
            depth=self._latest_depth.copy(),
            intrinsics=self._intrinsics.copy(),
            width=self._width,
            height=self._height,
        )

    def set_orientation(self, pan: float, tilt: float) -> None:
        """Send pan/tilt command via ROS2 and update local state."""
        self._pan = pan
        self._tilt = tilt
        if self._pan_tilt_pub is not None:
            msg = JointState()
            msg.position = [float(pan), float(tilt)]
            self._pan_tilt_pub.publish(msg)

    def get_orientation(self) -> tuple[float, float]:
        return self._pan, self._tilt

    def get_intrinsics(self) -> np.ndarray:
        return self._intrinsics.copy()

    def is_connected(self) -> bool:
        return self._connected and self._latest_rgb is not None

    def shutdown(self) -> None:
        self._connected = False
        logger.info("ROS2RGBDSubscriber shutdown")
