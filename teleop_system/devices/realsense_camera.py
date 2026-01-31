"""Intel RealSense camera driver via ROS2 topics.

Implements ICameraStream by subscribing to standard ROS2 camera topics
(rgb + depth_registered) and providing pan-tilt control for the
camera mount servo.
"""

from __future__ import annotations

import numpy as np

from teleop_system.interfaces.camera_stream import ICameraStream, RGBDFrame
from teleop_system.utils.logger import get_logger

logger = get_logger("realsense_camera")

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Float64MultiArray
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — RealSenseCamera cannot be used")


class RealSenseCamera(ICameraStream):
    """RealSense RGB-D camera via ROS2.

    Subscribes to standard camera topics and provides RGBDFrame data.
    Supports pan-tilt control for the camera mount.
    """

    def __init__(
        self,
        rgb_topic: str = "/camera/color/image_raw",
        depth_topic: str = "/camera/aligned_depth_to_color/image_raw",
        info_topic: str = "/camera/color/camera_info",
        pan_tilt_topic: str = "/slave/camera/pan_tilt_cmd",
        node: object | None = None,
    ):
        self._rgb_topic = rgb_topic
        self._depth_topic = depth_topic
        self._info_topic = info_topic
        self._pan_tilt_topic = pan_tilt_topic
        self._node = node
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
            logger.error("ROS2 not available — cannot initialize RealSenseCamera")
            return False

        try:
            if self._node is None:
                self._node = rclpy.create_node("realsense_camera")

            self._node.create_subscription(
                Image, self._rgb_topic, self._rgb_callback, 10
            )
            self._node.create_subscription(
                Image, self._depth_topic, self._depth_callback, 10
            )
            self._node.create_subscription(
                CameraInfo, self._info_topic, self._info_callback, 10
            )
            self._pan_tilt_pub = self._node.create_publisher(
                Float64MultiArray, self._pan_tilt_topic, 10
            )

            self._connected = True
            logger.info("RealSenseCamera initialized")
            return True
        except Exception as e:
            logger.error(f"RealSenseCamera init failed: {e}")
            return False

    def _rgb_callback(self, msg: object) -> None:
        h, w = msg.height, msg.width
        encoding = msg.encoding
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if encoding == "rgb8":
            self._latest_rgb = data.reshape(h, w, 3)
        elif encoding == "bgr8":
            self._latest_rgb = data.reshape(h, w, 3)[:, :, ::-1].copy()
        else:
            self._latest_rgb = data.reshape(h, w, -1)[:, :, :3]

        self._width = w
        self._height = h

    def _depth_callback(self, msg: object) -> None:
        h, w = msg.height, msg.width
        if msg.encoding == "16UC1":
            depth_raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            self._latest_depth = depth_raw.astype(np.float32) * 0.001  # mm to meters
        elif msg.encoding == "32FC1":
            self._latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)

    def _info_callback(self, msg: object) -> None:
        K = np.array(msg.k).reshape(3, 3)
        self._intrinsics = K
        self._width = msg.width
        self._height = msg.height

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
        self._pan = pan
        self._tilt = tilt
        if self._pan_tilt_pub is not None:
            msg = Float64MultiArray()
            msg.data = [pan, tilt]
            self._pan_tilt_pub.publish(msg)

    def get_orientation(self) -> tuple[float, float]:
        return self._pan, self._tilt

    def get_intrinsics(self) -> np.ndarray:
        return self._intrinsics.copy()

    def is_connected(self) -> bool:
        return self._connected

    def shutdown(self) -> None:
        self._connected = False
        logger.info("RealSenseCamera shutdown")
