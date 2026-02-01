"""ROS2 head orientation sync helper.

Subscribes to /mujoco/joint_states, extracts head pan/tilt joint
positions, and forwards them to an ICameraStream (typically an
RGBDStreamClient) via set_orientation().

Used to bridge ROS2 head tracking to the TCP streaming reverse channel
so the streaming server's MuJoCo instance receives head orientation
commands from the HMD.
"""

from __future__ import annotations

import threading

from teleop_system.interfaces.camera_stream import ICameraStream
from teleop_system.utils.logger import get_logger

logger = get_logger("ros2_head_sync")

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False


class ROS2HeadSync:
    """Forwards head orientation from ROS2 joint_states to an ICameraStream.

    Creates a minimal ROS2 node, subscribes to the joint states topic,
    extracts head_0 (pan) and head_1 (tilt) positions, and calls
    camera.set_orientation() on each update.

    Args:
        camera: ICameraStream to forward orientation to (e.g. RGBDStreamClient).
        joint_states_topic: ROS2 topic name for joint states.
        head_joint_names: (pan_name, tilt_name) joint names in the JointState msg.
    """

    def __init__(
        self,
        camera: ICameraStream,
        joint_states_topic: str = "/mujoco/joint_states",
        head_joint_names: tuple[str, str] = ("head_0", "head_1"),
    ):
        self._camera = camera
        self._topic = joint_states_topic
        self._head_names = head_joint_names
        self._node = None
        self._spin_thread: threading.Thread | None = None
        self._running = False

    def start(self) -> bool:
        """Initialize ROS2 and start subscribing in a background thread."""
        if not _ROS2_AVAILABLE:
            logger.warning("ROS2 not available — head sync disabled")
            return False

        try:
            if not rclpy.ok():
                rclpy.init()
        except RuntimeError:
            rclpy.init()

        self._node = rclpy.create_node("head_sync_bridge")

        from teleop_system.utils.ros2_helpers import QoSPreset, get_qos_profile

        sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)

        self._node.create_subscription(
            JointState, self._topic, self._callback, sensor_qos
        )

        self._running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name="head-sync-ros2"
        )
        self._spin_thread.start()
        logger.info(f"ROS2HeadSync started: subscribing to {self._topic}")
        return True

    def _callback(self, msg) -> None:
        """Extract head pan/tilt from joint states and forward to camera."""
        pan = None
        tilt = None
        for i, name in enumerate(msg.name):
            if name == self._head_names[0] and i < len(msg.position):
                pan = float(msg.position[i])
            elif name == self._head_names[1] and i < len(msg.position):
                tilt = float(msg.position[i])
        if pan is not None and tilt is not None:
            self._camera.set_orientation(pan, tilt)

    def _spin_loop(self) -> None:
        """Background ROS2 spin loop."""
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def stop(self) -> None:
        """Stop the sync and destroy the ROS2 node."""
        self._running = False
        if self._spin_thread:
            self._spin_thread.join(timeout=2.0)
        if self._node:
            self._node.destroy_node()
            self._node = None
        logger.info("ROS2HeadSync stopped")
