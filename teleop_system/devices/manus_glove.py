"""Manus Glove driver via ROS2 topics.

Implements IHandInput by subscribing to MANUS SDK ROS2 ergonomics topics.
Parses 20-DOF finger joint angles (4 joints x 5 fingers).
"""

from __future__ import annotations

import numpy as np

from teleop_system.interfaces.master_device import IHandInput, HandJointState
from teleop_system.utils.logger import get_logger

logger = get_logger("manus_glove")

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — ManusGlove cannot be used")


class ManusGlove(IHandInput):
    """Manus Glove input device via ROS2 topics.

    Subscribes to the MANUS SDK ROS2 package topic which publishes
    ergonomics data as sensor_msgs/JointState with 20 joint positions
    (4 per finger: MCP_spread, MCP_flex, PIP, DIP).
    """

    def __init__(
        self,
        side: str = "right",
        topic: str | None = None,
        node: object | None = None,
    ):
        """Initialize Manus Glove driver.

        Args:
            side: 'left' or 'right'.
            topic: ROS2 topic to subscribe. Defaults based on side.
            node: Optional existing ROS2 node for subscription.
        """
        self._side = side
        self._topic = topic or (
            "/manus_glove_0" if side == "left" else "/manus_glove_1"
        )
        self._node = node
        self._connected = False
        self._latest_state: HandJointState = HandJointState(valid=False)
        self._subscription = None

    def initialize(self) -> bool:
        if not _ROS2_AVAILABLE:
            logger.error("ROS2 not available — cannot initialize ManusGlove")
            return False

        try:
            if self._node is None:
                self._node = rclpy.create_node(f"manus_glove_{self._side}")

            self._subscription = self._node.create_subscription(
                JointState,
                self._topic,
                self._glove_callback,
                10,
            )
            self._connected = True
            logger.info(f"ManusGlove initialized: {self._side} on {self._topic}")
            return True
        except Exception as e:
            logger.error(f"ManusGlove init failed: {e}")
            return False

    def _glove_callback(self, msg: object) -> None:
        """Process incoming glove JointState message."""
        positions = np.array(msg.position, dtype=np.float64)
        n = min(len(positions), 20)
        angles = np.zeros(20)
        angles[:n] = positions[:n]

        self._latest_state = HandJointState(
            joint_angles=angles,
            timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            side=self._side,
            valid=True,
        )

    def get_joint_state(self) -> HandJointState:
        if not self._connected:
            return HandJointState(valid=False)
        return self._latest_state

    def is_connected(self) -> bool:
        return self._connected

    def get_side(self) -> str:
        return self._side

    def shutdown(self) -> None:
        self._connected = False
        if self._subscription is not None and self._node is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        logger.info(f"ManusGlove shutdown: {self._side}")
