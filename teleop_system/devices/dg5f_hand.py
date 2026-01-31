"""TESOLLO DG-5F robot hand driver via ROS2 ros2_control.

Implements ISlaveHand using the DELTO_M_ROS2 controller interface.
Sends 20-DOF joint position commands (4 joints x 5 fingers).
"""

from __future__ import annotations

import numpy as np

from teleop_system.interfaces.slave_robot import ISlaveHand, JointState
from teleop_system.utils.logger import get_logger

logger = get_logger("dg5f_hand")

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    from sensor_msgs.msg import JointState as JointStateMsg
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — DG5FHand cannot be used")


class DG5FHand(ISlaveHand):
    """TESOLLO DG-5F hand control via ROS2 ros2_control.

    Publishes joint position commands to the DELTO_M_ROS2 controller
    and subscribes to joint state feedback.
    """

    def __init__(
        self,
        side: str = "right",
        controller_name: str | None = None,
        node: object | None = None,
    ):
        """Initialize DG-5F hand driver.

        Args:
            side: 'left' or 'right'.
            controller_name: ros2_control controller name. Defaults based on side.
            node: Optional existing ROS2 node.
        """
        self._side = side
        self._controller_name = controller_name or f"{side}_hand_controller"
        self._node = node
        self._connected = False
        self._n_joints = 20
        self._last_positions = np.zeros(self._n_joints)
        self._cmd_publisher = None
        self._state_subscription = None

    def initialize(self) -> bool:
        if not _ROS2_AVAILABLE:
            logger.error("ROS2 not available — cannot initialize DG5FHand")
            return False

        try:
            if self._node is None:
                self._node = rclpy.create_node(f"dg5f_hand_{self._side}")

            cmd_topic = f"/{self._controller_name}/commands"
            state_topic = f"/{self._controller_name}/state"

            self._cmd_publisher = self._node.create_publisher(
                Float64MultiArray, cmd_topic, 10
            )
            self._state_subscription = self._node.create_subscription(
                JointStateMsg, state_topic, self._state_callback, 10
            )

            self._connected = True
            logger.info(
                f"DG5FHand initialized: {self._side}, "
                f"cmd={cmd_topic}, state={state_topic}"
            )
            return True
        except Exception as e:
            logger.error(f"DG5FHand init failed: {e}")
            return False

    def _state_callback(self, msg: object) -> None:
        positions = np.array(msg.position)
        n = min(len(positions), self._n_joints)
        self._last_positions[:n] = positions[:n]

    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        if not self._connected or self._cmd_publisher is None:
            return

        try:
            msg = Float64MultiArray()
            n = min(len(joint_positions), self._n_joints)
            msg.data = joint_positions[:n].tolist()
            self._cmd_publisher.publish(msg)
        except Exception as e:
            logger.error(f"DG5FHand send_joint_command failed: {e}")

    def get_joint_state(self) -> JointState:
        return JointState(positions=self._last_positions.copy())

    def get_joint_count(self) -> int:
        return self._n_joints

    def get_side(self) -> str:
        return self._side

    def is_connected(self) -> bool:
        return self._connected

    def shutdown(self) -> None:
        self._connected = False
        if self._node is not None:
            if self._cmd_publisher is not None:
                self._node.destroy_publisher(self._cmd_publisher)
            if self._state_subscription is not None:
                self._node.destroy_subscription(self._state_subscription)
        logger.info(f"DG5FHand shutdown: {self._side}")
