"""ROS2 adapter classes for HandController integration.

Bridges ROS2 message-based communication and the IHandInput / ISlaveHand
interfaces that HandController expects. These adapters allow HandController
to work unchanged in a ROS2 context.
"""

import time

import numpy as np

from teleop_system.interfaces.master_device import IHandInput, HandJointState
from teleop_system.interfaces.slave_robot import ISlaveHand, JointState
from teleop_system.utils.logger import get_logger

logger = get_logger("hand_ros2_adapters")


class ROS2GloveAdapter(IHandInput):
    """Adapts ROS2 JointState subscription data to IHandInput interface.

    The parent ROS2 node stores JointState messages via update_joints().
    HandController reads them via get_joint_state().
    """

    def __init__(self, side: str, timeout_sec: float = 0.5):
        self._side = side
        self._timeout_sec = timeout_sec
        self._latest_state: HandJointState | None = None
        self._last_update_time: float = 0.0
        self._connected = False

    def update_joints(self, msg) -> None:
        """Store a new joint state from a ROS2 JointState message.

        Args:
            msg: sensor_msgs.msg.JointState message with position values.
        """
        positions = np.array(msg.position, dtype=np.float64)
        n = min(len(positions), 20)
        angles = np.zeros(20)
        angles[:n] = positions[:n]

        stamp = msg.header.stamp
        self._latest_state = HandJointState(
            joint_angles=angles,
            timestamp=stamp.sec + stamp.nanosec * 1e-9,
            side=self._side,
            valid=True,
        )
        self._last_update_time = time.monotonic()
        self._connected = True

    def initialize(self) -> bool:
        return True

    def get_joint_state(self) -> HandJointState:
        if self._latest_state is None:
            return HandJointState(valid=False)
        if time.monotonic() - self._last_update_time > self._timeout_sec:
            return HandJointState(valid=False)
        return self._latest_state

    def is_connected(self) -> bool:
        if not self._connected:
            return False
        return (time.monotonic() - self._last_update_time) < self._timeout_sec

    def get_side(self) -> str:
        return self._side

    def shutdown(self) -> None:
        self._connected = False


class ROS2HandCommandPublisher(ISlaveHand):
    """Adapts ISlaveHand.send_joint_command() to publish JointState on a ROS2 topic.

    When HandController calls send_joint_command(), this adapter publishes
    a sensor_msgs/JointState message via the injected ROS2 publisher.
    """

    def __init__(self, side: str, publisher, clock, n_joints: int = 20):
        """Initialize the ROS2 hand command publisher adapter.

        Args:
            side: 'left' or 'right'.
            publisher: rclpy Publisher for sensor_msgs/JointState.
            clock: rclpy Clock for timestamping messages.
            n_joints: Number of hand joints (20 for DG-5F).
        """
        self._side = side
        self._publisher = publisher
        self._clock = clock
        self._n_joints = n_joints
        self._last_command: np.ndarray | None = None

    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        from sensor_msgs.msg import JointState as JointStateMsg

        msg = JointStateMsg()
        msg.header.stamp = self._clock.now().to_msg()
        msg.position = joint_positions.tolist()
        self._publisher.publish(msg)
        self._last_command = joint_positions.copy()

    def initialize(self) -> bool:
        return True

    def get_joint_state(self) -> JointState:
        if self._last_command is not None:
            return JointState(positions=self._last_command.copy())
        return JointState(positions=np.zeros(self._n_joints))

    def get_joint_count(self) -> int:
        return self._n_joints

    def get_side(self) -> str:
        return self._side

    def is_connected(self) -> bool:
        return True

    def shutdown(self) -> None:
        pass
