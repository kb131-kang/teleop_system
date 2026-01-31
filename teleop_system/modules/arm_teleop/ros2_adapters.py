"""ROS2 adapter classes for ArmController integration.

Bridges ROS2 message-based communication and the IMasterTracker / ISlaveArm
interfaces that ArmController expects. These adapters allow ArmController
to work unchanged in a ROS2 context.
"""

import time

import numpy as np

from teleop_system.interfaces.master_device import IMasterTracker, Pose6D, TrackerRole
from teleop_system.interfaces.slave_robot import ArmSide, ISlaveArm
from teleop_system.interfaces.slave_robot import JointState as InternalJointState
from teleop_system.utils.logger import get_logger

logger = get_logger("ros2_adapters")


class ROS2TrackerAdapter(IMasterTracker):
    """Adapts ROS2 PoseStamped subscription data to IMasterTracker interface.

    The parent ROS2 node stores PoseStamped messages via update_pose().
    ArmController reads them via get_pose().
    """

    def __init__(self, role: TrackerRole, timeout_sec: float = 0.5):
        self._role = role
        self._timeout_sec = timeout_sec
        self._latest_pose: Pose6D | None = None
        self._last_update_time: float = 0.0
        self._connected = False

    def update_pose(self, pose_msg) -> None:
        """Store a new pose from a ROS2 PoseStamped message.

        Converts geometry_msgs/PoseStamped to internal Pose6D.
        Position and orientation are already in ROS2 convention (xyzw).

        Args:
            pose_msg: geometry_msgs.msg.PoseStamped message.
        """
        p = pose_msg.pose.position
        o = pose_msg.pose.orientation
        stamp = pose_msg.header.stamp
        self._latest_pose = Pose6D(
            position=np.array([p.x, p.y, p.z]),
            orientation=np.array([o.x, o.y, o.z, o.w]),
            timestamp=stamp.sec + stamp.nanosec * 1e-9,
            valid=True,
        )
        self._last_update_time = time.monotonic()
        self._connected = True

    def initialize(self) -> bool:
        return True

    def get_pose(self) -> Pose6D:
        if self._latest_pose is None:
            return Pose6D(valid=False)
        if time.monotonic() - self._last_update_time > self._timeout_sec:
            return Pose6D(valid=False)
        return self._latest_pose

    def is_connected(self) -> bool:
        if not self._connected:
            return False
        return (time.monotonic() - self._last_update_time) < self._timeout_sec

    def get_role(self) -> TrackerRole:
        return self._role

    def shutdown(self) -> None:
        self._connected = False


class ROS2ArmCommandPublisher(ISlaveArm):
    """Adapts ISlaveArm.send_joint_command() to publish JointState on a ROS2 topic.

    When ArmController calls send_joint_command(), this adapter publishes
    a sensor_msgs/JointState message via the injected ROS2 publisher.
    """

    def __init__(
        self,
        side: ArmSide,
        publisher,
        clock,
        joint_names: list[str] | None = None,
        n_joints: int = 7,
    ):
        """Initialize the ROS2 arm command publisher adapter.

        Args:
            side: Which arm chain (LEFT, RIGHT, TORSO).
            publisher: rclpy Publisher for sensor_msgs/JointState.
            clock: rclpy Clock for timestamping messages.
            joint_names: Optional joint name list.
            n_joints: Number of joints in this chain.
        """
        self._side = side
        self._publisher = publisher
        self._clock = clock
        self._joint_names = joint_names or [f"joint_{i}" for i in range(n_joints)]
        self._n_joints = n_joints
        self._last_command: np.ndarray | None = None

    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        """Publish joint positions as a sensor_msgs/JointState message."""
        from sensor_msgs.msg import JointState as JointStateMsg

        msg = JointStateMsg()
        msg.header.stamp = self._clock.now().to_msg()
        msg.name = self._joint_names
        msg.position = joint_positions.tolist()
        self._publisher.publish(msg)
        self._last_command = joint_positions.copy()

    def initialize(self) -> bool:
        return True

    def get_joint_state(self) -> InternalJointState:
        if self._last_command is not None:
            return InternalJointState(
                positions=self._last_command.copy(),
                names=self._joint_names,
            )
        return InternalJointState(
            positions=np.zeros(self._n_joints),
            names=self._joint_names,
        )

    def get_joint_count(self) -> int:
        return self._n_joints

    def get_side(self) -> ArmSide:
        return self._side

    def is_connected(self) -> bool:
        return True

    def shutdown(self) -> None:
        pass
