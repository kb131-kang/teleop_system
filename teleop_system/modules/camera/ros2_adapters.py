"""ROS2 adapter classes for CameraController integration.

Bridges ROS2 message-based communication and the IMasterTracker / ICameraStream
interfaces that CameraController expects. These adapters allow CameraController
to work unchanged in a ROS2 context.
"""

import time

import numpy as np

from teleop_system.interfaces.camera_stream import ICameraStream, RGBDFrame
from teleop_system.interfaces.master_device import IMasterTracker, Pose6D, TrackerRole
from teleop_system.utils.logger import get_logger

logger = get_logger("camera_ros2_adapters")


class ROS2HMDAdapter(IMasterTracker):
    """Adapts ROS2 QuaternionStamped subscription data to IMasterTracker interface.

    The parent ROS2 node stores QuaternionStamped messages via update_orientation().
    CameraController reads them via get_pose().

    Position is fixed at head center; only orientation changes from HMD.
    """

    def __init__(self, timeout_sec: float = 0.5):
        self._timeout_sec = timeout_sec
        self._latest_pose: Pose6D | None = None
        self._last_update_time: float = 0.0
        self._connected = False
        self._head_position = np.array([0.0, 0.0, 1.6])

    def update_orientation(self, msg) -> None:
        """Store a new orientation from a ROS2 QuaternionStamped message.

        Args:
            msg: geometry_msgs.msg.QuaternionStamped with quaternion orientation.
        """
        q = msg.quaternion
        orientation = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)

        stamp = msg.header.stamp
        self._latest_pose = Pose6D(
            position=self._head_position.copy(),
            orientation=orientation,
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
        return TrackerRole.HEAD

    def shutdown(self) -> None:
        self._connected = False


class ROS2CameraAdapter(ICameraStream):
    """Adapts ICameraStream to publish pan/tilt commands via ROS2.

    When CameraController calls set_orientation(), this adapter publishes
    a JointState message on /slave/camera/pan_tilt_cmd.

    get_rgbd() returns empty frames -- RGB-D data comes directly from the
    MuJoCo bridge, not through this adapter. This is a control-output-only
    adapter (same pattern as ROS2HandCommandPublisher).
    """

    def __init__(self, publisher, clock):
        """Initialize the ROS2 camera adapter.

        Args:
            publisher: rclpy Publisher for sensor_msgs/JointState.
            clock: rclpy Clock for timestamping messages.
        """
        self._publisher = publisher
        self._clock = clock
        self._last_pan: float = 0.0
        self._last_tilt: float = 0.0

    def initialize(self) -> bool:
        return True

    def get_rgbd(self) -> RGBDFrame:
        return RGBDFrame()

    def set_orientation(self, pan: float, tilt: float) -> None:
        from sensor_msgs.msg import JointState

        msg = JointState()
        msg.header.stamp = self._clock.now().to_msg()
        msg.position = [float(pan), float(tilt)]
        self._publisher.publish(msg)
        self._last_pan = pan
        self._last_tilt = tilt

    def get_orientation(self) -> tuple[float, float]:
        return (self._last_pan, self._last_tilt)

    def get_intrinsics(self) -> np.ndarray:
        return np.eye(3)

    def is_connected(self) -> bool:
        return True

    def shutdown(self) -> None:
        pass
