"""HTC Vive Tracker driver using PyOpenVR.

Implements IMasterTracker for 6DoF pose tracking via SteamVR.
Supports serial-number-based tracker-to-role mapping and automatic
SteamVR Y-up to ROS2 Z-up coordinate conversion.
"""

from __future__ import annotations

import numpy as np

from teleop_system.interfaces.master_device import IMasterTracker, Pose6D, TrackerRole
from teleop_system.utils.logger import get_logger
from teleop_system.utils.transforms import (
    steamvr_position_to_ros2,
    steamvr_orientation_to_ros2,
)

logger = get_logger("vive_tracker")

try:
    import openvr
    _OPENVR_AVAILABLE = True
except ImportError:
    _OPENVR_AVAILABLE = False
    logger.warning("PyOpenVR not available — ViveTracker cannot be used")


_ROLE_NAME_MAP = {
    "right_hand": TrackerRole.RIGHT_HAND,
    "left_hand": TrackerRole.LEFT_HAND,
    "waist": TrackerRole.WAIST,
    "right_foot": TrackerRole.RIGHT_FOOT,
    "left_foot": TrackerRole.LEFT_FOOT,
    "head": TrackerRole.HEAD,
}


def _hmd_matrix_to_position_quat(mat) -> tuple[np.ndarray, np.ndarray]:
    """Convert a SteamVR 3x4 HmdMatrix34_t to position + quaternion (wxyz).

    Args:
        mat: openvr.HmdMatrix34_t tracking matrix.

    Returns:
        Tuple of (position (3,), quaternion_wxyz (4,)).
    """
    m = np.array([
        [mat[0][0], mat[0][1], mat[0][2], mat[0][3]],
        [mat[1][0], mat[1][1], mat[1][2], mat[1][3]],
        [mat[2][0], mat[2][1], mat[2][2], mat[2][3]],
    ])
    pos = m[:3, 3]
    rot = m[:3, :3]

    # Rotation matrix to quaternion (wxyz)
    trace = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rot[2, 1] - rot[1, 2]) * s
        y = (rot[0, 2] - rot[2, 0]) * s
        z = (rot[1, 0] - rot[0, 1]) * s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s

    quat_wxyz = np.array([w, x, y, z])
    quat_wxyz /= np.linalg.norm(quat_wxyz)
    return pos, quat_wxyz


class ViveTracker(IMasterTracker):
    """Single Vive Tracker implementing IMasterTracker.

    Reads pose data from SteamVR via a shared ViveTrackerManager.
    Automatically converts from SteamVR Y-up to ROS2 Z-up frame.
    """

    def __init__(
        self,
        role: TrackerRole,
        device_index: int = -1,
        serial: str = "",
    ):
        self._role = role
        self._device_index = device_index
        self._serial = serial
        self._connected = False

    def initialize(self) -> bool:
        if not _OPENVR_AVAILABLE:
            logger.error("PyOpenVR not installed")
            return False
        self._connected = self._device_index >= 0
        if self._connected:
            logger.info(f"ViveTracker initialized: {self._role.name} (idx={self._device_index})")
        return self._connected

    def get_pose(self) -> Pose6D:
        if not self._connected or not _OPENVR_AVAILABLE:
            return Pose6D(valid=False)

        try:
            poses = openvr.VRSystem().getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, self._device_index + 1
            )
            pose = poses[self._device_index]
            if not pose.bPoseIsValid:
                return Pose6D(valid=False)

            pos_steamvr, quat_wxyz = _hmd_matrix_to_position_quat(pose.mDeviceToAbsoluteTracking)

            # Convert SteamVR Y-up to ROS2 Z-up
            pos_ros2 = steamvr_position_to_ros2(pos_steamvr)
            quat_ros2 = steamvr_orientation_to_ros2(quat_wxyz)

            return Pose6D(
                position=pos_ros2,
                orientation=quat_ros2,
                valid=True,
            )
        except Exception as e:
            logger.error(f"ViveTracker get_pose failed: {e}")
            return Pose6D(valid=False)

    def is_connected(self) -> bool:
        return self._connected

    def get_role(self) -> TrackerRole:
        return self._role

    def shutdown(self) -> None:
        self._connected = False


class ViveTrackerManager:
    """Manages multiple Vive Trackers via a single SteamVR connection.

    Discovers connected trackers, maps them to roles (by serial number
    or auto-detect by position), and provides ViveTracker instances.
    """

    def __init__(
        self,
        tracker_mapping: dict[str, str] | None = None,
        auto_detect: bool = True,
    ):
        """Initialize tracker manager.

        Args:
            tracker_mapping: Dict of role_name -> serial_number.
                e.g. {"right_hand": "LHR-XXXXXXXX"}.
            auto_detect: If True, assign roles by tracker height.
        """
        self._tracker_mapping = tracker_mapping or {}
        self._auto_detect = auto_detect
        self._vr_system = None
        self._trackers: dict[TrackerRole, ViveTracker] = {}
        self._initialized = False

    def initialize(self) -> bool:
        if not _OPENVR_AVAILABLE:
            logger.error("PyOpenVR not installed — cannot initialize ViveTrackerManager")
            return False

        try:
            self._vr_system = openvr.init(openvr.VRApplication_Other)
            logger.info("SteamVR connected")
        except Exception as e:
            logger.error(f"Failed to connect to SteamVR: {e}")
            return False

        self._discover_trackers()
        self._initialized = True
        return True

    def _discover_trackers(self) -> None:
        """Scan SteamVR for connected trackers and assign roles."""
        serial_to_role = {}
        for role_name, serial in self._tracker_mapping.items():
            if role_name in _ROLE_NAME_MAP:
                serial_to_role[serial] = _ROLE_NAME_MAP[role_name]

        for idx in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self._vr_system.getTrackedDeviceClass(idx)
            if device_class != openvr.TrackedDeviceClass_GenericTracker:
                continue

            serial = self._vr_system.getStringTrackedDeviceProperty(
                idx, openvr.Prop_SerialNumber_String
            )

            if serial in serial_to_role:
                role = serial_to_role[serial]
                tracker = ViveTracker(role=role, device_index=idx, serial=serial)
                tracker.initialize()
                self._trackers[role] = tracker
                logger.info(f"Mapped tracker {serial} -> {role.name} (idx={idx})")

    def get_tracker(self, role: TrackerRole) -> ViveTracker | None:
        return self._trackers.get(role)

    def get_all_trackers(self) -> dict[TrackerRole, ViveTracker]:
        return dict(self._trackers)

    def shutdown(self) -> None:
        for tracker in self._trackers.values():
            tracker.shutdown()
        self._trackers.clear()
        if _OPENVR_AVAILABLE and self._initialized:
            try:
                openvr.shutdown()
            except Exception:
                pass
        self._initialized = False
        logger.info("ViveTrackerManager shutdown")
