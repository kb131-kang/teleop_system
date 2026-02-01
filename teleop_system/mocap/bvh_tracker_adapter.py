"""BVH motion capture tracker adapter implementing IMasterTracker.

Provides time-indexed playback of BVH motion data through the standard
tracker interface, allowing BVH recordings to replace live Vive Trackers
or synthetic data in the teleoperation pipeline.
"""

import time

import numpy as np

from teleop_system.interfaces.master_device import IMasterTracker, Pose6D, TrackerRole
from teleop_system.mocap.skeleton_mapper import MappedMotion
from teleop_system.utils.logger import get_logger

logger = get_logger("bvh_tracker_adapter")


class BVHTrackerAdapter(IMasterTracker):
    """IMasterTracker implementation that replays BVH motion data.

    Time-indexed playback: get_pose() returns the Pose6D corresponding
    to the current elapsed time since initialize() was called.
    """

    def __init__(
        self,
        role: TrackerRole,
        mapped_motion: MappedMotion,
        playback_speed: float = 1.0,
        loop: bool = True,
    ):
        """Initialize BVH tracker adapter.

        Args:
            role: Body part this tracker represents.
            mapped_motion: Pre-mapped BVH motion data from SkeletonMapper.
            playback_speed: Playback speed multiplier (1.0 = real-time).
            loop: Whether to loop when reaching end of recording.
        """
        self._role = role
        self._motion = mapped_motion
        self._playback_speed = playback_speed
        self._loop = loop
        self._connected = False
        self._start_time = 0.0
        self._paused = False
        self._pause_time = 0.0

        # Validate that the role has data
        self._has_data = role in mapped_motion.roles

        if not self._has_data:
            logger.warning(
                f"No BVH data available for role {role.name}. "
                "Adapter will return invalid poses."
            )

    def initialize(self) -> bool:
        self._start_time = time.monotonic()
        self._connected = True
        logger.info(
            f"BVHTrackerAdapter initialized: {self._role.name}, "
            f"{self._motion.frame_count} frames, "
            f"speed={self._playback_speed}x, loop={self._loop}"
        )
        return True

    def get_pose(self) -> Pose6D:
        if not self._connected or not self._has_data:
            return Pose6D(valid=False)

        # Compute elapsed time
        if self._paused:
            elapsed = self._pause_time - self._start_time
        else:
            elapsed = time.monotonic() - self._start_time

        elapsed *= self._playback_speed

        # Compute frame index
        total_duration = self._motion.frame_count * self._motion.frame_time
        if total_duration <= 0:
            return Pose6D(valid=False)

        if self._loop:
            elapsed = elapsed % total_duration
        elif elapsed >= total_duration:
            return Pose6D(valid=False)

        frame_idx = int(elapsed / self._motion.frame_time)
        frame_idx = min(frame_idx, self._motion.frame_count - 1)

        # Get pose from mapped motion
        frame_data = self._motion.frames[frame_idx]
        if self._role in frame_data.poses:
            pose = frame_data.poses[self._role]
            return Pose6D(
                position=pose.position.copy(),
                orientation=pose.orientation.copy(),
                timestamp=elapsed,
                valid=True,
            )

        return Pose6D(valid=False)

    def is_connected(self) -> bool:
        return self._connected

    def get_role(self) -> TrackerRole:
        return self._role

    def shutdown(self) -> None:
        self._connected = False
        logger.info(f"BVHTrackerAdapter shutdown: {self._role.name}")

    def pause(self) -> None:
        """Pause playback at current position."""
        if not self._paused:
            self._pause_time = time.monotonic()
            self._paused = True

    def resume(self) -> None:
        """Resume playback from paused position."""
        if self._paused:
            pause_duration = time.monotonic() - self._pause_time
            self._start_time += pause_duration
            self._paused = False

    def get_frame_index(self) -> int:
        """Get the current frame index."""
        if not self._connected:
            return 0
        if self._paused:
            elapsed = self._pause_time - self._start_time
        else:
            elapsed = time.monotonic() - self._start_time
        elapsed *= self._playback_speed
        total_duration = self._motion.frame_count * self._motion.frame_time
        if total_duration <= 0:
            return 0
        if self._loop:
            elapsed = elapsed % total_duration
        frame_idx = int(elapsed / self._motion.frame_time)
        return min(frame_idx, self._motion.frame_count - 1)

    def get_progress(self) -> float:
        """Get playback progress as a fraction (0.0 to 1.0)."""
        if self._motion.frame_count <= 0:
            return 0.0
        return self.get_frame_index() / max(1, self._motion.frame_count - 1)
