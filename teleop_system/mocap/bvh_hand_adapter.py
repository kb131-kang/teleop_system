"""BVH motion capture hand adapter implementing IHandInput.

Since CMU BVH data has minimal hand articulation (only RightHandIndex1,
LeftHandIndex1, RThumb, LThumb), this adapter generates approximate
finger motion from the wrist orientation changes. The grip amount is
derived from the angular velocity of the hand joint.
"""

import time

import numpy as np

from teleop_system.interfaces.master_device import IHandInput, HandJointState
from teleop_system.mocap.skeleton_mapper import MappedMotion
from teleop_system.interfaces.master_device import TrackerRole
from teleop_system.utils.logger import get_logger

logger = get_logger("bvh_hand_adapter")


class BVHHandAdapter(IHandInput):
    """IHandInput implementation that derives hand motion from BVH data.

    Since most BVH datasets lack detailed finger articulation, this adapter
    generates plausible finger motion by:
    1. Measuring wrist angular velocity from the hand joint orientation
    2. Mapping higher angular velocity to more closed grip (activity-based)
    3. Applying a smoothed grip-to-joint-angle mapping for 20 joints

    This provides a reasonable approximation for testing the hand retargeting
    pipeline even without real glove data.
    """

    def __init__(
        self,
        side: str,
        mapped_motion: MappedMotion,
        playback_speed: float = 1.0,
        loop: bool = True,
        grip_sensitivity: float = 2.0,
        max_angle: float = 1.4,
        smoothing: float = 0.3,
    ):
        """Initialize BVH hand adapter.

        Args:
            side: 'left' or 'right'.
            mapped_motion: Pre-mapped BVH motion data from SkeletonMapper.
            playback_speed: Playback speed multiplier.
            loop: Whether to loop playback.
            grip_sensitivity: How much angular velocity maps to grip.
            max_angle: Maximum finger joint angle in radians.
            smoothing: Exponential smoothing factor for grip amount.
        """
        self._side = side
        self._motion = mapped_motion
        self._playback_speed = playback_speed
        self._loop = loop
        self._grip_sensitivity = grip_sensitivity
        self._max_angle = max_angle
        self._smoothing = smoothing
        self._connected = False
        self._start_time = 0.0
        self._paused = False
        self._pause_time = 0.0
        self._n_joints = 20  # 4 joints x 5 fingers
        self._prev_grip = 0.0

        # Map side to TrackerRole
        self._role = TrackerRole.RIGHT_HAND if side == "right" else TrackerRole.LEFT_HAND
        self._has_data = self._role in mapped_motion.roles

        # Pre-compute angular velocities from orientation changes
        self._grip_sequence = self._compute_grip_sequence()

    def _compute_grip_sequence(self) -> np.ndarray:
        """Pre-compute grip amount for each frame based on angular velocity."""
        if not self._has_data or self._motion.frame_count < 2:
            return np.zeros(max(1, self._motion.frame_count))

        grips = np.zeros(self._motion.frame_count)

        for i in range(1, self._motion.frame_count):
            prev_quat = self._motion.frames[i - 1].poses[self._role].orientation
            curr_quat = self._motion.frames[i].poses[self._role].orientation

            # Compute angular difference (simple quaternion distance)
            dot = np.abs(np.dot(prev_quat, curr_quat))
            dot = np.clip(dot, 0.0, 1.0)
            angle_diff = 2.0 * np.arccos(dot)

            # Convert to angular velocity
            dt = self._motion.frame_time
            angular_vel = angle_diff / max(dt, 1e-6)

            # Map to grip amount (0=open, 1=closed)
            grip = np.clip(angular_vel * self._grip_sensitivity / 10.0, 0.0, 1.0)
            grips[i] = grip

        # Smooth the grip sequence
        smoothed = np.zeros_like(grips)
        smoothed[0] = grips[0]
        for i in range(1, len(grips)):
            smoothed[i] = self._smoothing * grips[i] + (1 - self._smoothing) * smoothed[i - 1]

        return smoothed

    def initialize(self) -> bool:
        self._start_time = time.monotonic()
        self._connected = True
        self._prev_grip = 0.0
        logger.info(f"BVHHandAdapter initialized: {self._side}")
        return True

    def get_joint_state(self) -> HandJointState:
        if not self._connected:
            return HandJointState(valid=False)

        # Compute current frame
        if self._paused:
            elapsed = (self._pause_time - self._start_time) * self._playback_speed
        else:
            elapsed = (time.monotonic() - self._start_time) * self._playback_speed
        total_duration = self._motion.frame_count * self._motion.frame_time

        if total_duration <= 0:
            return HandJointState(valid=False)

        if self._loop:
            elapsed = elapsed % total_duration
        elif elapsed >= total_duration:
            return HandJointState(valid=False)

        frame_idx = int(elapsed / self._motion.frame_time)
        frame_idx = min(frame_idx, self._motion.frame_count - 1)

        # Get grip amount for this frame
        grip = self._grip_sequence[frame_idx] if self._has_data else 0.0

        # Convert grip to 20 joint angles
        angles = np.zeros(self._n_joints)
        for finger_idx in range(5):
            # Slight phase offset per finger
            finger_grip = np.clip(grip + (finger_idx - 2) * 0.05, 0.0, 1.0)
            for joint_idx in range(4):
                idx = finger_idx * 4 + joint_idx
                # Distal joints move proportionally less
                joint_scale = 1.0 - 0.15 * joint_idx
                angles[idx] = finger_grip * self._max_angle * joint_scale

        return HandJointState(
            joint_angles=angles,
            timestamp=elapsed,
            side=self._side,
            valid=True,
        )

    def is_connected(self) -> bool:
        return self._connected

    def get_side(self) -> str:
        return self._side

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

    def shutdown(self) -> None:
        self._connected = False
        logger.info(f"BVHHandAdapter shutdown: {self._side}")
