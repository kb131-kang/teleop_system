"""Camera/head teleoperation controller logic.

Pure control logic (no ROS2) -- reads HMD head tracker orientation,
extracts pan/tilt angles, and commands the camera via ICameraStream.
"""

import numpy as np

from teleop_system.interfaces.camera_stream import ICameraStream
from teleop_system.interfaces.master_device import IMasterTracker
from teleop_system.utils.logger import get_logger
from teleop_system.utils.transforms import quat_to_euler

logger = get_logger("camera_controller")


class CameraController:
    """Controls camera pan-tilt based on HMD head tracker orientation.

    Reads from IMasterTracker (HEAD role), extracts yaw -> pan and
    pitch -> tilt, applies smoothing and velocity limiting, and sends
    to ICameraStream.
    """

    def __init__(
        self,
        tracker: IMasterTracker,
        camera: ICameraStream,
        smoothing_alpha: float = 0.3,
        max_angular_velocity: float = 2.0,
        dt: float = 1.0 / 30.0,
    ):
        """Initialize camera controller.

        Args:
            tracker: HMD tracker (HEAD role) providing orientation.
            camera: Camera stream to control (SimCameraStream or ROS2CameraAdapter).
            smoothing_alpha: EMA smoothing factor (0=no change, 1=no smoothing).
            max_angular_velocity: Max angular velocity in rad/s.
            dt: Control loop timestep in seconds.
        """
        self._tracker = tracker
        self._camera = camera
        self._smoothing_alpha = smoothing_alpha
        self._max_angular_velocity = max_angular_velocity
        self._dt = dt
        self._enabled = False
        self._prev_pan: float | None = None
        self._prev_tilt: float | None = None

    def enable(self) -> None:
        self._enabled = True
        logger.info("CameraController enabled")

    def disable(self) -> None:
        self._enabled = False
        self._prev_pan = None
        self._prev_tilt = None
        logger.info("CameraController disabled")

    def update(self) -> tuple[float, float] | None:
        """Run one control cycle: read HMD -> extract pan/tilt -> send to camera.

        Extracts euler angles from HMD quaternion orientation:
          - yaw (rotation about Z-axis) -> pan
          - pitch (rotation about Y-axis) -> tilt
        Applies exponential smoothing and velocity limiting.

        Returns:
            (pan, tilt) tuple if command sent, None if disabled/invalid.
        """
        if not self._enabled:
            return None

        if not self._tracker.is_connected() or not self._camera.is_connected():
            return None

        # Read tracker orientation
        pose = self._tracker.get_pose()
        if not pose.valid:
            return None

        # Extract euler angles: [roll, pitch, yaw] in sxyz convention
        euler = quat_to_euler(pose.orientation, axes="sxyz")
        raw_pan = float(euler[2])   # yaw -> head_0 (Z-axis)
        raw_tilt = float(euler[1])  # pitch -> head_1 (Y-axis)

        # Apply EMA smoothing
        if self._prev_pan is not None:
            pan = self._prev_pan + self._smoothing_alpha * (raw_pan - self._prev_pan)
            tilt = self._prev_tilt + self._smoothing_alpha * (raw_tilt - self._prev_tilt)
        else:
            pan = raw_pan
            tilt = raw_tilt

        # Apply velocity limiting
        if self._prev_pan is not None:
            max_delta = self._max_angular_velocity * self._dt
            pan_delta = np.clip(pan - self._prev_pan, -max_delta, max_delta)
            tilt_delta = np.clip(tilt - self._prev_tilt, -max_delta, max_delta)
            pan = self._prev_pan + pan_delta
            tilt = self._prev_tilt + tilt_delta

        self._prev_pan = pan
        self._prev_tilt = tilt

        # Send to camera
        self._camera.set_orientation(pan, tilt)
        return (pan, tilt)

    @property
    def is_enabled(self) -> bool:
        return self._enabled
