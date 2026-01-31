"""Camera streaming interface for RGB-D data and VR rendering.

Defines the abstract interface for camera devices that provide RGB-D data
and support pan-tilt orientation control.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np


@dataclass
class RGBDFrame:
    """A single RGB-D frame from the camera.

    Attributes:
        rgb: (H, W, 3) RGB image as uint8.
        depth: (H, W) depth image in meters as float32.
        intrinsics: (3, 3) camera intrinsic matrix.
        timestamp: Capture time in seconds.
        width: Image width in pixels.
        height: Image height in pixels.
    """
    rgb: np.ndarray = field(default_factory=lambda: np.zeros((480, 640, 3), dtype=np.uint8))
    depth: np.ndarray = field(default_factory=lambda: np.zeros((480, 640), dtype=np.float32))
    intrinsics: np.ndarray = field(default_factory=lambda: np.eye(3))
    timestamp: float = 0.0
    width: int = 640
    height: int = 480


class ICameraStream(ABC):
    """Abstract interface for RGB-D camera with pan-tilt control.

    Implementations: RealSenseCamera (ROS2), SimCameraStream (simulator render).
    """

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize camera connection. Returns True on success."""
        ...

    @abstractmethod
    def get_rgbd(self) -> RGBDFrame:
        """Get the latest RGB-D frame.

        Returns:
            RGBDFrame with RGB image, depth map, and intrinsics.
        """
        ...

    @abstractmethod
    def set_orientation(self, pan: float, tilt: float) -> None:
        """Set camera pan-tilt orientation.

        Args:
            pan: Pan angle in radians (yaw).
            tilt: Tilt angle in radians (pitch).
        """
        ...

    @abstractmethod
    def get_orientation(self) -> tuple[float, float]:
        """Get current camera pan-tilt angles.

        Returns:
            Tuple of (pan, tilt) in radians.
        """
        ...

    @abstractmethod
    def get_intrinsics(self) -> np.ndarray:
        """Get camera intrinsic matrix (3x3)."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if the camera is connected and providing data."""
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """Release camera resources."""
        ...
