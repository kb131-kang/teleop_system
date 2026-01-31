"""Simulator interface for physics backends.

Defines the abstract interface for simulation environments (Isaac Lab, MuJoCo)
that provide physics simulation, rendering, and state access.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

from teleop_system.interfaces.camera_stream import RGBDFrame


@dataclass
class SimState:
    """Full simulation state snapshot.

    Attributes:
        joint_positions: (N,) all joint positions in radians.
        joint_velocities: (N,) all joint velocities in rad/s.
        base_position: (3,) mobile base position in world frame (meters).
        base_orientation: (4,) mobile base orientation as xyzw quaternion.
        timestamp: Simulation time in seconds.
    """
    joint_positions: np.ndarray = field(default_factory=lambda: np.zeros(0))
    joint_velocities: np.ndarray = field(default_factory=lambda: np.zeros(0))
    base_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    timestamp: float = 0.0


class ISimulator(ABC):
    """Abstract interface for physics simulation backends.

    Implementations: MuJoCoSimulator, IsaacLabSimulator.
    Provides unified access to simulation state, control, and rendering.
    """

    @abstractmethod
    def initialize(self, config: dict) -> bool:
        """Initialize the simulator with configuration.

        Args:
            config: Configuration dict with keys like:
                - urdf_path: Robot URDF file path.
                - timestep: Physics timestep in seconds.
                - render: Whether to enable visual rendering.

        Returns:
            True on success.
        """
        ...

    @abstractmethod
    def step(self, dt: float | None = None) -> None:
        """Advance simulation by one timestep.

        Args:
            dt: Optional override for physics timestep.
        """
        ...

    @abstractmethod
    def get_state(self) -> SimState:
        """Get the current full simulation state."""
        ...

    @abstractmethod
    def set_joint_positions(self, positions: np.ndarray) -> None:
        """Set joint position targets for the robot.

        Args:
            positions: (N,) target joint positions in radians.
        """
        ...

    @abstractmethod
    def set_base_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        """Set mobile base velocity command.

        Args:
            linear_x: Forward/backward velocity (m/s).
            linear_y: Left/right velocity (m/s).
            angular_z: Rotational velocity (rad/s).
        """
        ...

    @abstractmethod
    def render(self) -> np.ndarray | None:
        """Render the current simulation state.

        Returns:
            (H, W, 3) RGB image or None if rendering is disabled.
        """
        ...

    @abstractmethod
    def get_camera_rgbd(self, camera_name: str) -> RGBDFrame:
        """Get RGB-D render from a named camera in the simulation.

        Args:
            camera_name: Name of the camera in the scene.

        Returns:
            RGBDFrame with rendered RGB and depth.
        """
        ...

    @abstractmethod
    def get_joint_names(self) -> list[str]:
        """Get ordered list of all controllable joint names."""
        ...

    @abstractmethod
    def get_time(self) -> float:
        """Get current simulation time in seconds."""
        ...

    @abstractmethod
    def reset(self) -> None:
        """Reset simulation to initial state."""
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """Clean up simulator resources."""
        ...
