"""Abstract interfaces for the teleoperation system.

All hardware-dependent components must implement these interfaces.
"""

from teleop_system.interfaces.camera_stream import ICameraStream, RGBDFrame
from teleop_system.interfaces.ik_solver import IIKSolver, IKResult
from teleop_system.interfaces.master_device import (
    HandJointState,
    IHandInput,
    IMasterTracker,
    Pose6D,
    TrackerRole,
)
from teleop_system.interfaces.simulator import ISimulator, SimState
from teleop_system.interfaces.slave_robot import (
    ArmSide,
    IMobileBase,
    ISlaveArm,
    ISlaveHand,
    JointState,
    VelocityCommand,
)

__all__ = [
    "ICameraStream",
    "IHandInput",
    "IIKSolver",
    "IMasterTracker",
    "IMobileBase",
    "ISimulator",
    "ISlaveArm",
    "ISlaveHand",
    "ArmSide",
    "HandJointState",
    "IKResult",
    "JointState",
    "Pose6D",
    "RGBDFrame",
    "SimState",
    "TrackerRole",
    "VelocityCommand",
]
