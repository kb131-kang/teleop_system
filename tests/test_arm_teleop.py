"""Unit tests for the ArmController (no ROS2 dependency)."""

import numpy as np
import pytest

from teleop_system.interfaces.master_device import IMasterTracker, Pose6D, TrackerRole
from teleop_system.interfaces.slave_robot import ISlaveArm, ArmSide, JointState
from teleop_system.interfaces.ik_solver import IIKSolver, IKResult
from teleop_system.modules.arm_teleop.arm_controller import ArmController


class MockTracker(IMasterTracker):
    """Mock tracker that returns a fixed pose."""

    def __init__(self, role: TrackerRole, pose: Pose6D | None = None):
        self._role = role
        self._pose = pose or Pose6D(
            position=np.array([0.4, -0.3, 1.0]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        )
        self._connected = True

    def initialize(self) -> bool:
        return True

    def get_pose(self) -> Pose6D:
        return self._pose

    def is_connected(self) -> bool:
        return self._connected

    def get_role(self) -> TrackerRole:
        return self._role

    def shutdown(self) -> None:
        self._connected = False


class MockArm(ISlaveArm):
    """Mock arm that records received commands."""

    def __init__(self, side: ArmSide, n_joints: int = 7):
        self._side = side
        self._n_joints = n_joints
        self._last_command: np.ndarray | None = None
        self._positions = np.zeros(n_joints)

    def initialize(self) -> bool:
        return True

    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        self._last_command = joint_positions.copy()

    def get_joint_state(self) -> JointState:
        return JointState(positions=self._positions.copy())

    def get_joint_count(self) -> int:
        return self._n_joints

    def get_side(self) -> ArmSide:
        return self._side

    def is_connected(self) -> bool:
        return True

    def shutdown(self) -> None:
        pass


class MockIKSolver(IIKSolver):
    """Mock IK solver that returns predictable results."""

    def __init__(self, n_joints: int = 7):
        self._n_joints = n_joints
        self._solve_count = 0

    def initialize(self, urdf_path: str, **kwargs) -> bool:
        return True

    def solve(self, target_pose, current_joints, dt=0.01):
        return self.solve_multi({"default": target_pose}, current_joints, dt)

    def solve_multi(self, target_poses, current_joints, dt=0.01):
        self._solve_count += 1
        # Return slightly modified joints (simple mock)
        new_joints = current_joints.copy()
        new_joints += 0.01 * np.ones_like(current_joints)
        return IKResult(joint_positions=new_joints, success=True)

    def get_joint_names(self):
        return [f"joint_{i}" for i in range(self._n_joints)]

    def get_joint_count(self):
        return self._n_joints

    def set_posture_target(self, posture):
        pass


class TestArmController:
    def _make_controller(self):
        ik = MockIKSolver(n_joints=7)
        tracker = MockTracker(TrackerRole.RIGHT_HAND)
        arm = MockArm(ArmSide.RIGHT)

        controller = ArmController(
            ik_solver=ik,
            trackers={TrackerRole.RIGHT_HAND: tracker},
            arms={ArmSide.RIGHT: arm},
            dt=0.01,
        )
        return controller, ik, tracker, arm

    def test_disabled_by_default(self):
        controller, _, _, _ = self._make_controller()
        assert controller.is_enabled is False
        result = controller.update()
        assert result.success is False

    def test_enable_and_update(self):
        controller, ik, _, arm = self._make_controller()
        controller.enable()
        assert controller.is_enabled is True

        result = controller.update()
        assert result.success is True
        assert ik._solve_count == 1
        assert arm._last_command is not None

    def test_multiple_updates(self):
        controller, ik, _, _ = self._make_controller()
        controller.enable()

        for _ in range(10):
            result = controller.update()
            assert result.success is True

        assert ik._solve_count == 10

    def test_disable(self):
        controller, _, _, _ = self._make_controller()
        controller.enable()
        controller.disable()
        assert controller.is_enabled is False
        result = controller.update()
        assert result.success is False

    def test_disconnected_tracker(self):
        ik = MockIKSolver(n_joints=7)
        tracker = MockTracker(TrackerRole.RIGHT_HAND)
        tracker.shutdown()  # Disconnect
        arm = MockArm(ArmSide.RIGHT)

        controller = ArmController(
            ik_solver=ik,
            trackers={TrackerRole.RIGHT_HAND: tracker},
            arms={ArmSide.RIGHT: arm},
        )
        controller.enable()
        result = controller.update()
        assert result.success is False

    def test_set_joint_state(self):
        controller, _, _, _ = self._make_controller()
        joints = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        controller.set_joint_state(joints)
        controller.enable()
        result = controller.update()
        assert result.success is True
