"""Phase 6 tests: Hardware device driver structure and data model.

Tests that device drivers:
- Import correctly (even without hardware SDKs)
- Have proper interface signatures
- Handle missing SDK gracefully
"""

import numpy as np
import pytest

from teleop_system.interfaces.master_device import TrackerRole
from teleop_system.interfaces.slave_robot import ArmSide


class TestViveTrackerImport:
    def test_module_imports(self):
        from teleop_system.devices.vive_tracker import ViveTracker, ViveTrackerManager
        assert ViveTracker is not None
        assert ViveTrackerManager is not None

    def test_tracker_instantiation(self):
        from teleop_system.devices.vive_tracker import ViveTracker
        tracker = ViveTracker(role=TrackerRole.RIGHT_HAND, device_index=-1)
        assert tracker.get_role() == TrackerRole.RIGHT_HAND
        assert tracker.is_connected() is False

    def test_tracker_invalid_returns_invalid_pose(self):
        from teleop_system.devices.vive_tracker import ViveTracker
        tracker = ViveTracker(role=TrackerRole.LEFT_HAND, device_index=-1)
        pose = tracker.get_pose()
        assert pose.valid is False

    def test_tracker_shutdown(self):
        from teleop_system.devices.vive_tracker import ViveTracker
        tracker = ViveTracker(role=TrackerRole.WAIST, device_index=0)
        tracker.shutdown()
        assert tracker.is_connected() is False

    def test_role_name_map(self):
        from teleop_system.devices.vive_tracker import _ROLE_NAME_MAP
        assert "right_hand" in _ROLE_NAME_MAP
        assert "left_hand" in _ROLE_NAME_MAP
        assert "waist" in _ROLE_NAME_MAP
        assert "right_foot" in _ROLE_NAME_MAP
        assert "left_foot" in _ROLE_NAME_MAP

    def test_hmd_matrix_to_position_quat(self):
        from teleop_system.devices.vive_tracker import _hmd_matrix_to_position_quat
        # Identity-like 3x4 matrix
        mat = [[1, 0, 0, 1.0], [0, 1, 0, 2.0], [0, 0, 1, 3.0]]
        pos, quat = _hmd_matrix_to_position_quat(mat)
        np.testing.assert_array_almost_equal(pos, [1.0, 2.0, 3.0])
        assert quat.shape == (4,)
        assert abs(np.linalg.norm(quat) - 1.0) < 1e-6


class TestManusGloveImport:
    def test_module_imports(self):
        from teleop_system.devices.manus_glove import ManusGlove
        assert ManusGlove is not None

    def test_instantiation(self):
        from teleop_system.devices.manus_glove import ManusGlove
        glove = ManusGlove(side="left")
        assert glove.get_side() == "left"
        assert glove.is_connected() is False

    def test_not_connected_returns_invalid(self):
        from teleop_system.devices.manus_glove import ManusGlove
        glove = ManusGlove(side="right")
        state = glove.get_joint_state()
        assert state.valid is False


class TestRBY1Import:
    def test_arm_imports(self):
        from teleop_system.devices.rby1_arm import RBY1Arm, RBY1Base
        assert RBY1Arm is not None
        assert RBY1Base is not None

    def test_arm_instantiation(self):
        from teleop_system.devices.rby1_arm import RBY1Arm
        arm = RBY1Arm(side=ArmSide.RIGHT, joint_count=7)
        assert arm.get_side() == ArmSide.RIGHT
        assert arm.get_joint_count() == 7
        assert arm.is_connected() is False

    def test_base_instantiation(self):
        from teleop_system.devices.rby1_arm import RBY1Base
        base = RBY1Base()
        assert base.is_connected() is False


class TestDG5FHandImport:
    def test_module_imports(self):
        from teleop_system.devices.dg5f_hand import DG5FHand
        assert DG5FHand is not None

    def test_instantiation(self):
        from teleop_system.devices.dg5f_hand import DG5FHand
        hand = DG5FHand(side="left")
        assert hand.get_side() == "left"
        assert hand.get_joint_count() == 20
        assert hand.is_connected() is False

    def test_not_connected_returns_zeros(self):
        from teleop_system.devices.dg5f_hand import DG5FHand
        hand = DG5FHand(side="right")
        state = hand.get_joint_state()
        assert state.positions.shape == (20,)


class TestRealSenseCameraImport:
    def test_module_imports(self):
        from teleop_system.devices.realsense_camera import RealSenseCamera
        assert RealSenseCamera is not None

    def test_instantiation(self):
        from teleop_system.devices.realsense_camera import RealSenseCamera
        cam = RealSenseCamera()
        assert cam.is_connected() is False

    def test_get_orientation_default(self):
        from teleop_system.devices.realsense_camera import RealSenseCamera
        cam = RealSenseCamera()
        pan, tilt = cam.get_orientation()
        assert pan == 0.0
        assert tilt == 0.0

    def test_get_intrinsics_default(self):
        from teleop_system.devices.realsense_camera import RealSenseCamera
        cam = RealSenseCamera()
        K = cam.get_intrinsics()
        assert K.shape == (3, 3)
        np.testing.assert_array_almost_equal(K, np.eye(3))


class TestLaunchFilesExist:
    def test_launch_files_present(self):
        from pathlib import Path
        launch_dir = Path(__file__).parent.parent / "launch"
        assert (launch_dir / "teleop_sim.launch.py").exists()
        assert (launch_dir / "teleop_full.launch.py").exists()
        assert (launch_dir / "arm_only.launch.py").exists()
        assert (launch_dir / "hand_only.launch.py").exists()
