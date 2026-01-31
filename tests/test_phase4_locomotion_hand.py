"""Phase 4 tests: Locomotion (GaitDetector) and Hand Teleoperation (Retargeting).

Tests gait detection from foot trackers, locomotion controller,
hand retargeting, and hand controller pipeline.
"""

import numpy as np
import pytest

from teleop_system.interfaces.master_device import (
    HandJointState,
    IHandInput,
    IMasterTracker,
    Pose6D,
    TrackerRole,
)
from teleop_system.interfaces.slave_robot import (
    IMobileBase,
    ISlaveHand,
    JointState,
    VelocityCommand,
)
from teleop_system.modules.hand_teleop.hand_controller import HandController
from teleop_system.modules.hand_teleop.retargeting import (
    FINGER_NAMES,
    TOTAL_JOINTS,
    HandRetargeting,
)
from teleop_system.modules.locomotion.gait_detector import GaitDetector
from teleop_system.modules.locomotion.locomotion_controller import LocomotionController
from teleop_system.simulators.simulated_hand import SimulatedHand
from teleop_system.simulators.simulated_tracker import SimulatedTracker


# ---------------------------------------------------------------------------
# Mock helpers
# ---------------------------------------------------------------------------


class FixedTracker(IMasterTracker):
    """Tracker returning a controllable fixed pose."""

    def __init__(self, role: TrackerRole, position: np.ndarray):
        self._role = role
        self._position = position.copy()
        self._orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self._connected = True

    def initialize(self) -> bool:
        return True

    def get_pose(self) -> Pose6D:
        if not self._connected:
            return Pose6D(valid=False)
        return Pose6D(
            position=self._position.copy(),
            orientation=self._orientation.copy(),
            valid=True,
        )

    def set_position(self, pos: np.ndarray):
        self._position = pos.copy()

    def is_connected(self) -> bool:
        return self._connected

    def get_role(self) -> TrackerRole:
        return self._role

    def shutdown(self) -> None:
        self._connected = False


class MockBase(IMobileBase):
    """Mock mobile base that records velocity commands."""

    def __init__(self):
        self._last_cmd = VelocityCommand()
        self._cmd_count = 0

    def initialize(self) -> bool:
        return True

    def send_velocity(self, linear_x: float, linear_y: float, angular_z: float):
        self._last_cmd = VelocityCommand(
            linear_x=linear_x, linear_y=linear_y, angular_z=angular_z
        )
        self._cmd_count += 1

    def get_odometry(self):
        return np.zeros(3), np.array([0, 0, 0, 1.0])

    def stop(self):
        self.send_velocity(0.0, 0.0, 0.0)

    def is_connected(self):
        return True

    def shutdown(self):
        pass


class MockHand(ISlaveHand):
    """Mock robot hand that records received commands."""

    def __init__(self, side: str = "right"):
        self._side = side
        self._last_command: np.ndarray | None = None
        self._cmd_count = 0

    def initialize(self) -> bool:
        return True

    def send_joint_command(self, joint_positions: np.ndarray):
        self._last_command = joint_positions.copy()
        self._cmd_count += 1

    def get_joint_state(self) -> JointState:
        return JointState()

    def get_joint_count(self) -> int:
        return 20

    def get_side(self) -> str:
        return self._side

    def is_connected(self) -> bool:
        return True

    def shutdown(self):
        pass


# ===========================================================================
# GaitDetector tests
# ===========================================================================


class TestGaitDetector:
    def test_calibrate(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        detector = GaitDetector(left, right)

        assert detector.is_calibrated is False
        assert detector.calibrate() is True
        assert detector.is_calibrated is True

    def test_zero_velocity_at_rest(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        detector = GaitDetector(left, right, deadzone=0.02)
        detector.calibrate()

        cmd = detector.update()
        assert abs(cmd.linear_x) < 1e-6
        assert abs(cmd.linear_y) < 1e-6
        assert abs(cmd.angular_z) < 1e-6

    def test_forward_motion(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        detector = GaitDetector(left, right, deadzone=0.01, linear_scale=1.0)
        detector.calibrate()

        # Move both feet forward
        left.set_position(np.array([0.1, 0.15, 0.0]))
        right.set_position(np.array([0.1, -0.15, 0.0]))

        cmd = detector.update()
        assert cmd.linear_x > 0.0, f"Expected positive linear_x, got {cmd.linear_x}"

    def test_lateral_motion(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        detector = GaitDetector(left, right, deadzone=0.01, linear_scale=1.0)
        detector.calibrate()

        # Move both feet laterally
        left.set_position(np.array([0.0, 0.25, 0.0]))
        right.set_position(np.array([0.0, -0.05, 0.0]))

        cmd = detector.update()
        assert cmd.linear_y > 0.0, f"Expected positive linear_y, got {cmd.linear_y}"

    def test_deadzone_filters_noise(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        detector = GaitDetector(left, right, deadzone=0.05)
        detector.calibrate()

        # Small movement within deadzone
        left.set_position(np.array([0.02, 0.15, 0.0]))
        right.set_position(np.array([0.02, -0.15, 0.0]))

        cmd = detector.update()
        assert abs(cmd.linear_x) < 1e-6

    def test_velocity_clamping(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        detector = GaitDetector(
            left, right, deadzone=0.01, linear_scale=10.0, max_linear_velocity=0.5
        )
        detector.calibrate()

        left.set_position(np.array([1.0, 0.15, 0.0]))
        right.set_position(np.array([1.0, -0.15, 0.0]))

        cmd = detector.update()
        assert abs(cmd.linear_x) <= 0.5 + 1e-6

    def test_calibrate_fails_disconnected(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        right.shutdown()

        detector = GaitDetector(left, right)
        assert detector.calibrate() is False

    def test_reset(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        detector = GaitDetector(left, right)
        detector.calibrate()
        assert detector.is_calibrated is True
        detector.reset()
        assert detector.is_calibrated is False


# ===========================================================================
# LocomotionController tests
# ===========================================================================


class TestLocomotionController:
    def test_enable_calibrates(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        base = MockBase()

        controller = LocomotionController(left, right, base)
        assert controller.enable() is True
        assert controller.is_enabled is True

    def test_update_sends_velocity(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        base = MockBase()

        controller = LocomotionController(left, right, base, deadzone=0.01)
        controller.enable()

        # Move forward
        left.set_position(np.array([0.1, 0.15, 0.0]))
        right.set_position(np.array([0.1, -0.15, 0.0]))

        cmd = controller.update()
        assert cmd.linear_x > 0.0
        assert base._cmd_count >= 1

    def test_disable_stops_base(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        base = MockBase()

        controller = LocomotionController(left, right, base)
        controller.enable()
        controller.disable()
        assert controller.is_enabled is False
        # Last cmd should be stop
        assert abs(base._last_cmd.linear_x) < 1e-6

    def test_disabled_returns_zero(self):
        left = FixedTracker(TrackerRole.LEFT_FOOT, np.array([0.0, 0.15, 0.0]))
        right = FixedTracker(TrackerRole.RIGHT_FOOT, np.array([0.0, -0.15, 0.0]))
        base = MockBase()

        controller = LocomotionController(left, right, base)
        cmd = controller.update()
        assert abs(cmd.linear_x) < 1e-6


# ===========================================================================
# HandRetargeting tests
# ===========================================================================


class TestHandRetargeting:
    def test_identity_mapping(self):
        rt = HandRetargeting()
        state = HandJointState(
            joint_angles=np.full(TOTAL_JOINTS, 0.5),
            valid=True,
        )
        output = rt.retarget(state)
        assert output.shape == (TOTAL_JOINTS,)
        np.testing.assert_array_almost_equal(output, 0.5)

    def test_scale_factors(self):
        rt = HandRetargeting(
            scale_factors={"index": [2.0, 2.0, 2.0, 2.0]},
        )
        angles = np.zeros(TOTAL_JOINTS)
        angles[4:8] = 0.3  # Index finger
        state = HandJointState(joint_angles=angles, valid=True)
        output = rt.retarget(state)
        # Index joints should be scaled 2x
        np.testing.assert_array_almost_equal(output[4:8], 0.6)

    def test_offsets(self):
        rt = HandRetargeting(
            offsets={"thumb": [0.1, 0.1, 0.1, 0.1]},
        )
        state = HandJointState(joint_angles=np.zeros(TOTAL_JOINTS), valid=True)
        output = rt.retarget(state)
        np.testing.assert_array_almost_equal(output[0:4], 0.1)

    def test_clamping(self):
        rt = HandRetargeting(max_angle=1.0)
        state = HandJointState(
            joint_angles=np.full(TOTAL_JOINTS, 2.0),
            valid=True,
        )
        output = rt.retarget(state)
        assert np.all(output <= 1.0)
        assert np.all(output >= 0.0)

    def test_smoothing(self):
        rt = HandRetargeting(smoothing_alpha=0.5)
        state1 = HandJointState(joint_angles=np.full(TOTAL_JOINTS, 1.0), valid=True)
        state2 = HandJointState(joint_angles=np.zeros(TOTAL_JOINTS), valid=True)

        out1 = rt.retarget(state1)
        np.testing.assert_array_almost_equal(out1, 1.0)

        out2 = rt.retarget(state2)
        # 0.5 * 1.0 + 0.5 * 0.0 = 0.5
        np.testing.assert_array_almost_equal(out2, 0.5)

    def test_invalid_state_returns_previous(self):
        rt = HandRetargeting()
        valid = HandJointState(joint_angles=np.full(TOTAL_JOINTS, 0.5), valid=True)
        invalid = HandJointState(valid=False)

        rt.retarget(valid)
        output = rt.retarget(invalid)
        np.testing.assert_array_almost_equal(output, 0.5)

    def test_reset_clears_smoothing(self):
        rt = HandRetargeting(smoothing_alpha=0.5)
        state = HandJointState(joint_angles=np.full(TOTAL_JOINTS, 1.0), valid=True)
        rt.retarget(state)
        rt.reset()
        assert rt._prev_output is None


# ===========================================================================
# HandController tests
# ===========================================================================


class TestHandController:
    def test_enable_disable(self):
        glove = SimulatedHand(side="right")
        glove.initialize()
        hand = MockHand("right")
        rt = HandRetargeting()

        ctrl = HandController(glove, hand, rt)
        assert ctrl.is_enabled is False
        ctrl.enable()
        assert ctrl.is_enabled is True
        ctrl.disable()
        assert ctrl.is_enabled is False

    def test_update_sends_command(self):
        glove = SimulatedHand(side="right")
        glove.initialize()
        hand = MockHand("right")
        rt = HandRetargeting()

        ctrl = HandController(glove, hand, rt)
        ctrl.enable()
        result = ctrl.update()

        assert result is not None
        assert result.shape == (TOTAL_JOINTS,)
        assert hand._last_command is not None
        assert hand._cmd_count == 1

    def test_disabled_returns_none(self):
        glove = SimulatedHand(side="right")
        glove.initialize()
        hand = MockHand("right")
        rt = HandRetargeting()

        ctrl = HandController(glove, hand, rt)
        assert ctrl.update() is None

    def test_disconnected_glove_returns_none(self):
        glove = SimulatedHand(side="right")
        # Not initialized = not connected
        hand = MockHand("right")
        rt = HandRetargeting()

        ctrl = HandController(glove, hand, rt)
        ctrl.enable()
        assert ctrl.update() is None

    def test_velocity_limiting(self):
        glove = SimulatedHand(side="left", max_angle=1.4)
        glove.initialize()
        hand = MockHand("left")
        rt = HandRetargeting()

        ctrl = HandController(glove, hand, rt, max_joint_velocity=1.0, dt=0.01)
        ctrl.enable()

        # First update establishes baseline
        ctrl.update()
        # Second update should be velocity-limited
        result = ctrl.update()
        assert result is not None

    def test_multiple_cycles(self):
        glove = SimulatedHand(side="right")
        glove.initialize()
        hand = MockHand("right")
        rt = HandRetargeting()

        ctrl = HandController(glove, hand, rt)
        ctrl.enable()

        for _ in range(50):
            result = ctrl.update()
            assert result is not None

        assert hand._cmd_count == 50


# ===========================================================================
# End-to-end: SimulatedTrackers → Locomotion + SimulatedHand → Hand
# ===========================================================================


class TestPhase4EndToEnd:
    def test_locomotion_pipeline(self):
        left = SimulatedTracker(
            role=TrackerRole.LEFT_FOOT,
            amplitude=0.05,
            frequency=0.5,
        )
        right = SimulatedTracker(
            role=TrackerRole.RIGHT_FOOT,
            amplitude=0.05,
            frequency=0.5,
            phase_offset=np.pi,
        )
        left.initialize()
        right.initialize()

        base = MockBase()
        controller = LocomotionController(
            left, right, base,
            deadzone=0.001,
            linear_scale=1.0,
        )
        assert controller.enable() is True

        # Run several cycles
        for _ in range(20):
            cmd = controller.update()

        assert base._cmd_count >= 20

    def test_hand_pipeline(self):
        glove = SimulatedHand(side="right")
        glove.initialize()
        hand = MockHand("right")
        rt = HandRetargeting()

        ctrl = HandController(glove, hand, rt)
        ctrl.enable()

        for _ in range(20):
            ctrl.update()

        assert hand._cmd_count == 20
        assert hand._last_command is not None
        assert len(hand._last_command) == TOTAL_JOINTS

    def test_dual_hand_pipeline(self):
        """Both left and right hands running simultaneously."""
        left_glove = SimulatedHand(side="left")
        right_glove = SimulatedHand(side="right")
        left_glove.initialize()
        right_glove.initialize()

        left_hand = MockHand("left")
        right_hand = MockHand("right")

        left_rt = HandRetargeting()
        right_rt = HandRetargeting()

        left_ctrl = HandController(left_glove, left_hand, left_rt)
        right_ctrl = HandController(right_glove, right_hand, right_rt)
        left_ctrl.enable()
        right_ctrl.enable()

        for _ in range(30):
            left_ctrl.update()
            right_ctrl.update()

        assert left_hand._cmd_count == 30
        assert right_hand._cmd_count == 30
        assert left_ctrl.side == "left"
        assert right_ctrl.side == "right"
