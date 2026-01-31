"""Phase 3 integration tests: 3-chain IK (left arm, right arm, torso).

Tests PinkIKSolver with per-chain weights, ArmController with 3 trackers
and proper per-chain joint dispatch, using a branching humanoid upper-body
test URDF.
"""

import tempfile
from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest

from teleop_system.interfaces.ik_solver import IKResult
from teleop_system.interfaces.master_device import IMasterTracker, Pose6D, TrackerRole
from teleop_system.interfaces.slave_robot import ArmSide, ISlaveArm, JointState
from teleop_system.modules.arm_teleop.arm_controller import ArmController
from teleop_system.simulators.simulated_tracker import SimulatedTracker
from teleop_system.solvers.pink_ik_solver import ChainConfig, ChainError, PinkIKSolver


# ---------------------------------------------------------------------------
# Test URDF: branching humanoid upper body
#   base_link
#     └─ torso_joint (revolute Z) → torso_link
#           ├─ left_shoulder (revolute Y) → left_upper_arm
#           │     └─ left_elbow (revolute Y) → left_forearm
#           │           └─ left_wrist (revolute Z) → left_hand_link
#           └─ right_shoulder (revolute Y) → right_upper_arm
#                 └─ right_elbow (revolute Y) → right_forearm
#                       └─ right_wrist (revolute Z) → right_hand_link
# ---------------------------------------------------------------------------

_MULTICHAIN_URDF = """\
<?xml version="1.0"?>
<robot name="upper_body_test">
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" iyy="0.05" izz="0.05" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso_link">
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.03" iyy="0.03" izz="0.03" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="torso_joint" type="revolute">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" velocity="2.0" effort="100"/>
  </joint>

  <!-- Left arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="left_shoulder" type="revolute">
    <parent link="torso_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" velocity="2.0" effort="100"/>
  </joint>

  <link name="left_forearm">
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.008" iyy="0.008" izz="0.008" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="0.1" velocity="2.0" effort="100"/>
  </joint>

  <link name="left_hand_link">
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.003" iyy="0.003" izz="0.003" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="left_wrist" type="revolute">
    <parent link="left_forearm"/>
    <child link="left_hand_link"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" velocity="2.0" effort="100"/>
  </joint>

  <!-- Right arm -->
  <link name="right_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="right_shoulder" type="revolute">
    <parent link="torso_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 -0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" velocity="2.0" effort="100"/>
  </joint>

  <link name="right_forearm">
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.008" iyy="0.008" izz="0.008" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="0.1" velocity="2.0" effort="100"/>
  </joint>

  <link name="right_hand_link">
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.003" iyy="0.003" izz="0.003" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="right_wrist" type="revolute">
    <parent link="right_forearm"/>
    <child link="right_hand_link"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" velocity="2.0" effort="100"/>
  </joint>
</robot>
"""


@pytest.fixture(scope="module")
def multichain_urdf_path() -> str:
    """Write the multi-chain URDF to a temp file and return its path."""
    tmp = Path(tempfile.mkdtemp()) / "upper_body_test.urdf"
    tmp.write_text(_MULTICHAIN_URDF)
    return str(tmp)


# ---------------------------------------------------------------------------
# Mock arm adapter that records per-chain commands
# ---------------------------------------------------------------------------


class MockChainArm(ISlaveArm):
    """Mock arm adapter for a specific chain with configurable joint count."""

    def __init__(self, side: ArmSide, n_joints: int):
        self._side = side
        self._n_joints = n_joints
        self._positions = np.zeros(n_joints)
        self._last_command: np.ndarray | None = None
        self._command_count = 0

    def initialize(self) -> bool:
        return True

    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        self._last_command = joint_positions.copy()
        self._command_count += 1

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


# ===========================================================================
# Test PinkIKSolver 3-chain capabilities
# ===========================================================================


class TestPinkIKSolverMultiChain:
    """Test PinkIKSolver with 3 simultaneous chains."""

    def test_initialize_3_chains(self, multichain_urdf_path):
        solver = PinkIKSolver()
        ok = solver.initialize(
            multichain_urdf_path,
            ee_frames={
                "left_arm": "left_hand_link",
                "right_arm": "right_hand_link",
                "torso": "torso_link",
            },
        )
        assert ok is True
        assert solver.get_joint_count() > 0
        chain_names = solver.get_chain_names()
        assert "left_arm" in chain_names
        assert "right_arm" in chain_names
        assert "torso" in chain_names

    def test_per_chain_weight_config(self, multichain_urdf_path):
        solver = PinkIKSolver()
        ok = solver.initialize(
            multichain_urdf_path,
            ee_frames={
                "left_arm": "left_hand_link",
                "right_arm": "right_hand_link",
                "torso": "torso_link",
            },
            chain_configs={
                "left_arm": ChainConfig(
                    ee_frame="left_hand_link",
                    position_cost=2.0,
                    orientation_cost=1.0,
                ),
                "right_arm": ChainConfig(
                    ee_frame="right_hand_link",
                    position_cost=2.0,
                    orientation_cost=1.0,
                ),
                "torso": ChainConfig(
                    ee_frame="torso_link",
                    position_cost=0.5,
                    orientation_cost=0.3,
                ),
            },
        )
        assert ok is True

    def test_solve_multi_3_chains(self, multichain_urdf_path):
        solver = PinkIKSolver()
        solver.initialize(
            multichain_urdf_path,
            ee_frames={
                "left_arm": "left_hand_link",
                "right_arm": "right_hand_link",
                "torso": "torso_link",
            },
        )

        nq = solver.get_joint_count()
        q = np.zeros(nq)

        # Targets near default EE positions
        targets = {
            "left_arm": Pose6D(
                position=np.array([0.0, 0.25, -0.05]),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            ),
            "right_arm": Pose6D(
                position=np.array([0.0, -0.25, -0.05]),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            ),
            "torso": Pose6D(
                position=np.array([0.0, 0.0, 0.5]),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            ),
        }

        result = solver.solve_multi(targets, q, dt=0.01)
        assert result.success is True
        assert result.joint_positions.shape == (nq,)

    def test_per_chain_errors_populated(self, multichain_urdf_path):
        solver = PinkIKSolver()
        solver.initialize(
            multichain_urdf_path,
            ee_frames={
                "left_arm": "left_hand_link",
                "right_arm": "right_hand_link",
            },
        )

        nq = solver.get_joint_count()
        targets = {
            "left_arm": Pose6D(
                position=np.array([0.0, 0.3, -0.1]),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            ),
            "right_arm": Pose6D(
                position=np.array([0.0, -0.3, -0.1]),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            ),
        }

        solver.solve_multi(targets, np.zeros(nq), dt=0.01)
        errors = solver.get_chain_errors()
        assert "left_arm" in errors
        assert "right_arm" in errors
        assert isinstance(errors["left_arm"], ChainError)
        assert errors["left_arm"].position_error >= 0.0

    def test_update_chain_weights_at_runtime(self, multichain_urdf_path):
        solver = PinkIKSolver()
        solver.initialize(
            multichain_urdf_path,
            ee_frames={"left_arm": "left_hand_link"},
        )
        assert solver.update_chain_weights("left_arm", position_cost=5.0) is True
        assert solver.update_chain_weights("nonexistent", position_cost=1.0) is False

    def test_get_ee_pose(self, multichain_urdf_path):
        solver = PinkIKSolver()
        solver.initialize(
            multichain_urdf_path,
            ee_frames={"left_arm": "left_hand_link"},
        )
        result = solver.get_ee_pose("left_arm")
        assert result is not None
        pos, rot = result
        assert pos.shape == (3,)
        assert rot.shape == (3, 3)

    def test_get_ee_pose_unknown_chain(self, multichain_urdf_path):
        solver = PinkIKSolver()
        solver.initialize(
            multichain_urdf_path,
            ee_frames={"left_arm": "left_hand_link"},
        )
        assert solver.get_ee_pose("nonexistent") is None

    def test_joint_index_lookup(self, multichain_urdf_path):
        solver = PinkIKSolver()
        solver.initialize(
            multichain_urdf_path,
            ee_frames={"left_arm": "left_hand_link"},
        )
        names = solver.get_joint_names()
        assert len(names) > 0
        # Each joint name should have a valid index
        for name in names:
            idx = solver.get_joint_index_by_name(name)
            assert idx is not None
            assert idx >= 0

    def test_get_joint_indices_for_names(self, multichain_urdf_path):
        solver = PinkIKSolver()
        solver.initialize(
            multichain_urdf_path,
            ee_frames={"left_arm": "left_hand_link"},
        )
        indices = solver.get_joint_indices_for_names(["left_shoulder", "left_elbow"])
        assert len(indices) == 2
        assert indices[0] != indices[1]

    def test_3chain_convergence(self, multichain_urdf_path):
        """Run multiple IK iterations and check convergence for 3 chains."""
        solver = PinkIKSolver()
        solver.initialize(
            multichain_urdf_path,
            ee_frames={
                "left_arm": "left_hand_link",
                "right_arm": "right_hand_link",
                "torso": "torso_link",
            },
            posture_cost=0.01,
        )

        nq = solver.get_joint_count()
        q = np.zeros(nq)

        targets = {
            "left_arm": Pose6D(
                position=np.array([0.0, 0.25, -0.05]),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            ),
            "right_arm": Pose6D(
                position=np.array([0.0, -0.25, -0.05]),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            ),
            "torso": Pose6D(
                position=np.array([0.0, 0.0, 0.5]),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            ),
        }

        # Run 200 IK steps
        for _ in range(200):
            result = solver.solve_multi(targets, q, dt=0.01)
            assert result.success is True
            q = result.joint_positions

        # All chains should have converged to small errors
        errors = solver.get_chain_errors()
        for chain_name, err in errors.items():
            assert err.position_error < 0.05, (
                f"{chain_name} pos error too large: {err.position_error}"
            )


# ===========================================================================
# Test ArmController with 3 trackers and per-chain joint dispatch
# ===========================================================================


class MockIKSolver:
    """Simple mock IK solver for ArmController tests."""

    def __init__(self, n_joints: int = 7):
        self._n = n_joints
        self._solve_count = 0

    def initialize(self, urdf_path, **kwargs):
        return True

    def solve(self, target_pose, current_joints, dt=0.01):
        return self.solve_multi({"default": target_pose}, current_joints, dt)

    def solve_multi(self, target_poses, current_joints, dt=0.01):
        self._solve_count += 1
        new_joints = current_joints.copy()
        new_joints += 0.01 * np.ones_like(current_joints)
        return IKResult(joint_positions=new_joints, success=True)

    def get_joint_names(self):
        return [f"joint_{i}" for i in range(self._n)]

    def get_joint_count(self):
        return self._n

    def set_posture_target(self, posture):
        pass


class TestArmController3Chain:
    """Test ArmController with 3 simultaneous tracker channels."""

    def _make_3chain_controller(self):
        ik = MockIKSolver(n_joints=7)

        trackers = {
            TrackerRole.RIGHT_HAND: SimulatedTracker(
                role=TrackerRole.RIGHT_HAND, amplitude=0.0
            ),
            TrackerRole.LEFT_HAND: SimulatedTracker(
                role=TrackerRole.LEFT_HAND, amplitude=0.0
            ),
            TrackerRole.WAIST: SimulatedTracker(
                role=TrackerRole.WAIST, amplitude=0.0
            ),
        }
        for t in trackers.values():
            t.initialize()

        arms = {
            ArmSide.RIGHT: MockChainArm(ArmSide.RIGHT, n_joints=3),
            ArmSide.LEFT: MockChainArm(ArmSide.LEFT, n_joints=3),
            ArmSide.TORSO: MockChainArm(ArmSide.TORSO, n_joints=1),
        }

        # Joint index map: torso=0, left=1,2,3, right=4,5,6
        joint_index_map = {
            ArmSide.TORSO: [0],
            ArmSide.LEFT: [1, 2, 3],
            ArmSide.RIGHT: [4, 5, 6],
        }

        controller = ArmController(
            ik_solver=ik,
            trackers=trackers,
            arms=arms,
            dt=0.01,
            joint_index_map=joint_index_map,
        )
        return controller, ik, trackers, arms

    def test_3_trackers_active(self):
        controller, _, _, _ = self._make_3chain_controller()
        assert controller.active_chain_count == 3

    def test_3_chains_dispatched(self):
        controller, ik, _, arms = self._make_3chain_controller()
        controller.enable()
        result = controller.update()
        assert result.success is True
        assert ik._solve_count == 1

        # All 3 arms should have received commands
        for side, arm in arms.items():
            assert arm._last_command is not None, f"{side} got no command"
            assert arm._command_count == 1, f"{side} command count wrong"

    def test_per_chain_joint_slicing(self):
        controller, _, _, arms = self._make_3chain_controller()
        controller.enable()
        controller.update()

        # Torso should get 1 joint
        assert len(arms[ArmSide.TORSO]._last_command) == 1
        # Left arm should get 3 joints
        assert len(arms[ArmSide.LEFT]._last_command) == 3
        # Right arm should get 3 joints
        assert len(arms[ArmSide.RIGHT]._last_command) == 3

    def test_sync_from_arms(self):
        controller, _, _, arms = self._make_3chain_controller()
        # Set arm positions
        arms[ArmSide.TORSO]._positions = np.array([0.5])
        arms[ArmSide.LEFT]._positions = np.array([0.1, 0.2, 0.3])
        arms[ArmSide.RIGHT]._positions = np.array([0.4, 0.5, 0.6])

        controller.sync_from_arms()
        controller.enable()
        result = controller.update()
        assert result.success is True

    def test_partial_tracker_disconnect(self):
        """Controller should still work if one tracker disconnects."""
        controller, ik, trackers, arms = self._make_3chain_controller()
        controller.enable()

        # Disconnect left hand tracker
        trackers[TrackerRole.LEFT_HAND].shutdown()
        assert controller.active_chain_count == 2

        result = controller.update()
        assert result.success is True
        # IK was still called (with 2 targets)
        assert ik._solve_count == 1

    def test_all_trackers_disconnected_returns_failure(self):
        controller, _, trackers, _ = self._make_3chain_controller()
        controller.enable()

        for t in trackers.values():
            t.shutdown()

        result = controller.update()
        assert result.success is False

    def test_multiple_update_cycles(self):
        controller, ik, _, arms = self._make_3chain_controller()
        controller.enable()

        for _ in range(50):
            result = controller.update()
            assert result.success is True

        assert ik._solve_count == 50
        for arm in arms.values():
            assert arm._command_count == 50


# ===========================================================================
# End-to-end: SimulatedTrackers → PinkIKSolver → ArmController
# ===========================================================================


class TestPhase3EndToEnd:
    """Full pipeline: 3 SimulatedTrackers → PinkIKSolver → ArmController."""

    def test_full_pipeline(self, multichain_urdf_path):
        # Initialize IK solver with 3 chains
        solver = PinkIKSolver()
        ok = solver.initialize(
            multichain_urdf_path,
            ee_frames={
                "left_arm": "left_hand_link",
                "right_arm": "right_hand_link",
                "torso": "torso_link",
            },
            posture_cost=0.01,
        )
        assert ok is True

        nq = solver.get_joint_count()

        # Get neutral EE positions as tracker centers
        left_pos = solver.get_ee_position("left_arm")
        right_pos = solver.get_ee_position("right_arm")
        torso_pos = solver.get_ee_position("torso")

        # Create trackers centered on the current EE positions (small amplitude)
        trackers = {
            TrackerRole.LEFT_HAND: SimulatedTracker(
                role=TrackerRole.LEFT_HAND,
                center_position=left_pos,
                amplitude=0.02,
                frequency=0.5,
            ),
            TrackerRole.RIGHT_HAND: SimulatedTracker(
                role=TrackerRole.RIGHT_HAND,
                center_position=right_pos,
                amplitude=0.02,
                frequency=0.5,
                phase_offset=1.0,
            ),
            TrackerRole.WAIST: SimulatedTracker(
                role=TrackerRole.WAIST,
                center_position=torso_pos,
                amplitude=0.01,
                frequency=0.3,
                phase_offset=2.0,
            ),
        }
        for t in trackers.values():
            t.initialize()

        # Create mock arms (no actual dispatch needed for this test)
        arms = {
            ArmSide.LEFT: MockChainArm(ArmSide.LEFT, n_joints=3),
            ArmSide.RIGHT: MockChainArm(ArmSide.RIGHT, n_joints=3),
            ArmSide.TORSO: MockChainArm(ArmSide.TORSO, n_joints=1),
        }

        # Build joint index map from URDF
        left_indices = solver.get_joint_indices_for_names(
            ["left_shoulder", "left_elbow", "left_wrist"]
        )
        right_indices = solver.get_joint_indices_for_names(
            ["right_shoulder", "right_elbow", "right_wrist"]
        )
        torso_indices = solver.get_joint_indices_for_names(["torso_joint"])

        joint_index_map = {
            ArmSide.LEFT: left_indices,
            ArmSide.RIGHT: right_indices,
            ArmSide.TORSO: torso_indices,
        }

        controller = ArmController(
            ik_solver=solver,
            trackers=trackers,
            arms=arms,
            dt=0.01,
            joint_index_map=joint_index_map,
            max_joint_velocity=5.0,
        )

        controller.enable()

        # Run 100 control cycles
        successes = 0
        for _ in range(100):
            result = controller.update()
            if result.success:
                successes += 1

        # Most cycles should succeed
        assert successes > 90, f"Only {successes}/100 succeeded"

        # All arms should have received commands
        for side, arm in arms.items():
            assert arm._command_count > 0, f"{side} got no commands"

        # Check that per-chain errors are reasonable after convergence
        chain_errors = solver.get_chain_errors()
        for chain_name, err in chain_errors.items():
            # With small tracker amplitude, errors should stay moderate
            assert err.position_error < 0.5, (
                f"{chain_name} pos_error={err.position_error:.4f} too large"
            )
