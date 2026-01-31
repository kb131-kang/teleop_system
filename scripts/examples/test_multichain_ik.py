#!/usr/bin/env python3
"""Example: Test 3-chain IK (left arm, right arm, torso) with SimulatedTrackers.

Demonstrates the full Phase 3 pipeline: 3 SimulatedTrackers drive
a branching humanoid upper-body model through the PinkIKSolver,
with per-chain weight configuration and error reporting.

Usage:
    python scripts/examples/test_multichain_ik.py
"""

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import numpy as np

from teleop_system.interfaces.master_device import Pose6D, TrackerRole
from teleop_system.interfaces.slave_robot import ArmSide
from teleop_system.modules.arm_teleop.arm_controller import ArmController
from teleop_system.simulators.simulated_tracker import SimulatedTracker
from teleop_system.solvers.pink_ik_solver import ChainConfig, PinkIKSolver
from teleop_system.utils.logger import get_logger

logger = get_logger("test_multichain")


def create_upper_body_urdf() -> str:
    """Create a branching humanoid upper-body URDF for testing."""
    urdf = """\
<?xml version="1.0"?>
<robot name="upper_body_test">
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" iyy="0.05" izz="0.05" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

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
</robot>"""
    path = Path(tempfile.mkdtemp()) / "upper_body.urdf"
    path.write_text(urdf)
    return str(path)


def main():
    urdf_path = create_upper_body_urdf()
    logger.info(f"Using URDF: {urdf_path}")

    # Initialize IK solver with 3 chains and per-chain weights
    solver = PinkIKSolver()
    ok = solver.initialize(
        urdf_path,
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
        posture_cost=0.01,
    )

    if not ok:
        logger.error("Failed to initialize IK solver")
        return

    nq = solver.get_joint_count()
    logger.info(f"DOF: {nq}, Chains: {solver.get_chain_names()}")
    logger.info(f"Joint names: {solver.get_joint_names()}")

    # Show neutral EE positions
    for chain in solver.get_chain_names():
        pos = solver.get_ee_position(chain)
        logger.info(f"  {chain} neutral EE: {pos}")

    # Create SimulatedTrackers centered near neutral EE positions
    left_pos = solver.get_ee_position("left_arm")
    right_pos = solver.get_ee_position("right_arm")
    torso_pos = solver.get_ee_position("torso")

    trackers = {
        TrackerRole.LEFT_HAND: SimulatedTracker(
            role=TrackerRole.LEFT_HAND,
            center_position=left_pos,
            amplitude=0.05,
            frequency=0.3,
        ),
        TrackerRole.RIGHT_HAND: SimulatedTracker(
            role=TrackerRole.RIGHT_HAND,
            center_position=right_pos,
            amplitude=0.05,
            frequency=0.3,
            phase_offset=np.pi / 2,
        ),
        TrackerRole.WAIST: SimulatedTracker(
            role=TrackerRole.WAIST,
            center_position=torso_pos,
            amplitude=0.02,
            frequency=0.2,
            phase_offset=np.pi,
        ),
    }
    for t in trackers.values():
        t.initialize()

    # Build joint index map from solver
    left_indices = solver.get_joint_indices_for_names(
        ["left_shoulder", "left_elbow", "left_wrist"]
    )
    right_indices = solver.get_joint_indices_for_names(
        ["right_shoulder", "right_elbow", "right_wrist"]
    )
    torso_indices = solver.get_joint_indices_for_names(["torso_joint"])

    logger.info(f"Joint index map: torso={torso_indices}, left={left_indices}, right={right_indices}")

    # Run IK loop
    q = np.zeros(nq)
    logger.info("Running 200 IK steps with 3 chains...")

    for i in range(200):
        # Get tracker poses
        targets = {}
        for role, tracker in trackers.items():
            pose = tracker.get_pose()
            if pose.valid:
                chain_map = {
                    TrackerRole.LEFT_HAND: "left_arm",
                    TrackerRole.RIGHT_HAND: "right_arm",
                    TrackerRole.WAIST: "torso",
                }
                targets[chain_map[role]] = pose

        result = solver.solve_multi(targets, q, dt=0.01)
        if result.success:
            q = result.joint_positions

        if (i + 1) % 50 == 0:
            errors = solver.get_chain_errors()
            logger.info(f"--- Step {i+1} ---")
            logger.info(
                f"  Total: pos_err={result.error_position:.6f} m, "
                f"ori_err={result.error_orientation:.6f} rad"
            )
            for chain, err in errors.items():
                logger.info(
                    f"  {chain}: pos={err.position_error:.6f} m, "
                    f"ori={err.orientation_error:.6f} rad"
                )

    logger.info(f"Final joints: {q}")

    # Show final EE positions
    for chain in solver.get_chain_names():
        pos = solver.get_ee_position(chain)
        logger.info(f"  {chain} final EE: {pos}")

    logger.info("Multi-chain IK test completed!")


if __name__ == "__main__":
    main()
