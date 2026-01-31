#!/usr/bin/env python3
"""Example: Test PinkIKSolver standalone with a URDF.

This script tests the IK solver independently. If no URDF is provided,
it creates a simple test robot using Pinocchio's built-in models.

Usage:
    python scripts/examples/test_ik_solver.py
    python scripts/examples/test_ik_solver.py --urdf models/rby1/rby1.urdf
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import numpy as np
import pinocchio as pin

from teleop_system.interfaces.master_device import Pose6D
from teleop_system.solvers.pink_ik_solver import PinkIKSolver
from teleop_system.utils.logger import get_logger

logger = get_logger("test_ik")


def create_test_urdf() -> str:
    """Create a simple 6-DOF robot URDF for testing."""
    urdf_content = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="link1">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" velocity="2.0" effort="100"/>
  </joint>

  <link name="link2">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" velocity="2.0" effort="100"/>
  </joint>

  <link name="link3">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" velocity="2.0" effort="100"/>
  </joint>

  <link name="link4">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" iyy="0.005" izz="0.005" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint4" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" velocity="2.0" effort="100"/>
  </joint>

  <link name="link5">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" iyy="0.005" izz="0.005" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint5" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" velocity="2.0" effort="100"/>
  </joint>

  <link name="ee_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint6" type="revolute">
    <parent link="link5"/>
    <child link="ee_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" velocity="2.0" effort="100"/>
  </joint>
</robot>"""

    import tempfile
    urdf_path = Path(tempfile.mkdtemp()) / "test_robot.urdf"
    urdf_path.write_text(urdf_content)
    return str(urdf_path)


def main():
    parser = argparse.ArgumentParser(description="Test PinkIKSolver")
    parser.add_argument("--urdf", type=str, default=None, help="URDF file path")
    args = parser.parse_args()

    # Use provided URDF or create test one
    urdf_path = args.urdf or create_test_urdf()
    logger.info(f"Using URDF: {urdf_path}")

    # Initialize IK solver
    solver = PinkIKSolver()
    success = solver.initialize(
        urdf_path=urdf_path,
        ee_frames={"right_arm": "ee_link"},
        position_cost=1.0,
        orientation_cost=0.5,
        posture_cost=0.01,
        lm_damping=1e-3,
    )

    if not success:
        logger.error("Failed to initialize IK solver")
        return

    logger.info(f"Joint count: {solver.get_joint_count()}")
    logger.info(f"Joint names: {solver.get_joint_names()}")

    # Test single solve
    q_init = np.zeros(solver.get_joint_count())
    target = Pose6D(
        position=np.array([0.3, 0.1, 0.8]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    logger.info(f"Target position: {target.position}")

    # Run 100 IK steps
    q = q_init.copy()
    for i in range(100):
        result = solver.solve(target, q, dt=0.01)
        if result.success:
            q = result.joint_positions
        if (i + 1) % 20 == 0:
            logger.info(
                f"Step {i+1}: pos_err={result.error_position:.6f} m, "
                f"ori_err={result.error_orientation:.6f} rad"
            )

    logger.info(f"Final joints: {q}")
    logger.info("IK test completed successfully!")


if __name__ == "__main__":
    main()
