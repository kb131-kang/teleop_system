# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RB-Y1 humanoid robot VR teleoperation system. Maps 5 Vive Trackers + 2 Manus Gloves to a dual-arm (2x7DoF) + torso (6DoF) + mobile base robot with DG-5F dexterous hands. Python 3.10+, ROS2 Jazzy, Pink/Pinocchio IK.

## Commands

### Running tests

```bash
# Full suite (system Python)
python3 -m pytest tests/ -v

# Full suite (Isaac Sim Python — required for Isaac Lab integration)
/workspace/isaaclab/_isaac_sim/python.sh -m pytest tests/ -v

# Single test file
python3 -m pytest tests/test_phase3_multichain.py -v

# Single test
python3 -m pytest tests/test_transforms.py::TestQuaternionOperations::test_multiply_identity -v
```

### Running the system

```bash
# Simulation mode (default)
python3 scripts/run_teleop.py --mode simulation --sim-backend mujoco

# Hardware mode
python3 scripts/run_teleop.py --mode hardware

# Selective modules
python3 scripts/run_teleop.py --modules arm locomotion --no-gui

# ROS2 launch files
ros2 launch teleop_system teleop_sim.launch.py sim_backend:=mujoco
ros2 launch teleop_system teleop_full.launch.py robot_ip:=192.168.0.100
ros2 launch teleop_system arm_only.launch.py
ros2 launch teleop_system hand_only.launch.py
```

## Architecture

### Interface-driven design (ABC pattern)

All hardware-dependent components are abstract base classes in `teleop_system/interfaces/`. Concrete implementations exist for both real hardware (`devices/`) and simulation (`simulators/`). The key interfaces:

- **IMasterTracker** → ViveTracker, SimulatedTracker
- **IHandInput** → ManusGlove, SimulatedHand
- **ISlaveArm** → RBY1Arm
- **ISlaveHand** → DG5FHand
- **IMobileBase** → RBY1Base
- **IIKSolver** → PinkIKSolver
- **ISimulator** → MuJoCoSimulator
- **ICameraStream** → RealSenseCamera

### Module independence

Four independent modules, each a ROS2 Lifecycle Node communicating via topics:

| Module | Input topics | Output topics |
|--------|-------------|---------------|
| arm_teleop | `/master/tracker/{left,right,waist}` | `/slave/arm/{left,right}/joint_cmd` |
| locomotion | `/master/tracker/{left,right}_foot` | `/slave/base/cmd_vel` |
| hand_teleop | `/master/hand/{left,right}/joints` | `/slave/hand/{left,right}/joint_cmd` |
| camera | RealSense RGB-D topics | Point cloud data |

### Multi-chain IK solver

`PinkIKSolver` (in `solvers/pink_ik_solver.py`) handles simultaneous left arm + right arm + torso IK using Pink's weighted FrameTasks. Per-chain weights are configured via `ChainConfig`. The QP solver is auto-detected at import time (`proxqp` > `osqp` > `daqp`).

### Coordinate conventions

All frame transforms are centralized in `teleop_system/utils/transforms.py` (pure numpy, no external dependency). Two conventions coexist:

- **ROS2/URDF**: X-forward, Y-left, Z-up, quaternion as `(x, y, z, w)`
- **SteamVR**: X-right, Y-up, Z-backward, quaternion as `(w, x, y, z)`

No other module should perform frame conversions directly.

### Configuration

YAML files in `config/` loaded via `utils/config_loader.py` (Hydra/OmegaConf). Structure:
- `default.yaml` — top-level system config
- `teleop/{arm,hand,locomotion}.yaml` — per-module parameters
- `hardware/{vive_tracker,manus_glove,rby1,dg5f}.yaml` — device configs
- `simulation/{mujoco,isaac_lab}.yaml` — simulator configs

### Hardware driver pattern

All device drivers in `devices/` use `try/except ImportError` to gracefully handle missing SDKs. When the SDK is absent, the driver still instantiates but reports `is_connected() == False`. This allows the full test suite to run without any hardware SDK installed.

## Key constraints

- `transforms3d` is listed in setup.py but **not actually used at runtime** — `utils/transforms.py` implements all rotation math with pure numpy. Remove it from dependencies when cleaning up.
- The `axes` parameter in `quat_to_euler`/`euler_to_quat` only supports `"sxyz"` convention.
- ROS2 nodes use lazy imports (`rclpy` imported inside class methods) so non-ROS2 tests pass without a ROS2 installation.
- Tests are organized by development phase (1-6). Phase 3+ tests require `pin` and `pink` packages.
