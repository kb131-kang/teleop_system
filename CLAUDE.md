# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RB-Y1 humanoid robot VR teleoperation system. Maps 5 Vive Trackers + 2 Manus Gloves to a dual-arm (2x7DoF) + torso (6DoF) + mobile base robot with DG-5F dexterous hands. Python 3.10+, ROS2 Jazzy, Pink/Pinocchio IK.

## Commands

### Running tests

```bash
python3 -m pytest tests/ -v                          # Full suite
python3 -m pytest tests/test_phase3_multichain.py -v  # Single file
python3 -m pytest tests/test_transforms.py::TestQuaternionOperations::test_multiply_identity -v  # Single test

# Isaac Sim Python (for Isaac Lab integration)
/workspace/isaaclab/_isaac_sim/python.sh -m pytest tests/ -v
```

Tests are organized by development phase: 1-2 need only numpy/scipy, phase 3+ requires `pin` and `pink`, phase 5+ needs `mujoco`. All 192 tests pass without ROS2 or hardware SDKs.

### Running the system

```bash
# Full simulation (slave + master) — ROS2 launch
ros2 launch teleop_system teleop_sim_full.launch.py
ros2 launch teleop_system teleop_sim_full.launch.py launch_viewer:=true publish_camera:=true launch_camera_viewer:=true

# Slave only / Master only
ros2 launch teleop_system slave_mujoco.launch.py launch_viewer:=true publish_camera:=true launch_streaming:=true
ros2 launch teleop_system master_sim.launch.py launch_viewer:=true

# Python scripts (no ros2 launch needed)
python3 scripts/launch_all.py --launch-viewer --publish-camera --launch-camera-viewer
python3 scripts/launch_slave.py --launch-viewer --publish-camera --launch-streaming
python3 scripts/launch_master.py --launch-viewer

# Standalone (no ROS2)
python3 scripts/run_teleop.py --mode simulation --sim-backend mujoco
```

### Camera / RGB-D streaming

```bash
# Same-machine viewer (subscribes to ROS2 camera topics)
python3 scripts/demo_rgbd_streaming.py --mode ros2-viewer

# Cross-machine: ROS2→TCP bridge on slave, TCP client on master
python3 scripts/demo_rgbd_streaming.py --mode ros2-server --host 0.0.0.0 --port 9876
python3 scripts/demo_rgbd_streaming.py --mode client --host <slave-ip> --port 9876 --ros2-sync

# Standalone (own MuJoCo instance, no teleop sync)
python3 scripts/demo_rgbd_streaming.py --mode server --host 0.0.0.0 --port 9999
```

## Architecture

### Interface-driven design (ABC pattern)

All hardware-dependent components are abstract base classes in `teleop_system/interfaces/`. Concrete implementations exist for both real hardware (`devices/`) and simulation (`simulators/`):

- **IMasterTracker** → ViveTracker, SimulatedTracker
- **IHandInput** → ManusGlove, SimulatedHand
- **ISlaveArm** → RBY1Arm
- **ISlaveHand** → DG5FHand
- **IMobileBase** → RBY1Base
- **IIKSolver** → PinkIKSolver, ProportionalMapper
- **ISimulator** → MuJoCoSimulator
- **ICameraStream** → RealSenseCamera, SimCameraStream, ROS2RGBDSubscriber, RGBDStreamClient

### Module structure pattern

Each of the four teleop modules follows a three-file pattern:

```
modules/<name>/
├── <name>_controller.py   # Pure Python logic — no ROS2 dep, independently testable
├── <name>_node.py         # ROS2 Lifecycle Node wrapping the controller
└── ros2_adapters.py       # ROS2 message ↔ interface type converters
```

Data flow (arm_teleop example):
```
DummyTrackerPub → PoseStamped → ArmTeleopNode → ArmController.update()
  → PinkIKSolver.solve_multi() → JointState → MuJoCoROS2Bridge → mujoco.mj_step()
```

### Module topic mapping

| Module | Input topics | Output topics |
|--------|-------------|---------------|
| arm_teleop | `/master/tracker/{left,right,waist}` | `/slave/arm/{left,right}/joint_cmd`, `/slave/torso/joint_cmd` |
| locomotion | `/master/tracker/{left,right}_foot` | `/slave/base/cmd_vel` |
| hand_teleop | `/master/hand/{left,right}/joints` | `/slave/hand/{left,right}/joint_cmd` |
| camera | `/master/hmd/orientation` | `/slave/camera/pan_tilt_cmd` |

### MuJoCo actuator mapping (model_teleop.xml — 26 actuators)

| ctrl index | Constant | Joints |
|-----------|----------|--------|
| 0 | `CTRL_LEFT_WHEEL` | Left wheel |
| 1 | `CTRL_RIGHT_WHEEL` | Right wheel |
| 2–7 | `CTRL_TORSO` | Torso (6 DOF) |
| 8–14 | `CTRL_RIGHT_ARM` | Right arm (7 DOF) |
| 15–21 | `CTRL_LEFT_ARM` | Left arm (7 DOF) |
| 22–23 | `CTRL_HEAD` | Head pan, tilt |
| 24 | `CTRL_RIGHT_GRIPPER` | Right gripper |
| 25 | `CTRL_LEFT_GRIPPER` | Left gripper |

These constants are defined in `teleop_system/simulators/mujoco_ros2_bridge.py`.

### Multi-chain IK solver

`PinkIKSolver` (in `solvers/pink_ik_solver.py`) handles simultaneous left arm + right arm + torso IK using Pink's weighted FrameTasks. Per-chain weights are configured via `ChainConfig`. The QP solver is auto-detected at import time (`proxqp` > `osqp` > `daqp`).

### Coordinate conventions

All frame transforms are centralized in `teleop_system/utils/transforms.py` (pure numpy, no external dependency). Two conventions coexist:

- **ROS2/URDF**: X-forward, Y-left, Z-up, quaternion as `(x, y, z, w)`
- **SteamVR**: X-right, Y-up, Z-backward, quaternion as `(w, x, y, z)`

No other module should perform frame conversions directly. `quat_to_euler`/`euler_to_quat` only support `"sxyz"` convention.

### Configuration

YAML files in `config/` loaded via `utils/config_loader.py` (Hydra/OmegaConf):
- `default.yaml` — top-level system config
- `teleop/{arm,hand,locomotion}.yaml` — per-module parameters
- `hardware/{vive_tracker,manus_glove,rby1,dg5f}.yaml` — device configs
- `simulation/{mujoco,isaac_lab}.yaml` — simulator configs

### Camera streaming architecture

Five modes in `scripts/demo_rgbd_streaming.py`:

| Mode | Description | Use case |
|------|-------------|----------|
| `server` | Standalone MuJoCo → TCP | Quick test (no teleop sync) |
| `client` | TCP viewer + optional `--ros2-sync` | Remote point cloud viewer |
| `local` | server + client in one process | Localhost testing |
| `ros2-viewer` | Subscribes to ROS2 camera topics | Same-machine viewing |
| `ros2-server` | ROS2 camera topics → TCP bridge | **Cross-machine with teleop sync** |

`ros2-server` is the production mode for cross-machine setups — it bridges the MuJoCo bridge's ROS2 camera topics to TCP, so there's only one MuJoCo instance and the camera reflects real robot motion. The standalone `server` mode creates its own MuJoCo instance that does **not** receive teleop commands.

## Key constraints

- `transforms3d` is listed in setup.py but **not used at runtime** — `utils/transforms.py` implements all rotation math with pure numpy.
- ROS2 nodes and all device drivers use lazy imports (`try: import rclpy` + `_ROS2_AVAILABLE` flag, `try: import sdk` + `_SDK_AVAILABLE` flag). Missing SDKs don't break imports; devices report `is_connected() == False`. This allows the full test suite to run without ROS2 or hardware SDKs.
- MuJoCo EGL rendering is thread-local. Camera capture must happen on the main thread (bridge uses single-threaded executor; streaming server uses `capture()` pattern on main thread).
- ROS2 `LaunchConfiguration` passes all values as strings. `mujoco_ros2_bridge.py` has `_get_float_param()`, `_get_bool_param()`, `_get_int_param()` helpers for type coercion. Use these when reading parameters from launch files.
- Topic names are centralized in `utils/ros2_helpers.py` `TopicNames` class — use constants, not hardcoded strings. QoS profiles are in `QoSPreset` enum: `SENSOR_DATA` (best effort, depth=1), `COMMAND` (reliable, depth=1), `STATUS` (transient local, depth=10).

## Documentation

- `docs/user_guide.md` — End-user guide (installation, running modes, camera streaming, troubleshooting)
- `docs/developer_guide.md` — Architecture, interfaces, adding modules, MuJoCo integration
- `docs/develop_plan.md` — **Ongoing development plan** (update at start of each task)
- `docs/develop_summary.md` — **Ongoing development log** (update after each task)
- `docs/debugging_log.md` — **Minor bug fixes and debugging notes** (lightweight log)
- `docs/PRD.md` — Product Requirements Document
- `docs/TRD.md` — Technical Requirements Document
- `docs/Tasks.md` — Task tracking

## Development Documentation Rules

**IMPORTANT: On every development task, you MUST automatically update these two documents:**

1. **`docs/develop_plan.md`** — Update at the **start** of a development task:
   - Add a new section at the top (reverse chronological order) with date and task title
   - Document the objective, planned components, planned files (with action: New/Modify), and implementation order
   - After the task is complete, add a "Result" line linking to the corresponding `develop_summary.md` entry

2. **`docs/develop_summary.md`** — Update at the **end** of a development task:
   - Add a new section at the top (reverse chronological order) with date and task title
   - Document: summary, new/modified files table, architecture diagrams (if applicable), configuration changes, test results, and usage examples

These updates are **mandatory** for any task that involves new features, significant refactoring, or architectural changes. Do not wait for the user to ask — update both documents as part of the standard workflow.

3. **`docs/debugging_log.md`** — For **minor fixes and debugging**:
   - Simple bug fixes, parameter tweaks, typo corrections, small patches to existing features
   - One-liner format: `[date] issue → cause → fix (files touched)`
   - Use this instead of develop_plan/summary when the change is small and doesn't introduce new functionality
   - When in doubt: if the fix touches ≤3 files and takes no architectural decisions, use debugging_log.md
