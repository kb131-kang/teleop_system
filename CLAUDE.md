# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RB-Y1 humanoid robot VR teleoperation system. Maps 5 Vive Trackers + 2 Manus Gloves to a dual-arm (2x7DoF) + torso (6DoF) + mobile base robot with DG-5F dexterous hands. Python 3.10+, ROS2 Jazzy, Pink/Pinocchio IK.

## Commands

### Running tests

```bash
python3 -m pytest tests/ -v                          # Full suite (270 tests)
python3 -m pytest tests/test_phase3_multichain.py -v  # Single file
python3 -m pytest tests/test_transforms.py::TestQuaternionOperations::test_multiply_identity -v  # Single test

# Isaac Sim Python (for Isaac Lab integration)
/workspace/isaaclab/_isaac_sim/python.sh -m pytest tests/ -v
```

Tests are organized by development phase: 1-2 need only numpy/scipy, phase 3+ requires `pin` and `pink`, phase 5+ needs `mujoco`. All 270 tests pass without ROS2 or hardware SDKs.

### Building with colcon

```bash
cd ~/ros2_ws
colcon build --packages-select teleop_system --symlink-install
source install/setup.bash
```

Use `--symlink-install` so asset files (models/, config/) resolve correctly from the source tree.

### Running the system

```bash
# Full simulation (slave + master) — ROS2 launch
ros2 launch teleop_system teleop_sim_full.launch.py
ros2 launch teleop_system teleop_sim_full.launch.py launch_viewer:=true publish_camera:=true launch_camera_viewer:=true

# Slave only / Master only
ros2 launch teleop_system slave_mujoco.launch.py launch_viewer:=true publish_camera:=true launch_streaming:=true
ros2 launch teleop_system master_sim.launch.py launch_viewer:=true

# BVH mocap replay (replaces live trackers with recorded motion)
ros2 launch teleop_system master_mocap.launch.py bvh_file:=/path/to/motion.bvh

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

### GUI architecture

**Framework:** Dear PyGui (DPG). Entry point: `gui_control_panel` console script → `teleop_system.gui.gui_node:main`.

**Threading model:**
- Dear PyGui runs on the **main thread** (required — DPG is not thread-safe)
- ROS2 MultiThreadedExecutor spins in a **background daemon thread**
- Shared data protected by `threading.Lock` on `ControlPanel` buffers
- ROS2 callbacks write to locked buffers; GUI reads via `render_frame()` on main thread

**Files:**
- `gui/control_panel.py` — All DPG widget creation, layout, and rendering logic. 5-tab interface: Status, Tracker View, Hand Data, Joint States, Parameters
- `gui/gui_node.py` — ROS2 node that subscribes to all topics and wires callbacks to the control panel

**Tracker View** uses drawlist canvases with isometric 3D projection (`project_3d_to_2d()`, elev=25°, azim=-60°) since DPG has no native 3D support.

### Calibration system

A-Pose calibration in `teleop_system/calibration/`:

- `pose_calibrator.py` — State machine: `IDLE → WAITING → CAPTURING → COMPUTING → CALIBRATED | ERROR`
- `calibration_node.py` — ROS2 node exposing `/teleop/calibrate` service (Trigger)
- Config: `config/calibration/a_pose_reference.yaml` (reference positions for 6 tracker points)

Process: 3s countdown → 1s capture at 100Hz → compute offsets (reference - captured) → store offsets.

### BVH mocap replay

`teleop_system/mocap/` provides BVH-file-driven motion replay as a drop-in replacement for live VR trackers:

| File | Role |
|------|------|
| `bvh_loader.py` | Parse BVH files (bvhio), Y-up → Z-up conversion |
| `skeleton_mapper.py` | Map BVH joints → TrackerRole Pose6D |
| `bvh_tracker_adapter.py` | IMasterTracker interface for BVH data |
| `bvh_hand_adapter.py` | IHandInput interface — generates finger motion from wrist angular velocity |
| `bvh_replay_publisher.py` | **Main ROS2 node** — publishes tracker/hand/HMD topics from BVH file |
| `data_recorder.py` | Records teleoperation data to disk |

**Staged playback:** When `auto_start=False` (default), the node holds at frame 0 (READY state), publishing initial pose. Call `/teleop/start_playback` service to begin advancing (PLAYING state). This allows the robot to align to init pose before motion starts.

### E-stop mechanism

Two-layer soft e-stop via `/system/estop_active` (Bool topic):

1. **GUI layer** (`gui_node.py`): publishes `Bool(True)` on e-stop/soft-stop, starts 50Hz timer publishing zero commands on all arm/hand/base topics
2. **Bridge layer** (`mujoco_ros2_bridge.py`): subscribes to `/system/estop_active`, ignores all incoming command callbacks when active, zeroes `ctrl[]` immediately

**Emergency Stop** vs **Soft Stop**: Emergency is immediate continuous zero publishing; Soft Stop ramps to zero over 0.5s (25 ticks). Both publish `estop_active=True`. Resume cancels the timer and publishes `estop_active=False`.

### ROS2 services

All service names in `utils/ros2_helpers.py` `ServiceNames`:

| Service | Type | Description |
|---------|------|-------------|
| `/teleop/set_mode` | SetMode | Switch simulation/hardware mode |
| `/teleop/enable_module` | EnableModule | Enable/disable individual modules |
| `/teleop/calibrate` | Trigger | Start A-Pose calibration |
| `/teleop/start_playback` | Trigger | Begin BVH replay playback |
| `/teleop/init_pose` | Trigger | Set robot to initial A-pose (smooth cosine interpolation, ~2s) |

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
- `simulation/{mujoco,isaac_lab}.yaml` — simulator configs (include `initial_pose` section)
- `mocap/{default,cmu_joint_mapping}.yaml` — BVH replay settings and joint mapping
- `calibration/a_pose_reference.yaml` — A-Pose calibration reference positions

### Camera streaming architecture

Five modes in `scripts/demo_rgbd_streaming.py`:

| Mode | Description | Use case |
|------|-------------|----------|
| `server` | Standalone MuJoCo → TCP | Quick test (no teleop sync) |
| `client` | TCP viewer + optional `--ros2-sync` | Remote point cloud viewer |
| `local` | server + client in one process | Localhost testing |
| `ros2-viewer` | Subscribes to ROS2 camera topics | Same-machine viewing |
| `ros2-server` | ROS2 camera topics → TCP bridge | **Cross-machine with teleop sync** |

`ros2-server` is the production mode for cross-machine setups — it bridges the MuJoCo bridge's ROS2 camera topics to TCP, so there's only one MuJoCo instance and the camera reflects real robot motion.

### Launch files

| File | Description |
|------|-------------|
| `teleop_sim.launch.py` | Base simulation (arm + hand + locomotion + MuJoCo bridge) |
| `teleop_sim_full.launch.py` | Full simulation including camera module |
| `teleop_full.launch.py` | Hardware teleoperation (real robot) |
| `teleop_mujoco_bridge.launch.py` | Standalone MuJoCo bridge |
| `master_sim.launch.py` | Master-only simulation (trackers + HMD + GUI) |
| `master_mocap.launch.py` | Master with BVH mocap replay |
| `slave_mujoco.launch.py` | Slave-only MuJoCo simulator |
| `arm_only.launch.py` | Arm teleoperation only |
| `hand_only.launch.py` | Hand teleoperation only |
| `mocap_replay.launch.py` | BVH playback + full teleop pipeline |

## Key constraints

- `transforms3d` is listed in setup.py but **not used at runtime** — `utils/transforms.py` implements all rotation math with pure numpy.
- ROS2 nodes and all device drivers use lazy imports (`try: import rclpy` + `_ROS2_AVAILABLE` flag, `try: import sdk` + `_SDK_AVAILABLE` flag). Missing SDKs don't break imports; devices report `is_connected() == False`. This allows the full test suite to run without ROS2 or hardware SDKs.
- MuJoCo EGL rendering is thread-local. Camera capture must happen on the main thread (bridge uses single-threaded executor; streaming server uses `capture()` pattern on main thread).
- ROS2 `LaunchConfiguration` passes all values as strings. `mujoco_ros2_bridge.py` has `_get_float_param()`, `_get_bool_param()`, `_get_int_param()` helpers for type coercion. All numeric/bool parameters use `ParameterDescriptor(dynamic_typing=True)` to accept int/double/string interchangeably.
- Topic names are centralized in `utils/ros2_helpers.py` `TopicNames` class — use constants, not hardcoded strings. QoS profiles are in `QoSPreset` enum: `SENSOR_DATA` (best effort, depth=1), `COMMAND` (reliable, depth=1), `STATUS` (transient local, depth=10).
- URDF path resolution uses 3-tier fallback: (1) `ament_index_python.get_package_share_directory`, (2) `__file__` parent traversal, (3) CWD. This handles both `colcon build` installed mode and source-tree execution.

## Documentation

- `docs/user_guide.md` — End-user guide (installation, running modes, camera streaming, troubleshooting)
- `docs/developer_guide.md` — Architecture, interfaces, adding modules, MuJoCo integration
- `docs/develop_plan.md` — **Ongoing development plan** (update at start of each task)
- `docs/develop_summary.md` — **Ongoing development log** (update after each task)
- `docs/debugging_log.md` — **Minor bug fixes and debugging notes** (lightweight log)
- `docs/hw_configuration_memo.md` — HW configuration memo for team meetings
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
