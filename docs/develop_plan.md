# Development Plan

This document tracks development plans for the RB-Y1 teleoperation system. Each session's plan is recorded in reverse chronological order (newest first). For implementation results, see `develop_summary.md`.

---

## 2026-02-01 (Session 6) — GUI Enhancements: E-Stop, Module Activity, Hand Viewer, 3-View Tracker

### Objective

Improve GUI control panel usability: fix non-functional Emergency Stop, add module activity tracking, add Hand Data tab, expand tracker view to 3-view, add joint states diagnostics, double window/font size.

### Changes

| File | Change |
|------|--------|
| `teleop_system/gui/control_panel.py` | HandData dataclass, 3-view tracker, hand tab, E-stop toggle with themes, module activity display, doubled window size |
| `teleop_system/gui/gui_node.py` | E-stop 50Hz timer, hand/base_vel subscriptions, module activity tracking |
| `docs/user_guide.md` | Font scaling, 5-tab descriptions, E-stop toggle docs |
| `docs/debugging_log.md` | 3 entries: E-stop, module status, window size |

### Result

**Completed.** 270/270 tests pass.

---

## 2026-02-01 (Session 5) — Debugging, Staged Playback, TCP Viewer, GUI Scaling

### Objective

Fix GUI crash (exit code -6), add staged BVH playback with Start Playback button, add cross-machine TCP viewer mode, add auto-scaling font/window sizing.

### Changes

| File | Change |
|------|--------|
| `teleop_system/gui/gui_node.py` | Fixed exit -6 (spin thread join), added playback/TCP viewer/font_scale params |
| `teleop_system/gui/control_panel.py` | Start Playback button, TCP viewer button, auto-scaling |
| `teleop_system/mocap/bvh_replay_publisher.py` | Staged start: auto_start, READY/PLAYING states |
| `launch/master_sim.launch.py` | viewer_mode/host/port, font_scale args |
| `launch/master_mocap.launch.py` | Same + auto_start arg |
| `scripts/launch_master.py`, `scripts/launch_master_mocap.py` | --viewer-mode/host/port flags |
| `teleop_system/simulators/mujoco_ros2_bridge.py` | ParameterDescriptor(dynamic_typing=True) |

### Result

**Completed.** 270/270 tests pass.

---

## 2026-02-01 (Session 4) — GUI Control Panel + Tracker Calibration

### Objective

Implement a ROS2 Node-based GUI control panel (Dear PyGui) with 4 tabs (Status, Tracker 3D View, Joint State Plots, Parameters), an A-Pose tracker calibration system, and update TRD.md architecture diagram.

### Planned Components

#### 1. Calibration System
- `teleop_system/calibration/pose_calibrator.py` — A-Pose calibration state machine (IDLE→WAITING→CAPTURING→COMPUTING→CALIBRATED|ERROR), computes position offsets per tracker
- `teleop_system/calibration/calibration_node.py` — ROS2 node with `/teleop/calibrate` service (std_srvs/Trigger), publishes state+offsets as JSON on `/calibration/state` and `/calibration/offsets`
- `config/calibration/a_pose_reference.yaml` — Robot A-Pose reference positions
- `tests/test_calibration.py` — 21 unit tests

#### 2. Enhanced GUI Control Panel
- `teleop_system/gui/control_panel.py` — Rewritten with 4-tab layout: Status (module indicators, calibration status, action buttons), Tracker View (XY+XZ scatter plots), Joint States (time series for left arm, right arm, torso), Parameters (sliders with bounds)
- `teleop_system/gui/gui_node.py` — ROS2 node wrapping ControlPanel. Main thread runs DearPyGui, background thread runs ROS2 executor. Subscribes to tracker/joint/calibration topics, provides calibrate/e-stop actions.

#### 3. Tracker Publisher Integration
- Added `/calibration/offsets` subscription to `dummy_tracker_pub.py`, `vive_tracker_pub.py`, `bvh_replay_publisher.py`
- Apply `position += offset` before publishing

#### 4. Launcher Updates
- Added `launch_gui` argument and `calibration_node` to `master_sim.launch.py`, `master_mocap.launch.py`, `teleop_sim_full.launch.py`
- Added `--no-gui` flag to `launch_master.py`, `launch_master_mocap.py`

### Planned Files

| File | Action | Description |
|------|--------|-------------|
| `teleop_system/calibration/__init__.py` | New | Package init |
| `teleop_system/calibration/pose_calibrator.py` | New | A-Pose calibration state machine |
| `teleop_system/calibration/calibration_node.py` | New | ROS2 calibration service/publisher |
| `config/calibration/a_pose_reference.yaml` | New | Robot A-Pose reference positions |
| `teleop_system/gui/gui_node.py` | New | ROS2 node wrapping ControlPanel |
| `teleop_system/gui/control_panel.py` | Modify | 4-tab layout with plots and actions |
| `teleop_system/simulators/dummy_tracker_pub.py` | Modify | Add calibration offset subscription |
| `teleop_system/mocap/bvh_replay_publisher.py` | Modify | Add calibration offset subscription |
| `teleop_system/devices/vive_tracker_pub.py` | Modify | Add calibration offset subscription |
| `teleop_system/utils/ros2_helpers.py` | Modify | Add calibration topic names |
| `setup.py` | Modify | Add entry points |
| `docs/TRD.md` | Modify | Update architecture diagram |
| `tests/test_calibration.py` | New | 21 calibration unit tests |

### Result

**Completed** — 21 new calibration tests pass, all existing tests pass with no regressions.

---

## 2026-02-01 (Session 3) — Mocap Master Launcher

### Objective

Create a master system launcher that uses BVH motion capture replay data instead of dummy sinusoidal inputs. This allows testing the teleop pipeline with real human motion data, with master and slave running on separate PCs.

### Planned Components

#### 1. ROS2 Launch File — `launch/master_mocap.launch.py`
- Same structure as `master_sim.launch.py`
- Replace 3 dummy publishers (tracker, glove, HMD) with single `bvh_replay_pub` node
- BVH parameters: bvh_file, rate_hz, playback_speed, loop, scale, normalize_mode
- Keep all teleop nodes (arm, locomotion, hand, camera) and optional viewer

#### 2. Python Script Launcher — `scripts/launch_master_mocap.py`
- Same structure as `scripts/launch_master.py`
- Replace 3 dummy publisher subprocesses with single `bvh_replay_publisher` subprocess
- Required `--bvh` argument, optional BVH tuning arguments

### Planned Files

| File | Action | Description |
|------|--------|-------------|
| `launch/master_mocap.launch.py` | New | ROS2 launch: BVH replay + teleop nodes |
| `scripts/launch_master_mocap.py` | New | Python script launcher for mocap master |
| `docs/develop_plan.md` | Modify | Add session entry |
| `docs/develop_summary.md` | Modify | Add session entry |
| `docs/user_guide.md` | Modify | Add mocap master usage section |
| `docs/debugging_log.md` | Modify | Add colcon rebuild note |

### Implementation Order

1. Create `launch/master_mocap.launch.py`
2. Create `scripts/launch_master_mocap.py`
3. Rebuild colcon workspace
4. Update documentation

### Result

**Completed.** All 249 tests pass (no regressions). See `develop_summary.md` Session 3 for details.

---

## 2025-02-01 (Session 2) — Motion Capture Dataset Replay Infrastructure

### Objective

Replace dummy sinusoidal inputs with real human motion capture data (BVH format) to test the teleop pipeline (arm IK, locomotion, hand retargeting) against recorded human motion. Provide tools for recording, analysis, visualization, and parameter tuning.

### Planned Components

#### 1. BVH Loading Infrastructure — `teleop_system/mocap/bvh_loader.py`
- Parse BVH files using `bvhio` library
- Extract per-frame joint positions and orientations from skeleton hierarchy
- Coordinate conversion: BVH Y-up → ROS2 Z-up
- Full load and lazy load (subset of joints) modes

#### 2. Skeleton → Tracker Mapping — `teleop_system/mocap/skeleton_mapper.py`
- Map 6 BVH joints (Hips, Head, LeftHand, RightHand, LeftFoot, RightFoot) to TrackerRole Pose6D
- Configurable joint name mapping via YAML (`config/mocap/cmu_joint_mapping.yaml`)
- Two normalization modes: `relative` (align first frame to robot workspace) and `absolute` (raw positions)

#### 3. BVH Adapters (IMasterTracker / IHandInput)
- `BVHTrackerAdapter(IMasterTracker)` — time-indexed BVH playback with loop/pause/speed controls
- `BVHHandAdapter(IHandInput)` — derive 20-joint finger motion from wrist angular velocity

#### 4. ROS2 Replay Publisher — `teleop_system/mocap/bvh_replay_publisher.py`
- `BVHReplayPub(Node)` replaces DummyTrackerPub + DummyGlovePub + DummyHMDPub
- Publishes on standard tracker/hand/HMD topics using TopicNames and QoSPreset.SENSOR_DATA

#### 5. Data Recording & Metrics
- `DataRecorder` — records named channels to `.npz` files
- `metrics.py` — tracking error, velocity saturation, workspace utilization, smoothness (jerk), report generation

#### 6. Visualization
- `skeleton_viewer.py` — Matplotlib 3D skeleton viewer with frame slider
- `dual_viewer.py` — side-by-side human skeleton + tracker position viewer

#### 7. Scripts
- `download_cmu_bvh.py` — download CMU BVH files from GitHub
- `run_mocap_replay.py` — standalone BVH replay + metrics (no ROS2)
- `run_parameter_sweep.py` — iterate scale/tuning parameters, output comparison table

#### 8. Launch & Config
- `launch/mocap_replay.launch.py` — ROS2 launch with optional MuJoCo bridge
- `config/mocap/default.yaml` — playback settings
- `config/mocap/cmu_joint_mapping.yaml` — CMU joint mapping + coordinate transform

### Planned Files

| File | Action | Description |
|------|--------|-------------|
| `teleop_system/mocap/__init__.py` | New | Package init |
| `teleop_system/mocap/bvh_loader.py` | New | BVH parser + coordinate conversion |
| `teleop_system/mocap/skeleton_mapper.py` | New | BVH joints → TrackerRole mapping |
| `teleop_system/mocap/bvh_tracker_adapter.py` | New | IMasterTracker for BVH playback |
| `teleop_system/mocap/bvh_hand_adapter.py` | New | IHandInput for BVH hand data |
| `teleop_system/mocap/bvh_replay_publisher.py` | New | ROS2 replay publisher node |
| `teleop_system/mocap/data_recorder.py` | New | Data recording to .npz |
| `teleop_system/mocap/metrics.py` | New | Tracking/workspace/smoothness metrics |
| `teleop_system/mocap/skeleton_viewer.py` | New | Matplotlib 3D skeleton viewer |
| `teleop_system/mocap/dual_viewer.py` | New | Side-by-side comparison viewer |
| `scripts/download_cmu_bvh.py` | New | CMU BVH downloader |
| `scripts/run_mocap_replay.py` | New | Standalone replay + metrics |
| `scripts/run_parameter_sweep.py` | New | Parameter sweep automation |
| `launch/mocap_replay.launch.py` | New | ROS2 launch for BVH replay |
| `config/mocap/default.yaml` | New | Playback settings |
| `config/mocap/cmu_joint_mapping.yaml` | New | CMU joint name mapping |
| `tests/test_bvh_loader.py` | New | BVH parser tests |
| `tests/test_skeleton_mapper.py` | New | Skeleton mapping tests |
| `tests/test_bvh_tracker_adapter.py` | New | Adapter interface tests |
| `tests/test_metrics.py` | New | Metrics computation tests |
| `setup.py` | Modify | Add mocap extras + entry points |
| `docs/develop_summary.md` | Modify | Add session entry |
| `docs/user_guide.md` | Modify | Add mocap testing section |
| `docs/developer_guide.md` | Modify | Add mocap architecture section |

### Implementation Order

1. Install `bvhio` dependency, download sample CMU BVH files
2. Implement `bvh_loader.py` + tests
3. Implement `skeleton_mapper.py` + tests
4. Implement adapters (`bvh_tracker_adapter.py`, `bvh_hand_adapter.py`) + tests
5. Implement `bvh_replay_publisher.py` (ROS2 node)
6. Implement `data_recorder.py` + `metrics.py` + tests
7. Implement visualization (`skeleton_viewer.py`, `dual_viewer.py`)
8. Implement standalone scripts + download script
9. Create launch file + config files
10. Update setup.py, docs

### Result

**Completed.** All 249 tests pass (192 existing + 57 new). See `develop_summary.md` Session 2 for details.

---

## 2025-02-01 (Session 1) — Bug Fixes, ros2-server Mode, Auto-Launch Features, Documentation

### Objective

Address 4 issues: parameter type crashes, camera sync in client/server mode, auto-launch support for viewer/streaming, and documentation reorganization.

### Planned Fixes

#### Issue 1: `camera_fps:=30` crashes bridge on `ros2 launch`
- Root cause: ROS2 `LaunchConfiguration` passes all values as strings; node tried `.double_value` on an integer-typed parameter
- Plan: Add `_get_float_param()`, `_get_bool_param()`, `_get_int_param()` type-coercion helpers to bridge

#### Issue 2: TCP client/server mode camera doesn't sync with robot motion
- Root cause: `--mode server` creates its own MuJoCo instance that doesn't receive teleop commands
- Plan: Add `--mode ros2-server` that bridges ROS2 camera topics to TCP (single MuJoCo instance)

#### Issue 3: Master should auto-launch viewer; slave should auto-launch streaming
- Root cause: No auto-launch support in launch files or scripts
- Plan: Add `launch_viewer`, `launch_streaming`, `launch_camera_viewer` args to launch files and scripts

#### Issue 4: PRD.md, TRD.md, Tasks.md should be in docs/
- Root cause: Files in project root
- Plan: Move to `docs/`

### Planned Files

| File | Action | Description |
|------|--------|-------------|
| `teleop_system/simulators/mujoco_ros2_bridge.py` | Modify | Add param type-coercion helpers |
| `scripts/demo_rgbd_streaming.py` | Modify | Add `ros2-server` mode |
| `launch/master_sim.launch.py` | Modify | Add `launch_viewer` arg |
| `launch/slave_mujoco.launch.py` | Modify | Add `launch_streaming` + `streaming_port` args |
| `launch/teleop_sim_full.launch.py` | Modify | Add `launch_camera_viewer` arg |
| `scripts/launch_master.py` | Modify | Add `--launch-viewer` flag |
| `scripts/launch_slave.py` | Modify | Add `--launch-streaming` + `--streaming-port` flags |
| `scripts/launch_all.py` | Modify | Add `--launch-camera-viewer` flag |
| `docs/user_guide.md` | Modify | Add ros2-server docs, camera mode guide |
| `docs/developer_guide.md` | Modify | Add ros2-server architecture section |

### Result

**Completed.** All 192 tests pass (no regressions). See `develop_summary.md` Session 1 for details.

---

## Previous Development (Phases 1–6)

See `develop_summary.md` "Previous Development" section and `CHANGE_LOG.md` for earlier phase plans.
