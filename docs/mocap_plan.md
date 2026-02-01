# Motion Capture Dataset Replay & Parameter Tuning Infrastructure

## Goal

Replace dummy sinusoidal input with real human motion capture data (BVH format) to test that the teleop pipeline (arm IK, locomotion, hand retargeting) correctly maps human motion to robot commands. Provide tools for recording, analysis, visualization, and parameter tuning.

---

## 1. BVH Loading Infrastructure

### `teleop_system/mocap/bvh_loader.py`

- Use `bvhio` library (pip installable) to parse BVH files
- Extract per-frame joint positions and orientations from the skeleton hierarchy
- Handle CMU BVH conventions: Z-up, centimeters, 120fps
- Convert to ROS2 convention: X-forward, Y-left, Z-up, meters, xyzw quaternion
- Return structured data: `BVHData` dataclass with frames, frame_rate, joint_names, per-frame transforms

---

## 2. Skeleton → Tracker Mapping

### `teleop_system/mocap/skeleton_mapper.py`

Map BVH skeleton joints to `TrackerRole` Pose6D data:

| TrackerRole | CMU BVH Joint | Data Extracted |
|-------------|---------------|----------------|
| RIGHT_HAND | RightHand | position + orientation |
| LEFT_HAND | LeftHand | position + orientation |
| WAIST | Hips | position + orientation |
| RIGHT_FOOT | RightFoot | position + orientation |
| LEFT_FOOT | LeftFoot | position + orientation |
| HEAD | Head | orientation only (for HMD) |

- Configurable joint name mapping via YAML (`config/mocap/cmu_joint_mapping.yaml`)
- Apply coordinate transform: CMU BVH (Z-up, cm) → ROS2 (Z-up, meters)
- Apply position offset to align skeleton center with robot workspace

---

## 3. BVH Tracker Adapter (implements IMasterTracker)

### `teleop_system/mocap/bvh_tracker_adapter.py`

- `BVHTrackerAdapter(IMasterTracker)` — time-indexed playback from loaded BVH data
- `get_pose()` returns `Pose6D` based on elapsed time since `initialize()`
- Supports: loop mode, playback speed multiplier, pause/resume
- Returns `Pose6D(valid=False)` when playback ends (non-loop mode)

### `teleop_system/mocap/bvh_hand_adapter.py`

- `BVHHandAdapter(IHandInput)` — extracts finger joint angles from BVH hand joints
- Maps BVH finger rotations → 20-element joint array matching `HandJointState`
- Note: CMU BVH has limited hand data; will generate approximate finger motion from wrist orientation

---

## 4. ROS2 Replay Publisher

### `teleop_system/mocap/bvh_replay_publisher.py`

- `BVHReplayPub(Node)` — replaces `DummyTrackerPub` for BVH playback
- Follows exact same pattern as dummy publishers
- Uses `TopicNames` and `QoSPreset.SENSOR_DATA` from `ros2_helpers.py`
- Parameters: `bvh_file`, `rate_hz`, `playback_speed`, `loop`

---

## 5. Data Recording & Metrics

### `teleop_system/mocap/data_recorder.py`

- `DataRecorder(Node)` — subscribes to input AND output topics, records to npz/csv
- Records: input poses (master tracker), output commands (slave joint_cmd, base cmd_vel), timestamps

### `teleop_system/mocap/metrics.py`

- `compute_tracking_error()` — position/orientation RMSE over time
- `compute_velocity_saturation()` — % of frames at velocity limit
- `compute_workspace_utilization()` — coverage analysis
- `compute_ik_success_rate()` — % of frames with valid IK solution
- `compute_smoothness()` — jerk metric for motion quality
- Summary report generation (dict/JSON output)

---

## 6. Visualization

### `teleop_system/mocap/skeleton_viewer.py`

- Matplotlib 3D skeleton visualization with frame-by-frame playback
- Draws BVH skeleton as stick figure with joint markers
- Shows tracker-mapped positions highlighted

### `teleop_system/mocap/dual_viewer.py`

- Side-by-side display: human skeleton (left) + robot state (right)
- Synchronized frame playback
- Overlays tracking error as color-coded markers

---

## 7. Scripts

### `scripts/download_cmu_bvh.py`
- Downloads selected CMU BVH files from GitHub (una-dinosauria/cmu-mocap)
- Recommended: walking (02_01, 07_01), arm reaching (49_02, 15_04), full body (86_02)

### `scripts/run_mocap_replay.py`
- Non-ROS2 standalone: BVH → trackers → controllers → metrics

### `scripts/run_parameter_sweep.py`
- Iterates over parameter combinations, outputs comparison table + plots

---

## 8. Configuration

### `config/mocap/default.yaml`
```yaml
mocap:
  data_dir: data/bvh/cmu
  default_file: 02/02_01.bvh
  playback_speed: 1.0
  loop: true
  rate_hz: 120
  position_offset: [0.0, 0.0, 0.0]
```

### `config/mocap/cmu_joint_mapping.yaml`
```yaml
joint_mapping:
  right_hand: RightHand
  left_hand: LeftHand
  waist: Hips
  right_foot: RightFoot
  left_foot: LeftFoot
  head: Head
coordinate_transform:
  scale: 0.01  # cm → meters
  axes: zup_to_ros2
```

---

## 9. Files Summary

| File | Action | Description |
|------|--------|-------------|
| `teleop_system/mocap/__init__.py` | New | Package init |
| `teleop_system/mocap/bvh_loader.py` | New | BVH file parser + coordinate conversion |
| `teleop_system/mocap/skeleton_mapper.py` | New | BVH joints → TrackerRole mapping |
| `teleop_system/mocap/bvh_tracker_adapter.py` | New | IMasterTracker impl for BVH playback |
| `teleop_system/mocap/bvh_hand_adapter.py` | New | IHandInput impl for BVH hand data |
| `teleop_system/mocap/bvh_replay_publisher.py` | New | ROS2 node publishing BVH data |
| `teleop_system/mocap/data_recorder.py` | New | ROS2 node recording input/output |
| `teleop_system/mocap/metrics.py` | New | Tracking error, velocity saturation, etc. |
| `teleop_system/mocap/skeleton_viewer.py` | New | Matplotlib 3D skeleton visualizer |
| `teleop_system/mocap/dual_viewer.py` | New | Side-by-side human+robot viewer |
| `scripts/download_cmu_bvh.py` | New | CMU BVH dataset downloader |
| `scripts/run_mocap_replay.py` | New | Standalone replay + metrics script |
| `scripts/run_parameter_sweep.py` | New | Parameter sweep automation |
| `launch/mocap_replay.launch.py` | New | ROS2 launch for BVH replay |
| `config/mocap/default.yaml` | New | Mocap playback config |
| `config/mocap/cmu_joint_mapping.yaml` | New | CMU joint name mapping |
| `tests/test_bvh_loader.py` | New | BVH parser tests |
| `tests/test_skeleton_mapper.py` | New | Skeleton mapping tests |
| `tests/test_bvh_tracker_adapter.py` | New | Adapter interface tests |
| `tests/test_metrics.py` | New | Metrics computation tests |
| `setup.py` | Modify | Add mocap extras + entry points |

## 10. Implementation Order

1. Install `bvhio` dependency, download sample CMU BVH files
2. Implement `bvh_loader.py` + `test_bvh_loader.py`
3. Implement `skeleton_mapper.py` + `test_skeleton_mapper.py`
4. Implement `bvh_tracker_adapter.py` + `bvh_hand_adapter.py` + tests
5. Implement `bvh_replay_publisher.py`
6. Implement `data_recorder.py` + `metrics.py` + `test_metrics.py`
7. Implement `skeleton_viewer.py` + `dual_viewer.py`
8. Implement standalone scripts + download script
9. Create launch file + config files
10. Update setup.py, docs
