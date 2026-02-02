"""GUI control panel using Dear PyGui.

Provides a tabbed control panel for teleoperation system monitoring and
configuration. Five tabs: Status, Tracker View, Hand Data, Joint States, Parameters.

Thread-safety: Dear PyGui calls must happen on the main thread only.
ROS2 callbacks write to shared buffers protected by threading.Lock.
The render_frame() method reads from those buffers on the main thread.
"""

from __future__ import annotations

import math
import subprocess
import sys
import threading
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from teleop_system.utils.logger import get_logger

logger = get_logger("control_panel")

try:
    import dearpygui.dearpygui as dpg
    _DPG_AVAILABLE = True
except ImportError:
    _DPG_AVAILABLE = False
    logger.warning("Dear PyGui not available — GUI control panel disabled")


# ---------------------------------------------------------------------------
# Data containers (written by ROS2 thread, read by GUI thread)
# ---------------------------------------------------------------------------

@dataclass
class ModuleStatus:
    """Shared state for a single module's GUI display."""
    name: str
    connected: bool = False
    enabled: bool = True
    error_msg: str = ""
    metrics: dict[str, float] = field(default_factory=dict)
    last_activity: float = 0.0  # monotonic timestamp of last message


@dataclass
class TrackerData:
    """Buffer for tracker position data."""
    positions: dict[str, np.ndarray] = field(default_factory=dict)
    offsets: dict[str, np.ndarray] = field(default_factory=dict)
    calibration_state: str = "IDLE"
    calibration_progress: float = 0.0
    calibration_countdown: float = 0.0
    calibration_error: str = ""


@dataclass
class HandData:
    """Buffer for hand/glove joint data."""
    left_joints: np.ndarray | None = None   # 20-DOF finger joints (input)
    right_joints: np.ndarray | None = None  # 20-DOF finger joints (input)
    left_cmd: np.ndarray | None = None      # commanded hand positions (output)
    right_cmd: np.ndarray | None = None     # commanded hand positions (output)


@dataclass
class JointStateData:
    """Buffer for joint state time series."""
    max_samples: int = 500  # ~5s at 100Hz
    # Deques of (timestamp, values) for rolling window
    left_arm_cmd: deque = field(default_factory=lambda: deque(maxlen=500))
    right_arm_cmd: deque = field(default_factory=lambda: deque(maxlen=500))
    torso_cmd: deque = field(default_factory=lambda: deque(maxlen=500))
    joint_states: deque = field(default_factory=lambda: deque(maxlen=500))


# ---------------------------------------------------------------------------
# Tracker display configuration
# ---------------------------------------------------------------------------

TRACKER_COLORS = {
    "right_hand": (255, 80, 80, 255),    # Red
    "left_hand": (80, 130, 255, 255),     # Blue
    "waist": (80, 220, 80, 255),          # Green
    "right_foot": (255, 180, 50, 255),    # Orange
    "left_foot": (255, 220, 80, 255),     # Yellow-orange
    "head": (200, 100, 255, 255),         # Purple
}

# Skeleton bone connections for dual viewer (simplified humanoid from trackers)
SKELETON_BONES = [
    ("head", "waist"),
    ("waist", "left_shoulder"),
    ("left_shoulder", "left_elbow"),
    ("left_elbow", "left_hand"),
    ("waist", "right_shoulder"),
    ("right_shoulder", "right_elbow"),
    ("right_elbow", "right_hand"),
    ("waist", "left_hip"),
    ("left_hip", "left_knee"),
    ("left_knee", "left_foot"),
    ("waist", "right_hip"),
    ("right_hip", "right_knee"),
    ("right_knee", "right_foot"),
]

# Bone colors for skeleton drawing
BONE_COLORS = {
    "spine": (200, 200, 200, 255),   # head-waist
    "arm": (100, 150, 255, 255),     # arm chain
    "leg": (100, 255, 150, 255),     # leg chain
}

# Bone category for coloring
BONE_CATEGORY = {}
for _p, _c in SKELETON_BONES:
    if _p == "head" or _c == "head":
        BONE_CATEGORY[(_p, _c)] = "spine"
    elif "shoulder" in _p or "elbow" in _p or "hand" in _p or \
         "shoulder" in _c or "elbow" in _c or "hand" in _c:
        BONE_CATEGORY[(_p, _c)] = "arm"
    elif "hip" in _p or "knee" in _p or "foot" in _p or \
         "hip" in _c or "knee" in _c or "foot" in _c:
        BONE_CATEGORY[(_p, _c)] = "leg"
    else:
        BONE_CATEGORY[(_p, _c)] = "spine"


def project_3d_to_2d(
    x: float, y: float, z: float,
    elev_deg: float = 25.0, azim_deg: float = -60.0,
) -> tuple[float, float]:
    """Project 3D ROS2 coords (X-fwd, Y-left, Z-up) to 2D screen coords.

    Uses an orthographic projection with elevation and azimuth rotation
    matching matplotlib's default 3D view angles.

    Returns:
        (screen_x, screen_y) where screen_y increases upward.
    """
    el = math.radians(elev_deg)
    az = math.radians(azim_deg)
    # Rotate by azimuth around Z axis
    rx = x * math.cos(az) + y * math.sin(az)
    ry = -x * math.sin(az) + y * math.cos(az)
    rz = z
    # Project with elevation
    sx = rx
    sy = -ry * math.sin(el) + rz * math.cos(el)
    return sx, sy


def interpolate_skeleton_joints(positions: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Compute intermediate skeleton joints from 6 tracker positions.

    Given head, waist, left/right hand, left/right foot positions,
    interpolates shoulder, elbow, hip, and knee positions.

    Args:
        positions: Dict mapping tracker role names to (3,) position arrays.

    Returns:
        Dict with all 14 joint positions (6 original + 8 interpolated).
    """
    result = {}
    # Copy original tracker positions
    for name in ("head", "waist", "left_hand", "right_hand", "left_foot", "right_foot"):
        if name in positions:
            result[name] = positions[name].copy()

    waist = positions.get("waist")
    head = positions.get("head")
    if waist is None:
        return result

    # Shoulder height: 65% from waist toward head
    shoulder_z = waist[2] + 0.65 * ((head[2] - waist[2]) if head is not None else 0.6)

    for side, hand_key, foot_key in [
        ("left", "left_hand", "left_foot"),
        ("right", "right_hand", "right_foot"),
    ]:
        hand = positions.get(hand_key)
        foot = positions.get(foot_key)

        # Shoulder: waist Y-shifted toward hand, at shoulder height
        if hand is not None:
            shoulder = np.array([
                waist[0],
                waist[1] + 0.3 * (hand[1] - waist[1]),
                shoulder_z,
            ])
            result[f"{side}_shoulder"] = shoulder
            # Elbow: midpoint between shoulder and hand
            result[f"{side}_elbow"] = 0.5 * (shoulder + hand)

        # Hip: just below waist, offset toward foot
        if foot is not None:
            hip = np.array([
                waist[0],
                waist[1] + 0.3 * (foot[1] - waist[1]),
                waist[2] - 0.05,
            ])
            result[f"{side}_hip"] = hip
            # Knee: midpoint between hip and foot
            result[f"{side}_knee"] = 0.5 * (hip + foot)

    return result


# ---------------------------------------------------------------------------
# Hand skeleton visualization constants
# ---------------------------------------------------------------------------

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]

FINGER_COLORS = {
    "thumb": (255, 80, 80, 255),     # Red
    "index": (255, 165, 0, 255),     # Orange
    "middle": (80, 200, 80, 255),    # Green
    "ring": (80, 130, 255, 255),     # Blue
    "pinky": (180, 80, 255, 255),    # Purple
}

# Finger MCP base positions (2D pixel coords in 400x450 canvas)
FINGER_BASES = {
    "thumb":  (100, 300),
    "index":  (120, 210),
    "middle": (175, 190),
    "ring":   (230, 200),
    "pinky":  (280, 220),
}

# Phalanx lengths in pixels: [MCP→PIP, PIP→DIP, DIP→tip]
BONE_LENGTHS = {
    "thumb":  [30, 25, 20],
    "index":  [40, 30, 20],
    "middle": [45, 30, 20],
    "ring":   [40, 28, 18],
    "pinky":  [35, 22, 15],
}

# Base pointing angle for each finger (degrees, 0=right, -90=up)
FINGER_BASE_ANGLES = {
    "thumb":  -60,
    "index":  -90,
    "middle": -88,
    "ring":   -86,
    "pinky":  -100,
}


def compute_finger_points(
    base_pos: tuple[float, float],
    base_angle_deg: float,
    bone_lengths: list[float],
    joint_angles: list[float],
) -> list[tuple[float, float]]:
    """Compute 2D finger joint positions via forward kinematics.

    Args:
        base_pos: (x, y) pixel position of MCP joint.
        base_angle_deg: Direction when fully extended (degrees).
        bone_lengths: [len1, len2, len3] for 3 phalanges.
        joint_angles: [MCP_spread, MCP_flex, PIP, DIP] in radians.

    Returns:
        List of 4 points: [MCP, PIP, DIP, tip].
    """
    points = [base_pos]
    # MCP spread has small rotational effect
    angle = math.radians(base_angle_deg) + joint_angles[0] * 0.3
    x, y = base_pos

    for length, flex in zip(bone_lengths, joint_angles[1:4]):
        angle += flex
        x += length * math.cos(angle)
        y += length * math.sin(angle)
        points.append((x, y))

    return points


# ---------------------------------------------------------------------------
# Parameter definitions
# ---------------------------------------------------------------------------

PARAMETER_DEFS = [
    ("position_scale", 1.0, 0.0, 5.0),
    ("orientation_scale", 1.0, 0.0, 5.0),
    ("max_joint_velocity", 2.0, 0.1, 10.0),
    ("ik_posture_cost", 0.1, 0.0, 1.0),
    ("hand_smoothing", 0.3, 0.0, 1.0),
    ("locomotion_deadzone", 0.02, 0.0, 0.2),
]


# ---------------------------------------------------------------------------
# Display scale detection
# ---------------------------------------------------------------------------

def _detect_display_scale() -> float:
    """Auto-detect display scale factor from screen resolution.

    Returns a font/UI scale factor (minimum 2.0 for readability):
        >= 3840px wide (4K)   -> 3.0
        >= 2560px wide (1440p) -> 2.5
        >= 1920px wide (1080p) -> 2.0
        otherwise              -> 2.0
    """
    try:
        import subprocess as _sp
        result = _sp.run(
            ["xrandr", "--current"],
            capture_output=True, text=True, timeout=2,
        )
        for line in result.stdout.splitlines():
            if "*" in line:
                res = line.split()[0]
                w = int(res.split("x")[0])
                if w >= 3840:
                    return 3.0
                elif w >= 2560:
                    return 2.5
                else:
                    return 2.0
    except Exception:
        pass
    return 2.0


# ---------------------------------------------------------------------------
# Main control panel class
# ---------------------------------------------------------------------------

class ControlPanel:
    """Dear PyGui-based control panel for the teleoperation system.

    Five-tab layout: Status | Tracker View | Hand Data | Joint States | Parameters
    """

    def __init__(
        self,
        title: str = "RB-Y1 Teleoperation Control Panel",
        width: int = 1800,
        height: int = 1000,
        font_scale: float | None = None,
    ):
        self._title = title
        self._font_scale = font_scale
        scale = font_scale or _detect_display_scale()
        self._width = max(width, int(1800 * (scale / 2.0)))
        self._height = max(height, int(1000 * (scale / 2.0)))
        self._running = False

        # Thread-safe shared data
        self._lock = threading.Lock()
        self._modules: dict[str, ModuleStatus] = {}
        self._tracker_data = TrackerData()
        self._joint_data = JointStateData()
        self._hand_data = HandData()

        # Parameters
        self._parameters: dict[str, float] = {
            name: default for name, default, _, _ in PARAMETER_DEFS
        }

        # Callbacks
        self._on_module_toggle: dict[str, callable] = {}
        self._on_parameter_change: dict[str, callable] = {}
        self._on_calibrate: callable | None = None
        self._on_emergency_stop: callable | None = None
        self._on_record_toggle: callable | None = None
        self._on_start_playback: callable | None = None
        self._on_init_pose: callable | None = None
        self._on_soft_stop: callable | None = None
        self._on_resume: callable | None = None

        # Internal state
        self._recording = False
        self._estop_active = False
        self._soft_stop_active = False
        self._playback_state = "UNKNOWN"
        self._viewer_process: subprocess.Popen | None = None
        self._tcp_viewer_process: subprocess.Popen | None = None
        self._module_enabled: dict[str, bool] = {}

        # TCP viewer config
        self._tcp_viewer_host: str = "localhost"
        self._tcp_viewer_port: int = 9876

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def lock(self) -> threading.Lock:
        return self._lock

    @property
    def tracker_data(self) -> TrackerData:
        return self._tracker_data

    @property
    def joint_data(self) -> JointStateData:
        return self._joint_data

    @property
    def hand_data(self) -> HandData:
        return self._hand_data

    @property
    def estop_active(self) -> bool:
        return self._estop_active

    @property
    def soft_stop_active(self) -> bool:
        return self._soft_stop_active

    # ------------------------------------------------------------------
    # Module registration and callback setters
    # ------------------------------------------------------------------

    def add_module(self, name: str, on_toggle: callable | None = None) -> ModuleStatus:
        status = ModuleStatus(name)
        self._modules[name] = status
        self._module_enabled[name] = True
        if on_toggle:
            self._on_module_toggle[name] = on_toggle
        return status

    def set_parameter_callback(self, param_name: str, callback: callable) -> None:
        self._on_parameter_change[param_name] = callback

    def set_calibrate_callback(self, callback: callable) -> None:
        self._on_calibrate = callback

    def set_emergency_stop_callback(self, callback: callable) -> None:
        self._on_emergency_stop = callback

    def set_record_toggle_callback(self, callback: callable) -> None:
        self._on_record_toggle = callback

    def set_start_playback_callback(self, callback: callable) -> None:
        self._on_start_playback = callback

    def set_init_pose_callback(self, callback: callable) -> None:
        self._on_init_pose = callback

    def set_soft_stop_callback(self, callback: callable) -> None:
        self._on_soft_stop = callback

    def set_resume_callback(self, callback: callable) -> None:
        self._on_resume = callback

    def set_tcp_viewer_config(self, host: str, port: int) -> None:
        self._tcp_viewer_host = host
        self._tcp_viewer_port = port

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def setup(self) -> bool:
        """Initialize Dear PyGui and create the GUI layout."""
        if not _DPG_AVAILABLE:
            logger.error(
                "Dear PyGui not installed. "
                "Install with: pip install dearpygui>=2.1.1"
            )
            return False

        dpg.create_context()

        scale = self._font_scale or _detect_display_scale()
        dpg.set_global_font_scale(scale)
        logger.info(f"GUI font scale: {scale:.1f}x (window: {self._width}x{self._height})")

        dpg.create_viewport(
            title=self._title, width=self._width, height=self._height,
        )

        with dpg.window(label="Control Panel", tag="main_window"):
            with dpg.tab_bar(tag="main_tabs"):
                self._build_status_tab()
                self._build_tracker_tab()
                self._build_hand_tab()
                self._build_joint_states_tab()
                self._build_parameters_tab()

        dpg.set_primary_window("main_window", True)
        dpg.setup_dearpygui()
        dpg.show_viewport()

        self._running = True
        logger.info("Control panel initialized")
        return True

    # ------------------------------------------------------------------
    # Tab builders
    # ------------------------------------------------------------------

    def _build_status_tab(self) -> None:
        with dpg.tab(label="Status", tag="tab_status"):
            # ── System mode ──
            dpg.add_text("System Mode", color=(200, 200, 255))
            dpg.add_separator()
            dpg.add_radio_button(
                items=["Simulation", "Hardware"],
                tag="mode_radio",
                horizontal=True,
            )
            dpg.add_spacer(height=8)

            # ── Module control (enable/disable checkboxes) ──
            dpg.add_text("Module Control", color=(200, 200, 255))
            dpg.add_separator()
            with dpg.group(horizontal=True):
                for name in self._modules:
                    dpg.add_checkbox(
                        label=name, tag=f"chk_module_{name}",
                        default_value=True,
                        callback=self._module_enable_cb,
                        user_data=name,
                    )
                    dpg.add_spacer(width=15)
            dpg.add_spacer(height=8)

            # ── Module status indicators ──
            dpg.add_text("Module Status", color=(200, 200, 255))
            dpg.add_separator()
            for name in self._modules:
                with dpg.group(horizontal=True):
                    dpg.add_text(
                        "●", tag=f"status_{name}", color=(128, 128, 128),
                    )
                    dpg.add_text(f" {name}", tag=f"label_{name}")
                    dpg.add_text(
                        " (no data)", tag=f"activity_{name}",
                        color=(128, 128, 128),
                    )
            dpg.add_spacer(height=8)

            # ── Calibration status ──
            dpg.add_text("Calibration", color=(200, 200, 255))
            dpg.add_separator()
            with dpg.group(horizontal=True):
                dpg.add_text("State:", color=(180, 180, 180))
                dpg.add_text("IDLE", tag="cal_state_text")
            dpg.add_progress_bar(
                tag="cal_progress", default_value=0.0, overlay="0%",
            )
            dpg.add_text("", tag="cal_error_text", color=(255, 80, 80))
            dpg.add_spacer(height=8)

            # ── Playback status ──
            dpg.add_text("Playback", color=(200, 200, 255))
            dpg.add_separator()
            with dpg.group(horizontal=True):
                dpg.add_text("State:", color=(180, 180, 180))
                dpg.add_text("UNKNOWN", tag="playback_state_text")
            dpg.add_spacer(height=8)

            # ── Teleop Workflow (ordered buttons) ──
            dpg.add_text("Teleop Workflow", color=(200, 200, 255))
            dpg.add_separator()
            with dpg.group(horizontal=True):
                dpg.add_button(
                    label="1. Init Pose", tag="btn_init_pose",
                    callback=self._init_pose_cb, width=140,
                )
                dpg.add_button(
                    label="2. Calibrate (A-Pose)", tag="btn_calibrate",
                    callback=self._calibrate_cb, width=180,
                )
                dpg.add_button(
                    label="3. Start Teleop", tag="btn_start_playback",
                    callback=self._start_playback_cb, width=150,
                )
            dpg.add_spacer(height=5)

            # ── Operation control ──
            dpg.add_text("Operation Control", color=(200, 200, 255))
            dpg.add_separator()
            with dpg.group(horizontal=True):
                dpg.add_button(
                    label="Soft Stop", tag="btn_soft_stop",
                    callback=self._soft_stop_cb, width=120,
                )
                dpg.add_button(
                    label="Resume", tag="btn_resume",
                    callback=self._resume_cb, width=120,
                )
                dpg.add_button(
                    label="Return to Init", tag="btn_return_init",
                    callback=self._init_pose_cb, width=140,
                )
            dpg.add_spacer(height=5)

            # ── Emergency Stop ──
            with dpg.theme(tag="estop_theme_inactive"):
                with dpg.theme_component(dpg.mvButton):
                    dpg.add_theme_color(dpg.mvThemeCol_Button, (180, 30, 30))
                    dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (220, 50, 50))
                    dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255))
            with dpg.theme(tag="estop_theme_active"):
                with dpg.theme_component(dpg.mvButton):
                    dpg.add_theme_color(dpg.mvThemeCol_Button, (255, 0, 0))
                    dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 60, 60))
                    dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 0))

            dpg.add_button(
                label="[ EMERGENCY STOP ]", tag="btn_estop",
                callback=self._estop_cb,
                width=-1, height=50,
            )
            dpg.bind_item_theme("btn_estop", "estop_theme_inactive")
            dpg.add_text("", tag="estop_status_text", color=(255, 80, 80))
            dpg.add_spacer(height=5)

            # ── Tools ──
            dpg.add_text("Tools", color=(200, 200, 255))
            dpg.add_separator()
            with dpg.group(horizontal=True):
                dpg.add_button(
                    label="Record: OFF", tag="btn_record",
                    callback=self._record_cb, width=120,
                )
                dpg.add_button(
                    label="Viewer (ROS2)", tag="btn_rgbd_viewer",
                    callback=self._launch_viewer_cb, width=130,
                )
            with dpg.group(horizontal=True):
                dpg.add_button(
                    label="Viewer (TCP)", tag="btn_tcp_viewer",
                    callback=self._launch_tcp_viewer_cb, width=120,
                )
                dpg.add_input_text(
                    tag="tcp_viewer_host",
                    default_value=self._tcp_viewer_host,
                    width=180,
                    hint="slave IP (e.g. 192.168.0.100)",
                    callback=self._tcp_host_changed_cb,
                    on_enter=True,
                )
                dpg.add_text(":", color=(180, 180, 180))
                dpg.add_input_int(
                    tag="tcp_viewer_port",
                    default_value=self._tcp_viewer_port,
                    width=80, min_value=1024, max_value=65535,
                    min_clamped=True, max_clamped=True,
                )

            dpg.add_spacer(height=8)

            # ── Metrics ──
            dpg.add_text("Metrics", color=(200, 200, 255))
            dpg.add_separator()
            dpg.add_text("", tag="metrics_text")

    def _build_tracker_tab(self) -> None:
        """Dual 3D viewer: skeleton + tracker positions (isometric projection)."""
        with dpg.tab(label="Tracker View", tag="tab_tracker"):
            dpg.add_text(
                "Dual 3D View — Skeleton + Tracker Positions (live)",
                color=(200, 200, 255),
            )
            dpg.add_separator()

            # Legend
            with dpg.group(horizontal=True):
                for name, color in TRACKER_COLORS.items():
                    dpg.add_text(f"● {name}", color=color[:3])
                    dpg.add_spacer(width=15)

            dpg.add_spacer(height=5)

            canvas_w, canvas_h = 700, 550
            with dpg.group(horizontal=True):
                # Left: Skeleton 3D view
                with dpg.group():
                    dpg.add_text("Skeleton (3D)", color=(180, 220, 255))
                    dpg.add_drawlist(
                        tag="tracker_canvas_skel",
                        width=canvas_w, height=canvas_h,
                    )
                dpg.add_spacer(width=20)
                # Right: Tracker 3D view
                with dpg.group():
                    dpg.add_text("Trackers (3D)", color=(255, 200, 180))
                    dpg.add_drawlist(
                        tag="tracker_canvas_pts",
                        width=canvas_w, height=canvas_h,
                    )

            # Draw initial empty state
            self._draw_tracker_3d("tracker_canvas_skel", {}, draw_skeleton=True)
            self._draw_tracker_3d("tracker_canvas_pts", {}, draw_skeleton=False)

    def _build_hand_tab(self) -> None:
        """Hand skeleton visualization + bar charts."""
        with dpg.tab(label="Hand Data", tag="tab_hand"):
            dpg.add_text(
                "Hand Skeleton Visualization (live)", color=(200, 200, 255),
            )
            dpg.add_separator()

            # Finger color legend
            with dpg.group(horizontal=True):
                for finger, color in FINGER_COLORS.items():
                    dpg.add_text(f"● {finger}", color=color[:3])
                    dpg.add_spacer(width=10)

            dpg.add_spacer(height=5)

            canvas_w, canvas_h = 400, 450
            with dpg.group(horizontal=True):
                # Left hand
                with dpg.group():
                    dpg.add_text("Left Hand (Input)", color=(180, 220, 255))
                    dpg.add_drawlist(
                        tag="hand_canvas_left",
                        width=canvas_w, height=canvas_h,
                    )
                dpg.add_spacer(width=30)
                # Right hand
                with dpg.group():
                    dpg.add_text("Right Hand (Input)", color=(255, 200, 180))
                    dpg.add_drawlist(
                        tag="hand_canvas_right",
                        width=canvas_w, height=canvas_h,
                    )

            # Draw initial empty hand outlines
            self._draw_hand_skeleton("hand_canvas_left", None, mirror=True)
            self._draw_hand_skeleton("hand_canvas_right", None, mirror=False)

            dpg.add_spacer(height=10)
            dpg.add_text("Joint Angles (Bar Chart)", color=(200, 200, 255))
            dpg.add_separator()

            with dpg.group(horizontal=True):
                with dpg.plot(
                    label="Left Hand Angles", tag="plot_hand_left",
                    width=750, height=200,
                ):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, label="Joint", tag="hand_left_x")
                    dpg.add_plot_axis(dpg.mvYAxis, label="Angle (rad)", tag="hand_left_y")
                    dpg.set_axis_limits("hand_left_y", -0.1, 2.0)
                    dpg.add_bar_series(
                        list(range(20)), [0.0] * 20,
                        label="joint angles", tag="series_hand_left",
                        parent="hand_left_y", weight=0.6,
                    )

                with dpg.plot(
                    label="Right Hand Angles", tag="plot_hand_right",
                    width=750, height=200,
                ):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, label="Joint", tag="hand_right_x")
                    dpg.add_plot_axis(dpg.mvYAxis, label="Angle (rad)", tag="hand_right_y")
                    dpg.set_axis_limits("hand_right_y", -0.1, 2.0)
                    dpg.add_bar_series(
                        list(range(20)), [0.0] * 20,
                        label="joint angles", tag="series_hand_right",
                        parent="hand_right_y", weight=0.6,
                    )

            dpg.add_spacer(height=5)
            dpg.add_text(
                "Waiting for hand data... "
                "(Hand publisher must be running: dummy_glove_pub or BVH replay)",
                tag="hand_status_text", color=(180, 180, 100),
            )

    def _build_joint_states_tab(self) -> None:
        with dpg.tab(label="Joint States", tag="tab_joints"):
            dpg.add_text("Joint Position Time Series", color=(200, 200, 255))
            dpg.add_separator()

            for group_name, group_label, n_joints in [
                ("left_arm", "Left Arm Joints", 7),
                ("right_arm", "Right Arm Joints", 7),
                ("torso", "Torso Joints", 6),
            ]:
                with dpg.plot(
                    label=group_label, tag=f"plot_{group_name}",
                    width=-1, height=180,
                ):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(
                        dpg.mvXAxis, label="Time (s)", tag=f"{group_name}_x",
                    )
                    dpg.add_plot_axis(
                        dpg.mvYAxis, label="Position (rad)", tag=f"{group_name}_y",
                    )
                    for i in range(n_joints):
                        dpg.add_line_series(
                            [], [], label=f"J{i}",
                            tag=f"series_{group_name}_{i}",
                            parent=f"{group_name}_y",
                        )
                dpg.add_spacer(height=5)

            dpg.add_text(
                "Waiting for joint data... "
                "(Arm teleop + slave MuJoCo bridge must be running)",
                tag="joint_status_text", color=(180, 180, 100),
            )

    def _build_parameters_tab(self) -> None:
        with dpg.tab(label="Parameters", tag="tab_params"):
            dpg.add_text("Teleop Parameters", color=(200, 200, 255))
            dpg.add_separator()

            for name, default, min_val, max_val in PARAMETER_DEFS:
                label = name.replace("_", " ").title()
                dpg.add_slider_float(
                    label=label, tag=f"param_{name}",
                    default_value=default,
                    min_value=min_val, max_value=max_val,
                    callback=self._parameter_change_cb,
                    user_data=name, width=400,
                )

    # ------------------------------------------------------------------
    # Hand skeleton drawing
    # ------------------------------------------------------------------

    def _draw_hand_skeleton(
        self,
        canvas_tag: str,
        joints: np.ndarray | None,
        mirror: bool,
    ) -> None:
        """Draw (or redraw) hand skeleton on a drawlist canvas.

        Args:
            canvas_tag: DearPyGui drawlist tag.
            joints: 20-DOF joint angles, or None for default (all zeros).
            mirror: If True, mirror horizontally (for left hand).
        """
        if not _DPG_AVAILABLE:
            return

        dpg.delete_item(canvas_tag, children_only=True)
        canvas_w = 400

        # Background
        dpg.draw_rectangle(
            (0, 0), (canvas_w, 450),
            color=(40, 40, 40, 255), fill=(30, 30, 30, 255),
            parent=canvas_tag,
        )

        # Palm polygon
        palm_pts = [FINGER_BASES[f] for f in FINGER_NAMES]
        wrist = (190, 380)
        palm_pts_full = palm_pts + [wrist]
        if mirror:
            palm_pts_full = [(canvas_w - x, y) for x, y in palm_pts_full]
        dpg.draw_polygon(
            palm_pts_full, color=(120, 120, 120, 200),
            fill=(60, 60, 60, 120), parent=canvas_tag,
        )

        # Default joint angles if none provided
        if joints is None:
            angles = [0.0] * 20
        else:
            angles = joints.tolist() if hasattr(joints, 'tolist') else list(joints)
            # Pad to 20 if shorter
            angles = (angles + [0.0] * 20)[:20]

        # Draw each finger
        for i, finger in enumerate(FINGER_NAMES):
            color = FINGER_COLORS[finger]
            base = FINGER_BASES[finger]
            if mirror:
                base = (canvas_w - base[0], base[1])

            finger_joints = angles[i * 4: (i + 1) * 4]
            base_angle = FINGER_BASE_ANGLES[finger]
            if mirror:
                base_angle = -(base_angle + 180)

            points = compute_finger_points(
                base, base_angle, BONE_LENGTHS[finger], finger_joints,
            )

            # Draw bones
            for j in range(len(points) - 1):
                dpg.draw_line(
                    points[j], points[j + 1],
                    color=color, thickness=3, parent=canvas_tag,
                )

            # Draw joints
            for j, pt in enumerate(points):
                radius = 6 if j == 0 else 4
                dpg.draw_circle(
                    pt, radius, color=color, fill=color,
                    parent=canvas_tag,
                )

        # Label
        label = "LEFT" if mirror else "RIGHT"
        dpg.draw_text(
            (canvas_w // 2 - 20, 10), label,
            color=(200, 200, 200, 200), size=16,
            parent=canvas_tag,
        )

    # ------------------------------------------------------------------
    # Render loop
    # ------------------------------------------------------------------

    def render_frame(self) -> bool:
        """Render one GUI frame. Call on main thread in a loop.

        Returns:
            False if the window was closed.
        """
        if not _DPG_AVAILABLE or not self._running:
            return False

        if not dpg.is_dearpygui_running():
            self._running = False
            return False

        with self._lock:
            self._update_status_tab()
            self._update_tracker_tab()
            self._update_hand_tab()
            self._update_joint_states_tab()

        dpg.render_dearpygui_frame()
        return True

    def _update_status_tab(self) -> None:
        import time as _time
        now = _time.monotonic()

        # Module status indicators
        for name, status in self._modules.items():
            age = now - status.last_activity if status.last_activity > 0 else float("inf")
            if age < 2.0:
                color = (0, 255, 0)
                activity_text = f" (active, {age:.1f}s ago)"
                activity_color = (0, 255, 0)
            elif age < 10.0:
                color = (255, 255, 0)
                activity_text = f" (stale, {age:.0f}s ago)"
                activity_color = (255, 255, 0)
            elif status.last_activity > 0:
                color = (255, 100, 0)
                activity_text = f" (no data for {age:.0f}s)"
                activity_color = (255, 100, 0)
            else:
                color = (128, 128, 128)
                activity_text = " (no data)"
                activity_color = (128, 128, 128)
            dpg.configure_item(f"status_{name}", color=color)
            dpg.set_value(f"activity_{name}", activity_text)
            dpg.configure_item(f"activity_{name}", color=activity_color)

        # Calibration status
        td = self._tracker_data
        dpg.set_value("cal_state_text", td.calibration_state)
        state_colors = {
            "IDLE": (180, 180, 180),
            "WAITING": (255, 255, 100),
            "CAPTURING": (100, 200, 255),
            "COMPUTING": (100, 200, 255),
            "CALIBRATED": (80, 255, 80),
            "ERROR": (255, 80, 80),
        }
        dpg.configure_item(
            "cal_state_text",
            color=state_colors.get(td.calibration_state, (180, 180, 180)),
        )

        progress = td.calibration_progress
        overlay = f"{int(progress * 100)}%"
        if td.calibration_state == "WAITING" and td.calibration_countdown > 0:
            overlay = f"Countdown: {td.calibration_countdown:.0f}s"
        dpg.set_value("cal_progress", progress)
        dpg.configure_item("cal_progress", overlay=overlay)
        dpg.set_value("cal_error_text", td.calibration_error)

        # Emergency stop status
        if self._estop_active:
            dpg.set_value(
                "estop_status_text",
                "E-STOP ACTIVE — all commands zeroed. Click again to release.",
            )
            dpg.configure_item("btn_estop", label="[ RELEASE E-STOP ]")
            dpg.bind_item_theme("btn_estop", "estop_theme_active")
        else:
            dpg.set_value("estop_status_text", "")
            dpg.configure_item("btn_estop", label="[ EMERGENCY STOP ]")
            dpg.bind_item_theme("btn_estop", "estop_theme_inactive")

        # Playback state
        dpg.set_value("playback_state_text", self._playback_state)
        pb_colors = {
            "READY": (255, 200, 80),
            "PLAYING": (80, 255, 80),
            "UNKNOWN": (180, 180, 180),
        }
        dpg.configure_item(
            "playback_state_text",
            color=pb_colors.get(self._playback_state, (180, 180, 180)),
        )
        if self._playback_state == "PLAYING":
            dpg.configure_item("btn_start_playback", enabled=False)
        elif self._playback_state == "READY":
            dpg.configure_item("btn_start_playback", enabled=True)

        # Metrics
        metrics_lines = []
        for name, status in self._modules.items():
            for key, val in status.metrics.items():
                metrics_lines.append(f"  {name}/{key}: {val:.4f}")
        dpg.set_value(
            "metrics_text",
            "\n".join(metrics_lines) if metrics_lines else "(no metrics)",
        )

    def _update_tracker_tab(self) -> None:
        td = self._tracker_data
        if "waist" not in td.positions:
            return

        self._draw_tracker_3d("tracker_canvas_skel", td.positions, draw_skeleton=True)
        self._draw_tracker_3d("tracker_canvas_pts", td.positions, draw_skeleton=False)

    def _draw_tracker_3d(
        self, canvas_tag: str, positions: dict, draw_skeleton: bool = True
    ) -> None:
        """Draw 3D-projected skeleton or tracker view in a drawlist canvas."""
        if not _DPG_AVAILABLE:
            return

        dpg.delete_item(canvas_tag, children_only=True)

        # Canvas dimensions
        cw = dpg.get_item_width(canvas_tag) or 700
        ch = dpg.get_item_height(canvas_tag) or 550

        # Background
        dpg.draw_rectangle(
            [0, 0], [cw, ch], fill=(25, 25, 35, 255),
            color=(60, 60, 80, 255), parent=canvas_tag,
        )

        # 3D-to-pixel mapping parameters
        # World range: X ~[-1, 1.5], Y ~[-1, 1], Z ~[0, 2]
        scale = min(cw, ch) * 0.35  # pixels per meter
        cx, cy = cw * 0.5, ch * 0.6  # canvas center (lower to show ground)

        def to_px(x, y, z):
            sx, sy = project_3d_to_2d(x, y, z)
            px = cx + sx * scale
            py = cy - sy * scale  # flip Y for screen coords
            return px, py

        # Draw ground grid
        grid_color = (50, 50, 70, 120)
        for gx in np.arange(-1.5, 2.0, 0.5):
            p1 = to_px(gx, -1.5, 0)
            p2 = to_px(gx, 1.5, 0)
            dpg.draw_line(p1, p2, color=grid_color, thickness=1, parent=canvas_tag)
        for gy in np.arange(-1.5, 2.0, 0.5):
            p1 = to_px(-1.5, gy, 0)
            p2 = to_px(1.5, gy, 0)
            dpg.draw_line(p1, p2, color=grid_color, thickness=1, parent=canvas_tag)

        # Draw axes at origin
        origin = to_px(0, 0, 0)
        ax_len = 0.3
        ax_x = to_px(ax_len, 0, 0)
        ax_y = to_px(0, ax_len, 0)
        ax_z = to_px(0, 0, ax_len)
        dpg.draw_line(origin, ax_x, color=(255, 80, 80, 200), thickness=2, parent=canvas_tag)
        dpg.draw_line(origin, ax_y, color=(80, 255, 80, 200), thickness=2, parent=canvas_tag)
        dpg.draw_line(origin, ax_z, color=(80, 80, 255, 200), thickness=2, parent=canvas_tag)
        dpg.draw_text([ax_x[0]+3, ax_x[1]-5], "X", color=(255, 80, 80), size=12, parent=canvas_tag)
        dpg.draw_text([ax_y[0]+3, ax_y[1]-5], "Y", color=(80, 255, 80), size=12, parent=canvas_tag)
        dpg.draw_text([ax_z[0]+3, ax_z[1]-5], "Z", color=(80, 80, 255), size=12, parent=canvas_tag)

        if not positions:
            dpg.draw_text(
                [cw//2 - 60, ch//2], "Waiting for data...",
                color=(150, 150, 150), size=16, parent=canvas_tag,
            )
            return

        if draw_skeleton:
            # Interpolate joints and draw skeleton
            all_joints = interpolate_skeleton_joints(positions)

            # Draw bones
            for parent, child in SKELETON_BONES:
                p1 = all_joints.get(parent)
                p2 = all_joints.get(child)
                if p1 is not None and p2 is not None:
                    px1 = to_px(float(p1[0]), float(p1[1]), float(p1[2]))
                    px2 = to_px(float(p2[0]), float(p2[1]), float(p2[2]))
                    cat = BONE_CATEGORY.get((parent, child), "spine")
                    color = BONE_COLORS.get(cat, (200, 200, 200, 255))
                    # Make it semi-transparent for 3D feel
                    draw_color = (color[0], color[1], color[2], 180)
                    dpg.draw_line(
                        px1, px2, color=draw_color, thickness=3, parent=canvas_tag,
                    )

            # Draw joint circles (interpolated joints: small, tracker joints: larger)
            tracker_names = set(TRACKER_COLORS.keys())
            for jname, pos in all_joints.items():
                px = to_px(float(pos[0]), float(pos[1]), float(pos[2]))
                if jname in tracker_names:
                    color = TRACKER_COLORS.get(jname, (200, 200, 200, 255))
                    dpg.draw_circle(
                        px, 7, color=color, fill=color, parent=canvas_tag,
                    )
                else:
                    dpg.draw_circle(
                        px, 4, color=(150, 150, 200, 200),
                        fill=(100, 100, 160, 150), parent=canvas_tag,
                    )

            # Labels for tracker joints
            for name in tracker_names:
                pos = positions.get(name)
                if pos is not None:
                    px = to_px(float(pos[0]), float(pos[1]), float(pos[2]))
                    color = TRACKER_COLORS.get(name, (200, 200, 200, 255))
                    dpg.draw_text(
                        [px[0] + 10, px[1] - 6], name,
                        color=color[:3], size=11, parent=canvas_tag,
                    )
        else:
            # Tracker-only view: larger markers with labels
            for name, color in TRACKER_COLORS.items():
                pos = positions.get(name)
                if pos is None:
                    continue
                px = to_px(float(pos[0]), float(pos[1]), float(pos[2]))
                # Outer ring (black edge)
                dpg.draw_circle(
                    px, 12, color=(0, 0, 0, 255), thickness=2, parent=canvas_tag,
                )
                # Filled circle
                dpg.draw_circle(
                    px, 10, color=color, fill=color, parent=canvas_tag,
                )
                # Label
                dpg.draw_text(
                    [px[0] + 15, px[1] - 6], name,
                    color=color[:3], size=13, parent=canvas_tag,
                )
                # Position text
                dpg.draw_text(
                    [px[0] + 15, px[1] + 8],
                    f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
                    color=(180, 180, 180), size=10, parent=canvas_tag,
                )

    def _update_hand_tab(self) -> None:
        hd = self._hand_data
        has_data = False

        if hd.left_joints is not None:
            self._draw_hand_skeleton("hand_canvas_left", hd.left_joints, mirror=True)
            vals = hd.left_joints.tolist()
            dpg.set_value("series_hand_left", [list(range(len(vals))), vals])
            has_data = True

        if hd.right_joints is not None:
            self._draw_hand_skeleton("hand_canvas_right", hd.right_joints, mirror=False)
            vals = hd.right_joints.tolist()
            dpg.set_value("series_hand_right", [list(range(len(vals))), vals])
            has_data = True

        if has_data:
            dpg.set_value("hand_status_text", "Receiving hand data")
            dpg.configure_item("hand_status_text", color=(80, 255, 80))

    def _update_joint_states_tab(self) -> None:
        jd = self._joint_data
        has_data = bool(jd.left_arm_cmd or jd.right_arm_cmd or jd.torso_cmd)

        if has_data:
            dpg.set_value("joint_status_text", "Receiving joint data")
            dpg.configure_item("joint_status_text", color=(80, 255, 80))

        for group_name, data, n_joints in [
            ("left_arm", jd.left_arm_cmd, 7),
            ("right_arm", jd.right_arm_cmd, 7),
            ("torso", jd.torso_cmd, 6),
        ]:
            if data:
                times = [s[0] for s in data]
                values = [s[1] for s in data]
                nj = min(n_joints, len(values[0]) if values else 0)
                for i in range(nj):
                    joint_vals = [v[i] for v in values]
                    dpg.set_value(f"series_{group_name}_{i}", [times, joint_vals])

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _module_enable_cb(self, sender, app_data, user_data):
        name = user_data
        self._module_enabled[name] = app_data
        if name in self._modules:
            self._modules[name].enabled = app_data
        if name in self._on_module_toggle:
            self._on_module_toggle[name](app_data)

    def _parameter_change_cb(self, sender, app_data, user_data):
        param_name = user_data
        self._parameters[param_name] = app_data
        if param_name in self._on_parameter_change:
            self._on_parameter_change[param_name](app_data)

    def _init_pose_cb(self, sender=None, app_data=None, user_data=None):
        if self._on_init_pose:
            self._on_init_pose()
        else:
            logger.warning("No init_pose callback registered")

    def _start_playback_cb(self, sender=None, app_data=None, user_data=None):
        if self._on_start_playback:
            self._on_start_playback()
        else:
            logger.warning("No start_playback callback registered")

    def _calibrate_cb(self, sender=None, app_data=None, user_data=None):
        if self._on_calibrate:
            self._on_calibrate()
        else:
            logger.warning("No calibrate callback registered")

    def _soft_stop_cb(self, sender=None, app_data=None, user_data=None):
        self._soft_stop_active = True
        logger.info("Soft stop requested")
        if self._on_soft_stop:
            self._on_soft_stop()

    def _resume_cb(self, sender=None, app_data=None, user_data=None):
        self._soft_stop_active = False
        logger.info("Resume requested")
        if self._on_resume:
            self._on_resume()

    def _estop_cb(self, sender=None, app_data=None, user_data=None):
        self._estop_active = not self._estop_active
        if self._estop_active:
            logger.warning("EMERGENCY STOP ACTIVATED — zeroing all commands")
        else:
            logger.info("Emergency stop released")
        if self._on_emergency_stop:
            self._on_emergency_stop(self._estop_active)

    def _record_cb(self, sender=None, app_data=None, user_data=None):
        self._recording = not self._recording
        label = f"Record: {'ON' if self._recording else 'OFF'}"
        dpg.configure_item("btn_record", label=label)
        if self._on_record_toggle:
            self._on_record_toggle(self._recording)
        logger.info(f"Recording: {self._recording}")

    def _launch_viewer_cb(self, sender=None, app_data=None, user_data=None):
        if self._viewer_process and self._viewer_process.poll() is None:
            logger.info("ROS2 viewer already running")
            return
        try:
            self._viewer_process = subprocess.Popen(
                [sys.executable, "-m", "teleop_system.modules.camera.pointcloud_viewer"],
                start_new_session=True,
            )
            logger.info("ROS2 viewer launched")
        except Exception as e:
            logger.error(f"Failed to launch ROS2 viewer: {e}")

    def _launch_tcp_viewer_cb(self, sender=None, app_data=None, user_data=None):
        if self._tcp_viewer_process and self._tcp_viewer_process.poll() is None:
            logger.info("TCP viewer already running")
            return
        if _DPG_AVAILABLE:
            host = dpg.get_value("tcp_viewer_host") or self._tcp_viewer_host
            port = dpg.get_value("tcp_viewer_port") or self._tcp_viewer_port
        else:
            host = self._tcp_viewer_host
            port = self._tcp_viewer_port
        script = Path(__file__).resolve().parent.parent.parent / "scripts" / "demo_rgbd_streaming.py"
        if not script.exists():
            logger.error(f"Streaming script not found: {script}")
            return
        try:
            self._tcp_viewer_process = subprocess.Popen(
                [
                    sys.executable, "-u", str(script),
                    "--mode", "client",
                    "--host", str(host),
                    "--port", str(port),
                    "--ros2-sync",
                ],
                start_new_session=True,
            )
            logger.info(f"TCP viewer launched -> {host}:{port}")
        except Exception as e:
            logger.error(f"Failed to launch TCP viewer: {e}")

    def _tcp_host_changed_cb(self, sender=None, app_data=None, user_data=None):
        if app_data:
            self._tcp_viewer_host = app_data

    def _module_toggle_cb(self, sender, app_data, user_data):
        name = user_data
        if name in self._on_module_toggle:
            self._on_module_toggle[name](app_data)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_parameter(self, name: str) -> float:
        return self._parameters.get(name, 0.0)

    def shutdown(self) -> None:
        """Destroy the GUI context."""
        if _DPG_AVAILABLE and self._running:
            dpg.destroy_context()
            self._running = False
            logger.info("Control panel shutdown")

        if self._viewer_process and self._viewer_process.poll() is None:
            self._viewer_process.terminate()
        if self._tcp_viewer_process and self._tcp_viewer_process.poll() is None:
            self._tcp_viewer_process.terminate()

    @property
    def is_running(self) -> bool:
        return self._running
