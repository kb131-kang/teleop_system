"""GUI control panel using Dear PyGui.

Provides a tabbed control panel for teleoperation system monitoring and
configuration. Four tabs: Status, Tracker View, Joint States, Parameters.

Thread-safety: Dear PyGui calls must happen on the main thread only.
ROS2 callbacks write to shared buffers protected by threading.Lock.
The render_frame() method reads from those buffers on the main thread.
"""

from __future__ import annotations

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
    enabled: bool = False
    error_msg: str = ""
    metrics: dict[str, float] = field(default_factory=dict)


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
class JointStateData:
    """Buffer for joint state time series."""
    max_samples: int = 500  # ~5s at 100Hz
    # Deques of (timestamp, values) for rolling window
    left_arm_cmd: deque = field(default_factory=lambda: deque(maxlen=500))
    right_arm_cmd: deque = field(default_factory=lambda: deque(maxlen=500))
    torso_cmd: deque = field(default_factory=lambda: deque(maxlen=500))
    joint_states: deque = field(default_factory=lambda: deque(maxlen=500))


# Tracker role display configuration
TRACKER_COLORS = {
    "right_hand": (255, 80, 80, 255),    # Red
    "left_hand": (80, 130, 255, 255),     # Blue
    "waist": (80, 220, 80, 255),          # Green
    "right_foot": (255, 180, 50, 255),    # Orange
    "left_foot": (255, 220, 80, 255),     # Yellow-orange
    "head": (200, 100, 255, 255),         # Purple
}

# Parameter definitions: (name, default, min, max)
PARAMETER_DEFS = [
    ("position_scale", 1.0, 0.0, 5.0),
    ("orientation_scale", 1.0, 0.0, 5.0),
    ("max_joint_velocity", 2.0, 0.1, 10.0),
    ("ik_posture_cost", 0.1, 0.0, 1.0),
    ("hand_smoothing", 0.3, 0.0, 1.0),
    ("locomotion_deadzone", 0.02, 0.0, 0.2),
]


def _detect_display_scale() -> float:
    """Auto-detect display scale factor from screen resolution.

    Returns a font/UI scale factor:
        >= 3840px wide (4K)   → 2.0
        >= 2560px wide (1440p) → 1.5
        >= 1920px wide (1080p) → 1.2
        otherwise              → 1.0
    """
    try:
        import subprocess as _sp
        result = _sp.run(
            ["xrandr", "--current"],
            capture_output=True, text=True, timeout=2,
        )
        for line in result.stdout.splitlines():
            if "*" in line:  # current resolution marked with *
                res = line.split()[0]  # e.g. "3840x2160"
                w = int(res.split("x")[0])
                if w >= 3840:
                    return 2.0
                elif w >= 2560:
                    return 1.5
                elif w >= 1920:
                    return 1.2
                else:
                    return 1.0
    except Exception:
        pass
    return 1.2  # safe default when detection fails


class ControlPanel:
    """Dear PyGui-based control panel for the teleoperation system.

    Four-tab layout: Status | Tracker View | Joint States | Parameters
    """

    def __init__(
        self,
        title: str = "RB-Y1 Teleoperation Control Panel",
        width: int = 1000,
        height: int = 700,
        font_scale: float | None = None,
    ):
        self._title = title
        self._font_scale = font_scale  # None = auto-detect
        # Detect scale for window sizing
        scale = font_scale or _detect_display_scale()
        self._width = max(width, int(1000 * scale))
        self._height = max(height, int(700 * scale))
        self._running = False

        # Thread-safe shared data
        self._lock = threading.Lock()
        self._modules: dict[str, ModuleStatus] = {}
        self._tracker_data = TrackerData()
        self._joint_data = JointStateData()

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

        # Internal state
        self._recording = False
        self._playback_state = "UNKNOWN"  # READY, PLAYING, or UNKNOWN
        self._viewer_process: subprocess.Popen | None = None
        self._tcp_viewer_process: subprocess.Popen | None = None

        # TCP viewer config (set via set_tcp_viewer_config)
        self._tcp_viewer_host: str = "localhost"
        self._tcp_viewer_port: int = 9876

    @property
    def lock(self) -> threading.Lock:
        """Lock for thread-safe data access."""
        return self._lock

    @property
    def tracker_data(self) -> TrackerData:
        """Direct access to tracker data buffer (use lock!)."""
        return self._tracker_data

    @property
    def joint_data(self) -> JointStateData:
        """Direct access to joint state buffer (use lock!)."""
        return self._joint_data

    # ------------------------------------------------------------------
    # Module registration
    # ------------------------------------------------------------------

    def add_module(self, name: str, on_toggle: callable | None = None) -> ModuleStatus:
        status = ModuleStatus(name)
        self._modules[name] = status
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

    def set_tcp_viewer_config(self, host: str, port: int) -> None:
        """Set TCP viewer connection parameters."""
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

        # Font scaling: auto-detect from screen resolution or use explicit value
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
            # System mode
            dpg.add_text("System Mode", color=(200, 200, 255))
            dpg.add_separator()
            dpg.add_radio_button(
                items=["Simulation", "Hardware"],
                tag="mode_radio",
                horizontal=True,
            )
            dpg.add_spacer(height=8)

            # Module status
            dpg.add_text("Module Status", color=(200, 200, 255))
            dpg.add_separator()

            for name in self._modules:
                with dpg.group(horizontal=True):
                    dpg.add_text(
                        "●", tag=f"status_{name}", color=(128, 128, 128),
                    )
                    dpg.add_text(f" {name}", tag=f"label_{name}")
                    dpg.add_checkbox(
                        label="Enable", tag=f"toggle_{name}",
                        callback=self._module_toggle_cb, user_data=name,
                    )

            dpg.add_spacer(height=8)

            # Calibration status
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

            # Playback status
            dpg.add_text("Playback", color=(200, 200, 255))
            dpg.add_separator()
            with dpg.group(horizontal=True):
                dpg.add_text("State:", color=(180, 180, 180))
                dpg.add_text("UNKNOWN", tag="playback_state_text")
            dpg.add_spacer(height=8)

            # Action buttons
            dpg.add_text("Actions", color=(200, 200, 255))
            dpg.add_separator()

            with dpg.group(horizontal=True):
                dpg.add_button(
                    label="Start Playback", tag="btn_start_playback",
                    callback=self._start_playback_cb,
                    width=130,
                )
                dpg.add_button(
                    label="Calibrate (A-Pose)", tag="btn_calibrate",
                    callback=self._calibrate_cb,
                    width=150,
                )
                dpg.add_button(
                    label="Viewer (ROS2)", tag="btn_rgbd_viewer",
                    callback=self._launch_viewer_cb,
                    width=120,
                )
                dpg.add_button(
                    label="EMERGENCY STOP", tag="btn_estop",
                    callback=self._estop_cb,
                    width=150,
                )
                dpg.add_button(
                    label="Record: OFF", tag="btn_record",
                    callback=self._record_cb,
                    width=120,
                )

            # TCP Viewer row
            with dpg.group(horizontal=True):
                dpg.add_button(
                    label="Viewer (TCP)", tag="btn_tcp_viewer",
                    callback=self._launch_tcp_viewer_cb,
                    width=120,
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
                    width=80,
                    min_value=1024,
                    max_value=65535,
                    min_clamped=True,
                    max_clamped=True,
                )

            dpg.add_spacer(height=8)

            # Metrics
            dpg.add_text("Metrics", color=(200, 200, 255))
            dpg.add_separator()
            dpg.add_text("", tag="metrics_text")

    def _build_tracker_tab(self) -> None:
        with dpg.tab(label="Tracker View", tag="tab_tracker"):
            dpg.add_text(
                "Tracker Positions (live)", color=(200, 200, 255),
            )
            dpg.add_separator()

            # Legend
            with dpg.group(horizontal=True):
                for name, color in TRACKER_COLORS.items():
                    dpg.add_text(f"● {name}", color=color[:3])
                    dpg.add_spacer(width=10)

            dpg.add_spacer(height=5)

            # Two plots side by side: Top-down (XY) and Side (XZ)
            with dpg.group(horizontal=True):
                # Top-down view (X-Y)
                with dpg.plot(
                    label="Top-Down (X-Y)", tag="plot_tracker_xy",
                    width=450, height=400,
                ):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(
                        dpg.mvXAxis, label="X (m)",
                        tag="tracker_xy_x",
                    )
                    dpg.set_axis_limits("tracker_xy_x", -1.0, 1.0)
                    dpg.add_plot_axis(
                        dpg.mvYAxis, label="Y (m)",
                        tag="tracker_xy_y",
                    )
                    dpg.set_axis_limits("tracker_xy_y", -1.0, 1.0)

                    for name, color in TRACKER_COLORS.items():
                        dpg.add_scatter_series(
                            [0.0], [0.0],
                            label=name,
                            tag=f"series_xy_{name}",
                            parent="tracker_xy_y",
                        )

                # Side view (X-Z)
                with dpg.plot(
                    label="Side (X-Z)", tag="plot_tracker_xz",
                    width=450, height=400,
                ):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(
                        dpg.mvXAxis, label="X (m)",
                        tag="tracker_xz_x",
                    )
                    dpg.set_axis_limits("tracker_xz_x", -1.0, 1.0)
                    dpg.add_plot_axis(
                        dpg.mvYAxis, label="Z (m)",
                        tag="tracker_xz_y",
                    )
                    dpg.set_axis_limits("tracker_xz_y", -0.5, 2.0)

                    for name, color in TRACKER_COLORS.items():
                        dpg.add_scatter_series(
                            [0.0], [0.0],
                            label=name,
                            tag=f"series_xz_{name}",
                            parent="tracker_xz_y",
                        )

    def _build_joint_states_tab(self) -> None:
        with dpg.tab(label="Joint States", tag="tab_joints"):
            dpg.add_text(
                "Joint Position Time Series", color=(200, 200, 255),
            )
            dpg.add_separator()

            # Left arm plot
            with dpg.plot(
                label="Left Arm Joints", tag="plot_left_arm",
                width=-1, height=180,
            ):
                dpg.add_plot_legend()
                dpg.add_plot_axis(
                    dpg.mvXAxis, label="Time (s)", tag="left_arm_x",
                )
                dpg.add_plot_axis(
                    dpg.mvYAxis, label="Position (rad)", tag="left_arm_y",
                )

                # 7 joints per arm
                for i in range(7):
                    dpg.add_line_series(
                        [], [], label=f"J{i}",
                        tag=f"series_left_arm_{i}",
                        parent="left_arm_y",
                    )

            dpg.add_spacer(height=5)

            # Right arm plot
            with dpg.plot(
                label="Right Arm Joints", tag="plot_right_arm",
                width=-1, height=180,
            ):
                dpg.add_plot_legend()
                dpg.add_plot_axis(
                    dpg.mvXAxis, label="Time (s)", tag="right_arm_x",
                )
                dpg.add_plot_axis(
                    dpg.mvYAxis, label="Position (rad)", tag="right_arm_y",
                )

                for i in range(7):
                    dpg.add_line_series(
                        [], [], label=f"J{i}",
                        tag=f"series_right_arm_{i}",
                        parent="right_arm_y",
                    )

            dpg.add_spacer(height=5)

            # Torso plot
            with dpg.plot(
                label="Torso Joints", tag="plot_torso",
                width=-1, height=180,
            ):
                dpg.add_plot_legend()
                dpg.add_plot_axis(
                    dpg.mvXAxis, label="Time (s)", tag="torso_x",
                )
                dpg.add_plot_axis(
                    dpg.mvYAxis, label="Position (rad)", tag="torso_y",
                )

                for i in range(6):
                    dpg.add_line_series(
                        [], [], label=f"J{i}",
                        tag=f"series_torso_{i}",
                        parent="torso_y",
                    )

    def _build_parameters_tab(self) -> None:
        with dpg.tab(label="Parameters", tag="tab_params"):
            dpg.add_text("Teleop Parameters", color=(200, 200, 255))
            dpg.add_separator()

            for name, default, min_val, max_val in PARAMETER_DEFS:
                label = name.replace("_", " ").title()
                dpg.add_slider_float(
                    label=label,
                    tag=f"param_{name}",
                    default_value=default,
                    min_value=min_val,
                    max_value=max_val,
                    callback=self._parameter_change_cb,
                    user_data=name,
                    width=400,
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
            self._update_joint_states_tab()

        dpg.render_dearpygui_frame()
        return True

    def _update_status_tab(self) -> None:
        # Module status indicators
        for name, status in self._modules.items():
            if status.connected and status.enabled:
                color = (0, 255, 0)
            elif status.connected:
                color = (255, 255, 0)
            else:
                color = (255, 0, 0)
            dpg.configure_item(f"status_{name}", color=color)

        # Calibration status
        td = self._tracker_data
        dpg.set_value("cal_state_text", td.calibration_state)

        # Color code calibration state text
        state_colors = {
            "IDLE": (180, 180, 180),
            "WAITING": (255, 255, 100),
            "CAPTURING": (100, 200, 255),
            "COMPUTING": (100, 200, 255),
            "CALIBRATED": (80, 255, 80),
            "ERROR": (255, 80, 80),
        }
        state_color = state_colors.get(td.calibration_state, (180, 180, 180))
        dpg.configure_item("cal_state_text", color=state_color)

        progress = td.calibration_progress
        overlay = f"{int(progress * 100)}%"
        if td.calibration_state == "WAITING" and td.calibration_countdown > 0:
            overlay = f"Countdown: {td.calibration_countdown:.0f}s"
        dpg.set_value("cal_progress", progress)
        dpg.configure_item("cal_progress", overlay=overlay)

        dpg.set_value("cal_error_text", td.calibration_error)

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
        # Disable start button once playing
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
        for name in TRACKER_COLORS:
            pos = td.positions.get(name)
            if pos is not None:
                dpg.set_value(f"series_xy_{name}", [[float(pos[0])], [float(pos[1])]])
                dpg.set_value(f"series_xz_{name}", [[float(pos[0])], [float(pos[2])]])

    def _update_joint_states_tab(self) -> None:
        jd = self._joint_data

        # Left arm
        if jd.left_arm_cmd:
            times = [s[0] for s in jd.left_arm_cmd]
            values = [s[1] for s in jd.left_arm_cmd]
            n_joints = min(7, len(values[0]) if values else 0)
            for i in range(n_joints):
                joint_vals = [v[i] for v in values]
                dpg.set_value(
                    f"series_left_arm_{i}", [times, joint_vals],
                )

        # Right arm
        if jd.right_arm_cmd:
            times = [s[0] for s in jd.right_arm_cmd]
            values = [s[1] for s in jd.right_arm_cmd]
            n_joints = min(7, len(values[0]) if values else 0)
            for i in range(n_joints):
                joint_vals = [v[i] for v in values]
                dpg.set_value(
                    f"series_right_arm_{i}", [times, joint_vals],
                )

        # Torso
        if jd.torso_cmd:
            times = [s[0] for s in jd.torso_cmd]
            values = [s[1] for s in jd.torso_cmd]
            n_joints = min(6, len(values[0]) if values else 0)
            for i in range(n_joints):
                joint_vals = [v[i] for v in values]
                dpg.set_value(
                    f"series_torso_{i}", [times, joint_vals],
                )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _module_toggle_cb(self, sender, app_data, user_data):
        name = user_data
        if name in self._on_module_toggle:
            self._on_module_toggle[name](app_data)

    def _parameter_change_cb(self, sender, app_data, user_data):
        param_name = user_data
        self._parameters[param_name] = app_data
        if param_name in self._on_parameter_change:
            self._on_parameter_change[param_name](app_data)

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

    def _launch_viewer_cb(self, sender=None, app_data=None, user_data=None):
        """Launch RGB-D point cloud viewer (ROS2 mode) as subprocess."""
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
        """Launch RGB-D point cloud viewer (TCP client mode) as subprocess."""
        if self._tcp_viewer_process and self._tcp_viewer_process.poll() is None:
            logger.info("TCP viewer already running")
            return

        # Read host/port from GUI inputs
        if _DPG_AVAILABLE:
            host = dpg.get_value("tcp_viewer_host") or self._tcp_viewer_host
            port = dpg.get_value("tcp_viewer_port") or self._tcp_viewer_port
        else:
            host = self._tcp_viewer_host
            port = self._tcp_viewer_port

        # Locate the streaming script
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
            logger.info(f"TCP viewer launched → {host}:{port}")
        except Exception as e:
            logger.error(f"Failed to launch TCP viewer: {e}")

    def _tcp_host_changed_cb(self, sender=None, app_data=None, user_data=None):
        """Update TCP host from input field."""
        if app_data:
            self._tcp_viewer_host = app_data

    def _estop_cb(self, sender=None, app_data=None, user_data=None):
        logger.warning("EMERGENCY STOP triggered")
        if self._on_emergency_stop:
            self._on_emergency_stop()

    def _record_cb(self, sender=None, app_data=None, user_data=None):
        self._recording = not self._recording
        label = f"Record: {'ON' if self._recording else 'OFF'}"
        dpg.configure_item("btn_record", label=label)
        if self._on_record_toggle:
            self._on_record_toggle(self._recording)
        logger.info(f"Recording: {self._recording}")

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
