"""GUI control panel using Dear PyGui.

Provides a single-window control panel for teleoperation system monitoring
and configuration. Shows module status, enable/disable toggles, and
real-time parameter sliders.

Designed to work without ROS2 — communicates through shared state objects.
"""

from __future__ import annotations

import numpy as np

from teleop_system.utils.logger import get_logger

logger = get_logger("control_panel")

try:
    import dearpygui.dearpygui as dpg
    _DPG_AVAILABLE = True
except ImportError:
    _DPG_AVAILABLE = False
    logger.warning("Dear PyGui not available — GUI control panel disabled")


class ModuleStatus:
    """Shared state for a single module's GUI display."""

    def __init__(self, name: str):
        self.name = name
        self.connected = False
        self.enabled = False
        self.error_msg = ""
        self.metrics: dict[str, float] = {}


class ControlPanel:
    """Dear PyGui-based control panel for the teleoperation system.

    Shows real-time status of all modules and allows interactive control.
    Thread-safe: can be updated from a control loop thread.
    """

    def __init__(
        self,
        title: str = "RB-Y1 Teleoperation Control Panel",
        width: int = 800,
        height: int = 600,
    ):
        self._title = title
        self._width = width
        self._height = height
        self._modules: dict[str, ModuleStatus] = {}
        self._running = False

        # Callbacks for user interaction
        self._on_module_toggle: dict[str, callable] = {}
        self._on_parameter_change: dict[str, callable] = {}

        # Parameter values
        self._parameters: dict[str, float] = {
            "position_scale": 1.0,
            "orientation_scale": 1.0,
            "max_joint_velocity": 2.0,
            "ik_posture_cost": 0.1,
            "hand_smoothing": 0.3,
            "locomotion_deadzone": 0.02,
        }

    def add_module(self, name: str, on_toggle: callable | None = None) -> ModuleStatus:
        """Register a module for display in the GUI.

        Args:
            name: Module display name.
            on_toggle: Callback when user toggles enable/disable.

        Returns:
            ModuleStatus object to update from the control loop.
        """
        status = ModuleStatus(name)
        self._modules[name] = status
        if on_toggle:
            self._on_module_toggle[name] = on_toggle
        return status

    def set_parameter_callback(self, param_name: str, callback: callable) -> None:
        """Register a callback for when a parameter slider changes."""
        self._on_parameter_change[param_name] = callback

    def setup(self) -> bool:
        """Initialize Dear PyGui context and create the GUI layout.

        Returns:
            True if GUI was created successfully.
        """
        if not _DPG_AVAILABLE:
            logger.error("Dear PyGui not installed")
            return False

        dpg.create_context()
        dpg.create_viewport(title=self._title, width=self._width, height=self._height)

        with dpg.window(label="Control Panel", tag="main_window"):
            # System mode section
            dpg.add_text("System Mode", color=(200, 200, 255))
            dpg.add_separator()
            dpg.add_radio_button(
                items=["Simulation", "Hardware"],
                tag="mode_radio",
                horizontal=True,
            )
            dpg.add_spacer(height=10)

            # Module status section
            dpg.add_text("Module Status", color=(200, 200, 255))
            dpg.add_separator()

            for name in self._modules:
                with dpg.group(horizontal=True):
                    dpg.add_text("●", tag=f"status_{name}", color=(128, 128, 128))
                    dpg.add_text(f" {name}")
                    dpg.add_checkbox(
                        label="Enable",
                        tag=f"toggle_{name}",
                        callback=self._module_toggle_callback,
                        user_data=name,
                    )

            dpg.add_spacer(height=10)

            # Parameters section
            dpg.add_text("Parameters", color=(200, 200, 255))
            dpg.add_separator()

            for param_name, default_val in self._parameters.items():
                label = param_name.replace("_", " ").title()
                dpg.add_slider_float(
                    label=label,
                    tag=f"param_{param_name}",
                    default_value=default_val,
                    min_value=0.0,
                    max_value=10.0,
                    callback=self._parameter_change_callback,
                    user_data=param_name,
                )

            dpg.add_spacer(height=10)

            # Metrics section
            dpg.add_text("Metrics", color=(200, 200, 255))
            dpg.add_separator()
            dpg.add_text("", tag="metrics_text")

        dpg.set_primary_window("main_window", True)
        dpg.setup_dearpygui()

        self._running = True
        logger.info("Control panel initialized")
        return True

    def render_frame(self) -> bool:
        """Render one GUI frame. Call this in a loop.

        Returns:
            False if the window was closed.
        """
        if not _DPG_AVAILABLE or not self._running:
            return False

        if not dpg.is_dearpygui_running():
            self._running = False
            return False

        # Update module status indicators
        for name, status in self._modules.items():
            if status.connected and status.enabled:
                color = (0, 255, 0)  # Green
            elif status.connected:
                color = (255, 255, 0)  # Yellow (connected but disabled)
            else:
                color = (255, 0, 0)  # Red

            dpg.configure_item(f"status_{name}", color=color)

        # Update metrics display
        metrics_lines = []
        for name, status in self._modules.items():
            for key, val in status.metrics.items():
                metrics_lines.append(f"  {name}/{key}: {val:.4f}")
        dpg.set_value("metrics_text", "\n".join(metrics_lines) if metrics_lines else "(no metrics)")

        dpg.render_dearpygui_frame()
        return True

    def shutdown(self) -> None:
        """Destroy the GUI context."""
        if _DPG_AVAILABLE and self._running:
            dpg.destroy_context()
            self._running = False
            logger.info("Control panel shutdown")

    def _module_toggle_callback(self, sender, app_data, user_data):
        name = user_data
        if name in self._on_module_toggle:
            self._on_module_toggle[name](app_data)

    def _parameter_change_callback(self, sender, app_data, user_data):
        param_name = user_data
        self._parameters[param_name] = app_data
        if param_name in self._on_parameter_change:
            self._on_parameter_change[param_name](app_data)

    def get_parameter(self, name: str) -> float:
        """Get current parameter value."""
        return self._parameters.get(name, 0.0)

    @property
    def is_running(self) -> bool:
        return self._running
