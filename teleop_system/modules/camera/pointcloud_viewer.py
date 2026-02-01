"""Real-time point cloud viewer using GLFW + OpenGL.

Displays 3D point clouds with mouse rotation/zoom. Supports
continuous updates for streaming visualization.

Optionally syncs viewer mouse orbit to a robot camera (ICameraStream),
so dragging the view also drives the robot's head pan/tilt.

Requires: glfw, PyOpenGL (both available in this environment).
"""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np

from teleop_system.interfaces.camera_stream import ICameraStream
from teleop_system.utils.logger import get_logger

logger = get_logger("pointcloud_viewer")

try:
    import glfw
    from OpenGL import GL

    _GL_AVAILABLE = True
except ImportError:
    _GL_AVAILABLE = False
    logger.warning("GLFW/OpenGL not available — PointCloudViewer cannot be used")


class PointCloudViewer:
    """Real-time 3D point cloud viewer with mouse orbit controls.

    Usage:
        viewer = PointCloudViewer(title="Point Cloud")
        viewer.initialize()

        while viewer.is_running():
            viewer.update_points(points, colors)  # (N,3) float32 each
            viewer.render()

        viewer.shutdown()

    Controls:
        Left-drag:  Rotate (orbit)
        Right-drag: Pan
        Scroll:     Zoom in/out
        R:          Reset view
        Q/ESC:      Close
    """

    def __init__(
        self,
        width: int = 960,
        height: int = 720,
        title: str = "Point Cloud Viewer",
        point_size: float = 2.0,
        background: tuple[float, float, float] = (0.1, 0.1, 0.15),
        camera: ICameraStream | None = None,
    ):
        self._width = width
        self._height = height
        self._title = title
        self._point_size = point_size
        self._bg = background
        self._window = None

        # Camera state (orbit camera)
        self._cam_distance = 5.0
        self._cam_yaw = 0.0      # degrees
        self._cam_pitch = -30.0   # degrees
        self._cam_target = np.array([0.0, 0.0, 0.0])
        self._cam_pan_offset = np.array([0.0, 0.0, 0.0])

        # Mouse state
        self._mouse_x = 0.0
        self._mouse_y = 0.0
        self._left_pressed = False
        self._right_pressed = False

        # Robot camera sync: viewer orbit drives robot head pan/tilt
        self._camera: ICameraStream | None = camera
        self._ref_yaw: float = 0.0     # viewer yaw at sync enable (degrees)
        self._ref_pitch: float = -30.0  # viewer pitch at sync enable (degrees)

        # Point data
        self._points: np.ndarray | None = None
        self._colors: np.ndarray | None = None
        self._point_count = 0

        # Stats overlay
        self._stats_text = ""
        self._frame_count = 0
        self._fps = 0.0
        self._fps_timer = 0.0

    def initialize(self) -> bool:
        if not _GL_AVAILABLE:
            logger.error("GLFW/OpenGL not available")
            return False

        if not glfw.init():
            logger.error("Failed to initialize GLFW")
            return False

        # Create window — use default GL context (don't request specific version,
        # which avoids GLX errors when MuJoCo uses EGL for off-screen rendering).
        glfw.default_window_hints()

        self._window = glfw.create_window(
            self._width, self._height, self._title, None, None
        )
        if not self._window:
            logger.error("Failed to create GLFW window")
            glfw.terminate()
            return False

        glfw.make_context_current(self._window)

        # Set callbacks
        glfw.set_cursor_pos_callback(self._window, self._cursor_callback)
        glfw.set_mouse_button_callback(self._window, self._mouse_button_callback)
        glfw.set_scroll_callback(self._window, self._scroll_callback)
        glfw.set_key_callback(self._window, self._key_callback)
        glfw.set_framebuffer_size_callback(self._window, self._resize_callback)

        # OpenGL setup
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_POINT_SMOOTH)
        GL.glPointSize(self._point_size)
        GL.glClearColor(*self._bg, 1.0)

        self._fps_timer = time.monotonic()

        # Record reference point for camera sync (uses current yaw/pitch)
        self._ref_yaw = self._cam_yaw
        self._ref_pitch = self._cam_pitch

        logger.info(f"PointCloudViewer initialized ({self._width}x{self._height})")
        return True

    def is_running(self) -> bool:
        if self._window is None:
            return False
        return not glfw.window_should_close(self._window)

    def update_points(
        self,
        points: np.ndarray,
        colors: np.ndarray | None = None,
    ) -> None:
        """Update the point cloud data.

        Args:
            points: (N, 3) float32 xyz positions.
            colors: (N, 3) float32 rgb in [0, 1]. If None, uses white.
        """
        if points is None or len(points) == 0:
            self._points = None
            self._colors = None
            self._point_count = 0
            return

        self._points = np.ascontiguousarray(points, dtype=np.float32)
        self._point_count = len(points)

        if colors is not None and len(colors) == len(points):
            self._colors = np.ascontiguousarray(colors, dtype=np.float32)
        else:
            self._colors = np.ones((self._point_count, 3), dtype=np.float32)

    def set_stats(self, text: str) -> None:
        """Set the stats overlay text (shown in window title)."""
        self._stats_text = text

    def render(self) -> None:
        """Render one frame."""
        if self._window is None:
            return

        glfw.make_context_current(self._window)
        glfw.poll_events()

        # Update window size
        w, h = glfw.get_framebuffer_size(self._window)
        if w == 0 or h == 0:
            return

        GL.glViewport(0, 0, w, h)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

        # Set projection matrix (perspective)
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        aspect = w / max(h, 1)
        _gl_perspective(60.0, aspect, 0.1, 200.0)

        # Set view matrix (orbit camera)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

        # Camera position from spherical coordinates
        yaw_rad = math.radians(self._cam_yaw)
        pitch_rad = math.radians(self._cam_pitch)
        cx = self._cam_distance * math.cos(pitch_rad) * math.sin(yaw_rad)
        cy = self._cam_distance * math.sin(pitch_rad)
        cz = self._cam_distance * math.cos(pitch_rad) * math.cos(yaw_rad)

        target = self._cam_target + self._cam_pan_offset
        _gl_look_at(
            cx + target[0], cy + target[1], cz + target[2],
            target[0], target[1], target[2],
            0.0, 1.0, 0.0,
        )

        # Draw grid on XZ plane
        self._draw_grid()

        # Draw axes at origin
        self._draw_axes()

        # Draw point cloud
        if self._points is not None and self._point_count > 0:
            GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
            GL.glEnableClientState(GL.GL_COLOR_ARRAY)

            GL.glVertexPointer(3, GL.GL_FLOAT, 0, self._points)
            GL.glColorPointer(3, GL.GL_FLOAT, 0, self._colors)
            GL.glDrawArrays(GL.GL_POINTS, 0, self._point_count)

            GL.glDisableClientState(GL.GL_COLOR_ARRAY)
            GL.glDisableClientState(GL.GL_VERTEX_ARRAY)

        glfw.swap_buffers(self._window)

        # Update FPS counter
        self._frame_count += 1
        now = time.monotonic()
        dt = now - self._fps_timer
        if dt >= 0.5:
            self._fps = self._frame_count / dt
            self._frame_count = 0
            self._fps_timer = now

            # Update window title with stats
            title = f"{self._title}  |  {self._point_count:,} pts  |  {self._fps:.0f} fps"
            if self._stats_text:
                title += f"  |  {self._stats_text}"
            glfw.set_window_title(self._window, title)

    def shutdown(self) -> None:
        if self._window is not None:
            glfw.destroy_window(self._window)
            self._window = None
        glfw.terminate()
        logger.info("PointCloudViewer shutdown")

    def reset_view(self) -> None:
        """Reset camera to default position."""
        self._cam_distance = 5.0
        self._cam_yaw = 0.0
        self._cam_pitch = -30.0
        self._cam_pan_offset = np.array([0.0, 0.0, 0.0])
        # Reset robot camera reference point
        self._ref_yaw = self._cam_yaw
        self._ref_pitch = self._cam_pitch
        if self._camera is not None:
            self._camera.set_orientation(0.0, 0.0)

    def set_camera(self, camera: ICameraStream | None) -> None:
        """Set or clear the robot camera for viewer-to-robot sync.

        When set, mouse orbit (left-drag) will also drive the robot
        camera's pan/tilt joints, simulating VR head tracking.

        Args:
            camera: ICameraStream to sync, or None to disable.
        """
        self._camera = camera
        # Record current viewer orientation as the reference (zero) point
        self._ref_yaw = self._cam_yaw
        self._ref_pitch = self._cam_pitch

    def _sync_camera_orientation(self) -> None:
        """Map viewer yaw/pitch delta to robot pan/tilt and send command."""
        if self._camera is None:
            return

        # Delta from reference point (degrees)
        delta_yaw = self._cam_yaw - self._ref_yaw
        delta_pitch = self._cam_pitch - self._ref_pitch

        # Convert to radians and map to robot joint ranges.
        # head_0 (pan):  [-0.523, 0.523] rad — positive = turn left
        # head_1 (tilt): [-0.35, 1.57] rad  — positive = tilt back/up
        #
        # Viewer yaw increases when dragging right (orbit right).
        # For intuitive VR-like control:
        #   drag right → see right side → robot pan negative (turn right)
        # Viewer pitch increases when dragging down (orbit below).
        #   drag up → pitch decreases → delta_pitch < 0 → robot tilt positive (look up)
        pan = float(np.clip(np.radians(-delta_yaw), -0.523, 0.523))
        tilt = float(np.clip(np.radians(-delta_pitch), -0.35, 1.57))

        self._camera.set_orientation(pan, tilt)

    # ── Drawing helpers ──

    def _draw_grid(self) -> None:
        """Draw a grid on the XZ plane at y=0."""
        GL.glBegin(GL.GL_LINES)
        GL.glColor3f(0.25, 0.25, 0.3)
        size = 10
        for i in range(-size, size + 1):
            GL.glVertex3f(float(i), 0.0, float(-size))
            GL.glVertex3f(float(i), 0.0, float(size))
            GL.glVertex3f(float(-size), 0.0, float(i))
            GL.glVertex3f(float(size), 0.0, float(i))
        GL.glEnd()

    def _draw_axes(self) -> None:
        """Draw RGB axes at origin (X=red, Y=green, Z=blue)."""
        GL.glLineWidth(2.0)
        GL.glBegin(GL.GL_LINES)
        # X axis (red)
        GL.glColor3f(1.0, 0.3, 0.3)
        GL.glVertex3f(0.0, 0.0, 0.0)
        GL.glVertex3f(1.0, 0.0, 0.0)
        # Y axis (green)
        GL.glColor3f(0.3, 1.0, 0.3)
        GL.glVertex3f(0.0, 0.0, 0.0)
        GL.glVertex3f(0.0, 1.0, 0.0)
        # Z axis (blue)
        GL.glColor3f(0.3, 0.3, 1.0)
        GL.glVertex3f(0.0, 0.0, 0.0)
        GL.glVertex3f(0.0, 0.0, 1.0)
        GL.glEnd()
        GL.glLineWidth(1.0)

    # ── GLFW callbacks ──

    def _cursor_callback(self, window, x, y):
        dx = x - self._mouse_x
        dy = y - self._mouse_y
        self._mouse_x = x
        self._mouse_y = y

        if self._left_pressed:
            self._cam_yaw += dx * 0.3
            self._cam_pitch += dy * 0.3
            self._cam_pitch = max(-89.0, min(89.0, self._cam_pitch))
            self._sync_camera_orientation()

        if self._right_pressed:
            # Pan: move target in screen-aligned plane
            yaw_rad = math.radians(self._cam_yaw)
            right = np.array([math.cos(yaw_rad), 0, -math.sin(yaw_rad)])
            up = np.array([0, 1, 0])
            scale = self._cam_distance * 0.002
            self._cam_pan_offset -= right * dx * scale
            self._cam_pan_offset += up * dy * scale

    def _mouse_button_callback(self, window, button, action, mods):
        if button == glfw.MOUSE_BUTTON_LEFT:
            self._left_pressed = (action == glfw.PRESS)
        if button == glfw.MOUSE_BUTTON_RIGHT:
            self._right_pressed = (action == glfw.PRESS)

    def _scroll_callback(self, window, xoff, yoff):
        self._cam_distance *= 0.9 if yoff > 0 else 1.1
        self._cam_distance = max(0.5, min(100.0, self._cam_distance))

    def _key_callback(self, window, key, scancode, action, mods):
        if action != glfw.PRESS:
            return
        if key in (glfw.KEY_Q, glfw.KEY_ESCAPE):
            glfw.set_window_should_close(window, True)
        if key == glfw.KEY_R:
            self.reset_view()

    def _resize_callback(self, window, width, height):
        self._width = width
        self._height = height


# ── OpenGL helper functions (fixed-function pipeline) ──

def _gl_perspective(fovy_deg: float, aspect: float, near: float, far: float):
    """Set perspective projection matrix (replaces gluPerspective)."""
    f = 1.0 / math.tan(math.radians(fovy_deg) / 2.0)
    mat = np.zeros(16, dtype=np.float64)
    mat[0] = f / aspect
    mat[5] = f
    mat[10] = (far + near) / (near - far)
    mat[11] = -1.0
    mat[14] = (2.0 * far * near) / (near - far)
    GL.glMultMatrixd(mat)


def _gl_look_at(
    ex, ey, ez,
    cx, cy, cz,
    ux, uy, uz,
):
    """Set view matrix (replaces gluLookAt)."""
    f = np.array([cx - ex, cy - ey, cz - ez], dtype=np.float64)
    f /= np.linalg.norm(f)
    u = np.array([ux, uy, uz], dtype=np.float64)
    s = np.cross(f, u)
    s /= np.linalg.norm(s)
    u = np.cross(s, f)

    mat = np.eye(4, dtype=np.float64)
    mat[0, :3] = s
    mat[1, :3] = u
    mat[2, :3] = -f
    GL.glMultMatrixd(mat.T.flatten())
    GL.glTranslated(-ex, -ey, -ez)
