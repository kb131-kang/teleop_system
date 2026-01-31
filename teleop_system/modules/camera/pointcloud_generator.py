"""Point cloud generation from RGB-D frames.

Converts RGB-D images to 3D point clouds using camera intrinsics
for back-projection. Supports Open3D for efficient processing and
optional voxel downsampling.
"""

import numpy as np

from teleop_system.interfaces.camera_stream import RGBDFrame
from teleop_system.utils.logger import get_logger

logger = get_logger("pointcloud_generator")

try:
    import open3d as o3d
    _OPEN3D_AVAILABLE = True
except ImportError:
    _OPEN3D_AVAILABLE = False
    logger.warning("Open3D not available — point cloud generation will use numpy fallback")


class PointCloudGenerator:
    """Generates 3D point clouds from RGB-D frames.

    Uses camera intrinsics to back-project depth pixels into 3D space.
    Can optionally use Open3D for accelerated processing and voxel filtering.
    """

    def __init__(
        self,
        voxel_size: float = 0.0,
        max_depth: float = 5.0,
        min_depth: float = 0.1,
        use_open3d: bool = True,
    ):
        """Initialize point cloud generator.

        Args:
            voxel_size: Voxel downsampling size in meters (0 = no downsampling).
            max_depth: Maximum depth threshold in meters.
            min_depth: Minimum depth threshold in meters.
            use_open3d: Whether to use Open3D (falls back to numpy if unavailable).
        """
        self._voxel_size = voxel_size
        self._max_depth = max_depth
        self._min_depth = min_depth
        self._use_open3d = use_open3d and _OPEN3D_AVAILABLE

    def generate(self, frame: RGBDFrame) -> dict:
        """Generate a point cloud from an RGB-D frame.

        Args:
            frame: Input RGB-D frame with intrinsics.

        Returns:
            Dict with:
                - 'points': (N, 3) float32 xyz positions
                - 'colors': (N, 3) float32 rgb in [0, 1]
                - 'count': Number of points
                - 'o3d_pcd': Open3D PointCloud object (if Open3D is available)
        """
        if self._use_open3d:
            return self._generate_open3d(frame)
        return self._generate_numpy(frame)

    def _generate_open3d(self, frame: RGBDFrame) -> dict:
        """Generate point cloud using Open3D."""
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=frame.width,
            height=frame.height,
            fx=frame.intrinsics[0, 0],
            fy=frame.intrinsics[1, 1],
            cx=frame.intrinsics[0, 2],
            cy=frame.intrinsics[1, 2],
        )

        rgb_o3d = o3d.geometry.Image(frame.rgb.astype(np.uint8))
        depth_o3d = o3d.geometry.Image(frame.depth.astype(np.float32))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d,
            depth_o3d,
            depth_scale=1.0,
            depth_trunc=self._max_depth,
            convert_rgb_to_intensity=False,
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, intrinsic
        )

        # Filter by min depth (Open3D doesn't have min_depth in create_from_rgbd)
        if self._min_depth > 0:
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)
            depths = np.linalg.norm(points, axis=1)
            mask = depths >= self._min_depth
            pcd = pcd.select_by_index(np.where(mask)[0])

        if self._voxel_size > 0:
            pcd = pcd.voxel_down_sample(self._voxel_size)

        points = np.asarray(pcd.points, dtype=np.float32)
        colors = np.asarray(pcd.colors, dtype=np.float32)

        return {
            "points": points,
            "colors": colors,
            "count": len(points),
            "o3d_pcd": pcd,
        }

    def _generate_numpy(self, frame: RGBDFrame) -> dict:
        """Generate point cloud using pure numpy (fallback)."""
        h, w = frame.depth.shape[:2]
        fx = frame.intrinsics[0, 0]
        fy = frame.intrinsics[1, 1]
        cx = frame.intrinsics[0, 2]
        cy = frame.intrinsics[1, 2]

        # Create pixel coordinate grid
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        depth = frame.depth.astype(np.float32)

        # Depth mask
        valid = (depth >= self._min_depth) & (depth <= self._max_depth)

        # Back-project to 3D
        z = depth[valid]
        x = (u[valid] - cx) * z / fx
        y = (v[valid] - cy) * z / fy

        points = np.stack([x, y, z], axis=-1).astype(np.float32)

        # Extract colors
        if frame.rgb.ndim == 3 and frame.rgb.shape[2] == 3:
            colors = frame.rgb[valid].astype(np.float32) / 255.0
        else:
            colors = np.zeros((len(points), 3), dtype=np.float32)

        return {
            "points": points,
            "colors": colors,
            "count": len(points),
            "o3d_pcd": None,
        }
