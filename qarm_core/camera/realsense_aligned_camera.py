"""RealSense camera reader with depth aligned to the color image.

This backend uses Intel's pyrealsense2 package directly. It is meant for the
physical RealSense camera when Python/OpenCV needs depth values that line up
with clicked color pixels.
"""

from __future__ import annotations

import numpy as np

from qarm_core import config

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


class RealSenseAlignedCamera:
    """Read color frames and aligned depth frames from a physical RealSense."""

    def __init__(
        self,
        width=config.CAMERA_WIDTH,
        height=config.CAMERA_HEIGHT,
        fps=config.CAMERA_FPS,
    ):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)

        self.pipeline = None
        self.align_to_color = None
        self.depth_scale = 0.001

    def open(self):
        """Open the RealSense stream and prepare color-aligned depth."""

        if rs is None:
            raise RuntimeError(
                "pyrealsense2 is not installed. Install it with: pip install pyrealsense2"
            )

        stream_config = rs.config()

        # Request BGR color frames because OpenCV displays BGR directly.
        stream_config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.bgr8,
            self.fps,
        )
        stream_config.enable_stream(
            rs.stream.depth,
            self.width,
            self.height,
            rs.format.z16,
            self.fps,
        )

        self.pipeline = rs.pipeline()
        profile = self.pipeline.start(stream_config)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())

        # Align depth pixels to color pixels. After this, aligned_depth_m[y, x]
        # is the depth in meters for the color image pixel at x, y.
        self.align_to_color = rs.align(rs.stream.color)

        print("Opened physical RealSense camera with depth aligned to color.")

    def read(self):
        """Return one OpenCV BGR color frame and aligned depth in meters."""

        if self.pipeline is None or self.align_to_color is None:
            return None, None

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align_to_color.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None

        color_bgr = np.asanyarray(color_frame.get_data()).copy()

        # RealSense depth arrives as integer units. Multiplying by depth_scale
        # converts those units into meters for simple distance math.
        aligned_depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        aligned_depth_m = aligned_depth_m * self.depth_scale

        return color_bgr, aligned_depth_m

    def close(self):
        """Stop the RealSense pipeline if it is running."""

        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
            self.align_to_color = None


def get_depth_at_rgb_pixel(
    depth_m,
    rgb_x,
    rgb_y,
    window_size,
    min_depth_m,
    max_depth_m,
):
    """Sample aligned depth at a color pixel without any manual x/y offset."""

    if depth_m is None:
        return {
            "valid": False,
            "distance_m": None,
            "rgb_x": int(round(rgb_x)),
            "rgb_y": int(round(rgb_y)),
            "valid_pixel_count": 0,
            "message": "No aligned depth frame available.",
        }

    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]

    if depth.ndim != 2:
        raise ValueError(f"Expected depth shape (H, W), got {depth.shape}.")

    rgb_x = int(round(rgb_x))
    rgb_y = int(round(rgb_y))

    result = {
        "valid": False,
        "distance_m": None,
        "rgb_x": rgb_x,
        "rgb_y": rgb_y,
        "valid_pixel_count": 0,
        "message": "No valid depth values near the clicked point.",
    }

    height, width = depth.shape
    if rgb_x < 0 or rgb_x >= width or rgb_y < 0 or rgb_y >= height:
        result["message"] = (
            f"RGB point ({rgb_x}, {rgb_y}) is outside the {width}x{height} depth frame."
        )
        return result

    window_size = max(1, int(window_size))
    if window_size % 2 == 0:
        window_size += 1

    half_window = window_size // 2

    # With aligned depth, the depth pixel uses the same x, y as the color pixel.
    x0 = max(0, rgb_x - half_window)
    x1 = min(width, rgb_x + half_window + 1)
    y0 = max(0, rgb_y - half_window)
    y1 = min(height, rgb_y + half_window + 1)

    depth_window = depth[y0:y1, x0:x1]
    valid_mask = (
        np.isfinite(depth_window)
        & (depth_window >= min_depth_m)
        & (depth_window <= max_depth_m)
    )
    valid_depths = depth_window[valid_mask]

    if valid_depths.size == 0:
        return result

    result["valid"] = True
    result["distance_m"] = float(np.median(valid_depths))
    result["valid_pixel_count"] = int(valid_depths.size)
    result["message"] = "OK"
    return result
