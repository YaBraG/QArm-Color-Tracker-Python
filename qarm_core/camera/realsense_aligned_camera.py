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
