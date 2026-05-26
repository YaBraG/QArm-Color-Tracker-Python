"""Camera wrapper for QArm RealSense streams."""

from __future__ import annotations

import numpy as np

from qarm import QArmRealSense
from qarm_core import config


class QArmCamera:
    """Lazy-opening wrapper around the existing QArmRealSense class."""

    def __init__(
        self,
        hardware: int = 1,
        device_id: int = 0,
        video_port: int = config.VIRTUAL_QARM_CAMERA_PORT,
        mode: str = "RGB&DEPTH",
        width: int = config.CAMERA_WIDTH,
        height: int = config.CAMERA_HEIGHT,
        fps: int = config.CAMERA_FPS,
        depth_min_m: float = config.DEPTH_MIN_M,
        depth_max_m: float = config.DEPTH_MAX_M,
    ):
        self.hardware = hardware
        self.device_id = device_id
        self.video_port = video_port
        self.mode = mode
        self.width = width
        self.height = height
        self.fps = fps
        self.depth_min_m = depth_min_m
        self.depth_max_m = depth_max_m

        self.camera = None
        self.last_rgb = None
        self.last_depth_m = None
        self.last_depth_display = None

    def open(self) -> None:
        """Open the camera stream using the stored settings."""

        self.camera = QArmRealSense(
            hardware=self.hardware,
            deviceID=self.device_id,
            videoPort=self.video_port,
            readMode=1,
            mode=self.mode,
            frameWidthRGB=self.width,
            frameHeightRGB=self.height,
            frameRateRGB=self.fps,
            frameWidthDepth=self.width,
            frameHeightDepth=self.height,
            frameRateDepth=self.fps,
        )

        if self.hardware:
            print(f"Opened physical QArm RealSense camera {self.device_id}.")
        else:
            print(f"Opened virtual QArm camera on port {self.video_port}.")

    def close(self) -> None:
        """Close the camera stream if it is open."""

        if self.camera is not None:
            self.camera.terminate()
            self.camera = None

    def read_rgb(self) -> np.ndarray | None:
        """Read the newest RGB frame, keeping the last valid frame."""

        if self.camera is None:
            return self.last_rgb

        timestamp = self.camera.read_RGB()
        if timestamp != -1:
            self.last_rgb = self.camera.imageBufferRGB.copy()

        return self.last_rgb

    def read_depth_m(self) -> np.ndarray | None:
        """Read depth in meters, mainly for the physical camera."""

        if self.camera is None:
            return self.last_depth_m

        timestamp = self.camera.read_depth(dataMode="M")
        if timestamp != -1:
            self.last_depth_m = self.camera.imageBufferDepthM.copy()

        return self.last_depth_m

    def read_depth_px(self) -> np.ndarray | None:
        """Read raw depth pixels, mainly for the virtual camera."""

        if self.camera is None:
            return None

        timestamp = self.camera.read_depth(dataMode="PX")
        if timestamp != -1:
            return self.camera.imageBufferDepthPX.copy()

        return None

    def read(self):
        """Read RGB and depth-meter frames."""

        rgb = self.read_rgb()
        depth_m = self.read_depth_m()
        return rgb, depth_m

    def depth_m_to_display(self, depth_m) -> np.ndarray:
        """Convert depth meters to a uint8 image with near objects brighter."""

        depth_m = np.asarray(depth_m, dtype=np.float32)
        display = np.zeros(depth_m.shape, dtype=np.uint8)

        valid = (
            np.isfinite(depth_m)
            & (depth_m >= self.depth_min_m)
            & (depth_m <= self.depth_max_m)
        )

        if not np.any(valid):
            return display

        depth_range = self.depth_max_m - self.depth_min_m
        if depth_range <= 0:
            return display

        scaled = 255.0 * (self.depth_max_m - depth_m[valid]) / depth_range
        display[valid] = np.clip(scaled, 0, 255).astype(np.uint8)
        return display

    def depth_px_to_display(self, depth_px) -> np.ndarray:
        """Convert virtual depth pixels to a uint8 display image manually."""

        depth_px = np.asarray(depth_px, dtype=np.float32)
        valid = np.isfinite(depth_px)

        if not np.any(valid):
            return np.zeros(depth_px.shape, dtype=np.uint8)

        min_value = float(np.min(depth_px[valid]))
        max_value = float(np.max(depth_px[valid]))

        if max_value <= min_value:
            return np.zeros(depth_px.shape, dtype=np.uint8)

        scaled = 255.0 * (depth_px - min_value) / (max_value - min_value)
        scaled[~valid] = 0
        return np.clip(scaled, 0, 255).astype(np.uint8)

    def read_display_frames(self):
        """Read RGB and depth frames for display without blinking on dropouts."""

        rgb, depth_m = self.read()

        if depth_m is not None:
            self.last_depth_display = self.depth_m_to_display(depth_m)

        return rgb, self.last_depth_display

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

