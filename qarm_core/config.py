"""Shared configuration values for the QArm color tracker project."""

import numpy as np


REAL_QARM_DEVICE_ID = 0
VIRTUAL_QARM_HIL_PORT = 18900
VIRTUAL_QARM_CAMERA_PORT = 18901

DEFAULT_SAMPLE_TIME = 0.05

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

DEPTH_MIN_M = 0.15
DEPTH_MAX_M = 3.0

# These offsets are only for the Quanser Camera3D/manual-offset backend.
# They are not used by the RealSense aligned-depth backend.
DEPTH_X_OFFSET = -25
DEPTH_Y_OFFSET = -3

HOME_POSE = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
SLEEP_POSE = np.array(
    [0.0, -17 * np.pi / 36, 15 * np.pi / 36, 0.0],
    dtype=np.float64,
)

GRIPPER_OPEN = np.array([0.1], dtype=np.float64)
GRIPPER_CLOSED = np.array([0.9], dtype=np.float64)

LED_OFF = np.array([0.0, 0.0, 0.0], dtype=np.float64)
LED_RED = np.array([1.0, 0.0, 0.0], dtype=np.float64)
LED_GREEN = np.array([0.0, 1.0, 0.0], dtype=np.float64)
LED_BLUE = np.array([0.0, 0.0, 1.0], dtype=np.float64)
