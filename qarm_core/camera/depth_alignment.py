"""Helpers for sampling distance from an aligned depth frame.

This module does not open cameras or display images. It assumes the depth frame
is already aligned to the RGB/color frame, such as depth returned by
RealSenseAlignedCamera.
"""

from __future__ import annotations

import numpy as np

from qarm_core import config


def make_depth_2d(depth_m):
    """Return depth_m as a 2D float32 array."""

    if depth_m is None:
        return None

    depth = np.asarray(depth_m, dtype=np.float32)

    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]

    if depth.ndim != 2:
        raise ValueError(f"Expected depth shape (H, W), got {depth.shape}.")

    return depth


def get_valid_depth_values_near_pixel(
    depth,
    depth_x,
    depth_y,
    window_size,
    min_depth_m,
    max_depth_m,
):
    """Return valid depth values from a small square window around a pixel."""

    height, width = depth.shape

    window_size = max(1, int(window_size))
    if window_size % 2 == 0:
        window_size += 1

    half_window = window_size // 2

    x0 = max(0, depth_x - half_window)
    x1 = min(width, depth_x + half_window + 1)
    y0 = max(0, depth_y - half_window)
    y1 = min(height, depth_y + half_window + 1)

    depth_window = depth[y0:y1, x0:x1]

    valid_mask = (
        np.isfinite(depth_window)
        & (depth_window >= min_depth_m)
        & (depth_window <= max_depth_m)
    )

    return depth_window[valid_mask]


def get_depth_at_rgb_pixel(
    depth_m,
    rgb_x,
    rgb_y,
    window_size=config.DEPTH_SAMPLE_WINDOW_SIZE,
    min_depth_m=config.DEPTH_MIN_M,
    max_depth_m=config.DEPTH_MAX_M,
):
    """Sample aligned depth at one RGB pixel.

    Alignment math:
    This function assumes depth_m is already aligned to the RGB/color frame.
    That means the depth pixel uses the same x, y coordinate as the RGB pixel:

        depth_x = rgb_x
        depth_y = rgb_y

    A small window is sampled because depth frames can contain noise, holes,
    NaN, infinity, or out-of-range values. The returned distance is the median
    of the valid depth values in that window.
    """

    rgb_x = int(round(rgb_x))
    rgb_y = int(round(rgb_y))
    depth_x = rgb_x
    depth_y = rgb_y

    result = {
        "valid": False,
        "distance_m": None,
        "rgb_x": rgb_x,
        "rgb_y": rgb_y,
        "depth_x": depth_x,
        "depth_y": depth_y,
        "valid_pixel_count": 0,
        "message": "No depth frame available.",
    }

    depth = make_depth_2d(depth_m)
    if depth is None:
        return result

    height, width = depth.shape
    if depth_x < 0 or depth_x >= width or depth_y < 0 or depth_y >= height:
        result["message"] = (
            f"RGB point ({rgb_x}, {rgb_y}) is outside the {width}x{height} depth frame."
        )
        return result

    valid_depths = get_valid_depth_values_near_pixel(
        depth,
        depth_x,
        depth_y,
        window_size,
        min_depth_m,
        max_depth_m,
    )

    if valid_depths.size == 0:
        result["message"] = "No valid depth values near the RGB point."
        return result

    result["valid"] = True
    result["distance_m"] = float(np.median(valid_depths))
    result["valid_pixel_count"] = int(valid_depths.size)
    result["message"] = "OK"
    return result


def get_depth_at_bbox_center(
    depth_m,
    bbox_xyxy,
    window_size=config.DEPTH_SAMPLE_WINDOW_SIZE,
    min_depth_m=config.DEPTH_MIN_M,
    max_depth_m=config.DEPTH_MAX_M,
):
    """Sample aligned depth at the center of a bounding box."""

    x1, y1, x2, y2 = bbox_xyxy

    # Center math: average the left/right and top/bottom edges of the box.
    rgb_x = int(round((x1 + x2) / 2))
    rgb_y = int(round((y1 + y2) / 2))

    return get_depth_at_rgb_pixel(
        depth_m,
        rgb_x,
        rgb_y,
        window_size,
        min_depth_m,
        max_depth_m,
    )
