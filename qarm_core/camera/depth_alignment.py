"""Depth/RGB click-alignment test for the QArm RealSense camera.

Run from the repository root:

    python -m qarm_core.camera.depth_alignment --hardware 1

This file can also be run directly from VS Code:

    python qarm_core/camera/depth_alignment.py

Controls
--------
Left click        : select/update the RGB point.
Right click or c  : clear selected point.
a / d             : move depth sample point left/right.
w / s             : move depth sample point up/down.
r                 : reset depth offset to 0,0.
q or Esc          : quit.

What it tests
-------------
The RGB click point is where you visually choose the object.

The depth sample point is:

    depth_x = rgb_x + depth_x_offset
    depth_y = rgb_y + depth_y_offset

If the depth image is not perfectly aligned to the RGB image, adjust the
offset until the white marker in the depth window lands on the same object
region as the yellow RGB marker.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Direct-run import fix
# ---------------------------------------------------------------------------
# When VS Code runs this file directly, Python puts qarm_core/camera/ on
# sys.path instead of the repository root. That makes:
#
#     from qarm_core import config
#
# fail with ModuleNotFoundError. This block adds the repository root only when
# the file is being run directly.
if __package__ is None or __package__ == "":
    import sys
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[2]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

import argparse
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from qarm_core import config
from qarm_core.camera.qarm_camera import QArmCamera


RGB_WINDOW_NAME = "QArm RGB Depth Alignment Test"
DEPTH_WINDOW_NAME = "QArm Depth Display"


@dataclass
class DepthSample:
    """Result of sampling depth near one mapped depth pixel."""

    rgb_x: int
    rgb_y: int
    depth_x: int
    depth_y: int
    valid: bool
    distance_m: Optional[float] = None
    center_depth_m: Optional[float] = None
    valid_pixel_count: int = 0
    window_size: int = 0
    message: str = ""


def _as_depth_2d(depth_m: np.ndarray | None) -> np.ndarray | None:
    """Convert a depth frame into a 2D float32 array."""

    if depth_m is None:
        return None

    depth = np.asarray(depth_m, dtype=np.float32)

    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]

    if depth.ndim != 2:
        raise ValueError(
            f"Expected depth frame shape (H, W) or (H, W, 1), got {depth.shape}."
        )

    return depth


def is_valid_depth(
    value: float,
    min_depth_m: float = config.DEPTH_MIN_M,
    max_depth_m: float = config.DEPTH_MAX_M,
) -> bool:
    """Check whether one depth value is usable."""

    value = value
    return np.isfinite(value) and min_depth_m <= value <= max_depth_m


def depth_at_rgb_pixel(
    depth_m: np.ndarray | None,
    rgb_x: int,
    rgb_y: int,
    depth_x_offset: int = 0,
    depth_y_offset: int = 0,
    window_size: int = 7,
    min_depth_m: float = config.DEPTH_MIN_M,
    max_depth_m: float = config.DEPTH_MAX_M,
) -> DepthSample:
    """Sample depth near a clicked RGB pixel after applying manual alignment offset.

    This is a practical calibration helper.

    The clicked RGB coordinate is:

        rgb_x, rgb_y

    The actual sampled depth coordinate is:

        depth_x = rgb_x + depth_x_offset
        depth_y = rgb_y + depth_y_offset

    A small median window is used instead of one exact depth pixel because
    depth images often contain noise, holes, and invalid values.
    """

    depth = _as_depth_2d(depth_m)

    rgb_x = int(round(rgb_x))
    rgb_y = int(round(rgb_y))

    depth_x = int(round(rgb_x + depth_x_offset))
    depth_y = int(round(rgb_y + depth_y_offset))

    if depth is None:
        return DepthSample(
            rgb_x=rgb_x,
            rgb_y=rgb_y,
            depth_x=depth_x,
            depth_y=depth_y,
            valid=False,
            window_size=window_size,
            message="No depth frame available yet.",
        )

    height, width = depth.shape

    if depth_x < 0 or depth_x >= width or depth_y < 0 or depth_y >= height:
        return DepthSample(
            rgb_x=rgb_x,
            rgb_y=rgb_y,
            depth_x=depth_x,
            depth_y=depth_y,
            valid=False,
            window_size=window_size,
            message=(
                f"Mapped depth point ({depth_x}, {depth_y}) is outside "
                f"depth frame {width}x{height}."
            ),
        )

    window_size = max(1, window_size)
    if window_size % 2 == 0:
        window_size += 1

    half = window_size // 2

    x0 = max(0, depth_x - half)
    x1 = min(width, depth_x + half + 1)
    y0 = max(0, depth_y - half)
    y1 = min(height, depth_y + half + 1)

    roi = depth[y0:y1, x0:x1]

    valid_mask = (np.isfinite(roi) & (roi >= min_depth_m)) & (roi <= max_depth_m)
    valid_values = roi[valid_mask]

    center_value = float(depth[depth_y, depth_x])
    center_depth = (
        center_value
        if is_valid_depth(center_value, min_depth_m, max_depth_m)
        else None
    )

    if valid_values.size == 0:
        return DepthSample(
            rgb_x=rgb_x,
            rgb_y=rgb_y,
            depth_x=depth_x,
            depth_y=depth_y,
            valid=False,
            center_depth_m=center_depth,
            valid_pixel_count=0,
            window_size=window_size,
            message=(
                f"No valid depth near mapped point ({depth_x}, {depth_y}). "
                f"Range: {min_depth_m:.2f} m to {max_depth_m:.2f} m."
            ),
        )

    return DepthSample(
        rgb_x=rgb_x,
        rgb_y=rgb_y,
        depth_x=depth_x,
        depth_y=depth_y,
        valid=True,
        distance_m=float(np.median(valid_values)),
        center_depth_m=center_depth,
        valid_pixel_count=int(valid_values.size),
        window_size=window_size,
        message="OK",
    )


def depth_at_bbox_center(
    depth_m: np.ndarray | None,
    bbox_xyxy: tuple[int, int, int, int],
    depth_x_offset: int = 0,
    depth_y_offset: int = 0,
    window_size: int = 7,
    min_depth_m: float = config.DEPTH_MIN_M,
    max_depth_m: float = config.DEPTH_MAX_M,
) -> DepthSample:
    """Sample depth at the center of a bounding box.

    This is for later color-tracker / YOLO-style usage.
    """

    x1, y1, x2, y2 = bbox_xyxy

    rgb_x = int(round((x1 + x2) / 2.0))
    rgb_y = int(round((y1 + y2) / 2.0))

    return depth_at_rgb_pixel(
        depth_m=depth_m,
        rgb_x=rgb_x,
        rgb_y=rgb_y,
        depth_x_offset=depth_x_offset,
        depth_y_offset=depth_y_offset,
        window_size=window_size,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
    )


def _rgb_to_bgr_for_cv(rgb: np.ndarray) -> np.ndarray:
    """Return the QArm color frame in OpenCV display format.

    Quanser Camera3D opens the color stream as BGR even though many variables
    in this project are named RGB. OpenCV also expects BGR, so do not convert.
    """

    frame = np.asarray(rgb)

    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError(f"Expected color frame shape (H, W, 3), got {frame.shape}.")

    return frame.copy()


def _depth_to_display(
    depth_m: np.ndarray | None,
    min_depth_m: float,
    max_depth_m: float,
) -> np.ndarray | None:
    """Create a uint8 depth image where closer objects appear brighter."""

    depth = _as_depth_2d(depth_m)

    if depth is None:
        return None

    display = np.zeros(depth.shape, dtype=np.uint8)

    valid = (np.isfinite(depth) & (depth >= min_depth_m)) & (depth <= max_depth_m)

    if not np.any(valid):
        return display

    depth_range = max_depth_m - min_depth_m
    if depth_range <= 0:
        return display

    scaled = 255.0 * (max_depth_m - depth[valid]) / depth_range
    display[valid] = np.clip(scaled, 0, 255).astype(np.uint8)

    return display


def _draw_rgb_overlay(
    frame_bgr: np.ndarray,
    selected_point: tuple[int, int] | None,
    sample: DepthSample | None,
    depth_x_offset: int,
    depth_y_offset: int,
) -> np.ndarray:
    """Draw selected RGB point and current depth information."""

    display = frame_bgr.copy()

    cv2.rectangle(display, (8, 8), (display.shape[1] - 8, 62), (0, 0, 0), -1)

    if selected_point is None:
        text = "Left click object point | q/Esc quit | c clear"
        cv2.putText(
            display,
            text,
            (15, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return display

    rgb_x, rgb_y = selected_point

    # Yellow marker on RGB view = point selected by the user.
    cv2.drawMarker(
        display,
        (rgb_x, rgb_y),
        (0, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=22,
        thickness=2,
    )
    cv2.circle(display, (rgb_x, rgb_y), 8, (0, 255, 255), 2)

    if sample is not None and sample.valid and sample.distance_m is not None:
        text = (
            f"RGB ({sample.rgb_x},{sample.rgb_y}) -> "
            f"Depth ({sample.depth_x},{sample.depth_y}) | "
            f"{sample.distance_m:.3f} m | "
            f"offset ({depth_x_offset:+d},{depth_y_offset:+d})"
        )
    elif sample is not None:
        text = (
            f"RGB ({rgb_x},{rgb_y}) | {sample.message} | "
            f"offset ({depth_x_offset:+d},{depth_y_offset:+d})"
        )
    else:
        text = (
            f"RGB ({rgb_x},{rgb_y}) | waiting for depth | "
            f"offset ({depth_x_offset:+d},{depth_y_offset:+d})"
        )

    cv2.putText(
        display,
        text,
        (15, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.54,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    return display


def _draw_depth_overlay(
    depth_display: np.ndarray,
    sample: DepthSample | None,
    selected_point: tuple[int, int] | None,
) -> np.ndarray:
    """Draw RGB point and mapped depth sample point on the depth image."""

    depth_color = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)

    if selected_point is not None:
        rgb_x, rgb_y = selected_point

        # Yellow = original RGB coordinate copied directly onto depth image.
        cv2.drawMarker(
            depth_color,
            (rgb_x, rgb_y),
            (0, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=16,
            thickness=2,
        )

    if sample is not None:
        # White = actual depth sample coordinate after offset.
        cv2.drawMarker(
            depth_color,
            (sample.depth_x, sample.depth_y),
            (255, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2,
        )
        cv2.circle(depth_color, (sample.depth_x, sample.depth_y), 8, (255, 255, 255), 2)

    cv2.rectangle(depth_color, (8, 8), (depth_color.shape[1] - 8, 62), (0, 0, 0), -1)

    cv2.putText(
        depth_color,
        "Yellow=RGB pixel | White=sampled depth pixel",
        (15, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.54,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    return depth_color


def run_click_depth_test(args: argparse.Namespace) -> None:
    """Open the camera and run the mouse-click depth alignment test."""

    selected_point: list[tuple[int, int] | None] = [None]
    latest_sample: list[DepthSample | None] = [None]

    depth_x_offset = int(args.depth_x_offset)
    depth_y_offset = int(args.depth_y_offset)

    last_print_time = 0.0
    last_print_distance: Optional[float] = None

    def on_mouse(event, x, y, flags, userdata):  # noqa: ARG001
        if event == cv2.EVENT_LBUTTONDOWN:
            selected_point[0] = (int(x), int(y))
            latest_sample[0] = None
            print(f"Selected RGB pixel: ({x}, {y})")

        elif event == cv2.EVENT_RBUTTONDOWN:
            selected_point[0] = None
            latest_sample[0] = None
            print("Cleared selected point.")

    camera = QArmCamera(
        hardware=int(args.hardware),
        device_id=int(args.device_id),
        video_port=int(args.video_port),
        width=int(args.width),
        height=int(args.height),
        fps=int(args.fps),
        depth_min_m=float(args.min_depth),
        depth_max_m=float(args.max_depth),
    )

    cv2.namedWindow(RGB_WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(RGB_WINDOW_NAME, on_mouse)

    if args.show_depth:
        cv2.namedWindow(DEPTH_WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        camera.open()

        print("")
        print("QArm RGB/depth click alignment test running.")
        print("Left click an object in the RGB window.")
        print("Yellow marker = raw RGB coordinate.")
        print("White marker  = actual sampled depth coordinate after offset.")
        print("")
        print("Controls:")
        print("  a/d = move sampled depth point left/right")
        print("  w/s = move sampled depth point up/down")
        print("  r   = reset offset")
        print("  c   = clear selected point")
        print("  q or Esc = quit")
        print("")
        print(f"Initial depth offset: ({depth_x_offset:+d}, {depth_y_offset:+d})")
        print("")

        while True:
            rgb, depth_m = camera.read()

            if rgb is None:
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                continue

            frame_bgr = _rgb_to_bgr_for_cv(rgb)

            if selected_point[0] is not None:
                sample = depth_at_rgb_pixel(
                    depth_m=depth_m,
                    rgb_x=selected_point[0][0],
                    rgb_y=selected_point[0][1],
                    depth_x_offset=depth_x_offset,
                    depth_y_offset=depth_y_offset,
                    window_size=int(args.window_size),
                    min_depth_m=float(args.min_depth),
                    max_depth_m=float(args.max_depth),
                )
                latest_sample[0] = sample

                now = time.time()
                should_print = now - last_print_time >= float(args.print_interval)

                if sample.valid and sample.distance_m is not None:
                    changed_enough = (
                        last_print_distance is None
                        or abs(sample.distance_m - last_print_distance)
                        >= float(args.print_delta)
                    )

                    if should_print and changed_enough:
                        print(
                            f"RGB ({sample.rgb_x}, {sample.rgb_y}) -> "
                            f"Depth ({sample.depth_x}, {sample.depth_y}) | "
                            f"{sample.distance_m:.3f} m | "
                            f"offset ({depth_x_offset:+d}, {depth_y_offset:+d}) | "
                            f"{sample.valid_pixel_count} valid px"
                        )
                        last_print_time = now
                        last_print_distance = sample.distance_m

                elif should_print:
                    print(sample.message)
                    last_print_time = now

            rgb_display = _draw_rgb_overlay(
                frame_bgr=frame_bgr,
                selected_point=selected_point[0],
                sample=latest_sample[0],
                depth_x_offset=depth_x_offset,
                depth_y_offset=depth_y_offset,
            )
            cv2.imshow(RGB_WINDOW_NAME, rgb_display)

            if args.show_depth:
                depth_display = _depth_to_display(depth_m, args.min_depth, args.max_depth)

                if depth_display is not None:
                    depth_color = _draw_depth_overlay(
                        depth_display=depth_display,
                        sample=latest_sample[0],
                        selected_point=selected_point[0],
                    )
                    cv2.imshow(DEPTH_WINDOW_NAME, depth_color)

            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q")):
                break

            if key == ord("c"):
                selected_point[0] = None
                latest_sample[0] = None
                print("Cleared selected point.")

            elif key == ord("r"):
                depth_x_offset = 0
                depth_y_offset = 0
                print("Reset depth offset to (0, 0).")

            elif key == ord("a"):
                depth_x_offset -= 1
                print(f"Depth offset: ({depth_x_offset:+d}, {depth_y_offset:+d})")

            elif key == ord("d"):
                depth_x_offset += 1
                print(f"Depth offset: ({depth_x_offset:+d}, {depth_y_offset:+d})")

            elif key == ord("w"):
                depth_y_offset -= 1
                print(f"Depth offset: ({depth_x_offset:+d}, {depth_y_offset:+d})")

            elif key == ord("s"):
                depth_y_offset += 1
                print(f"Depth offset: ({depth_x_offset:+d}, {depth_y_offset:+d})")

    finally:
        camera.close()
        cv2.destroyAllWindows()


def build_arg_parser() -> argparse.ArgumentParser:
    """Create CLI arguments for the click-depth alignment test."""

    parser = argparse.ArgumentParser(
        description="Click an RGB pixel and report QArm RealSense depth at that point."
    )

    parser.add_argument(
        "--hardware",
        type=int,
        default=1,
        choices=(0, 1),
        help="1 for physical QArm RealSense, 0 for virtual camera.",
    )

    parser.add_argument(
        "--device-id",
        type=int,
        default=config.REAL_QARM_DEVICE_ID,
        help="Physical RealSense device ID.",
    )

    parser.add_argument(
        "--video-port",
        type=int,
        default=config.VIRTUAL_QARM_CAMERA_PORT,
        help="Virtual QArm camera port.",
    )

    parser.add_argument(
        "--width",
        type=int,
        default=config.CAMERA_WIDTH,
        help="Camera frame width.",
    )

    parser.add_argument(
        "--height",
        type=int,
        default=config.CAMERA_HEIGHT,
        help="Camera frame height.",
    )

    parser.add_argument(
        "--fps",
        type=int,
        default=config.CAMERA_FPS,
        help="Camera FPS.",
    )

    parser.add_argument(
        "--window-size",
        type=int,
        default=7,
        help="Odd-sized pixel window used for median depth sampling.",
    )

    parser.add_argument(
        "--min-depth",
        type=float,
        default=config.DEPTH_MIN_M,
        help="Minimum valid depth in meters.",
    )

    parser.add_argument(
        "--max-depth",
        type=float,
        default=config.DEPTH_MAX_M,
        help="Maximum valid depth in meters.",
    )

    parser.add_argument(
        "--depth-x-offset",
        type=int,
        default=0,
        help="Initial X offset from RGB pixel to depth pixel.",
    )

    parser.add_argument(
        "--depth-y-offset",
        type=int,
        default=0,
        help="Initial Y offset from RGB pixel to depth pixel.",
    )

    parser.add_argument(
        "--print-interval",
        type=float,
        default=0.25,
        help="Minimum seconds between console depth reports.",
    )

    parser.add_argument(
        "--print-delta",
        type=float,
        default=0.01,
        help="Minimum depth change in meters before printing another valid sample.",
    )

    parser.add_argument(
        "--hide-depth",
        dest="show_depth",
        action="store_false",
        help="Hide the depth-display window.",
    )

    parser.set_defaults(show_depth=True)

    return parser


def main() -> None:
    """CLI entry point."""

    args = build_arg_parser().parse_args()
    run_click_depth_test(args)


if __name__ == "__main__":
    main()