"""RGB/depth alignment tool for the QArm RealSense camera.

Run from the repository root:

    python -m qarm_core.camera.depth_alignment --hardware 1

This file can also be run directly from VS Code:

    python qarm_core/camera/depth_alignment.py

Controls
--------
Left click        : select an RGB point.
Right click or c  : clear the selected point.
q or Esc          : quit.

Quanser backend only:
a / d             : decrease/increase depth_x_offset.
w / s             : decrease/increase depth_y_offset.
r                 : reset offset to qarm_core.config values.
p                 : save current offset to qarm_core/config.py.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Direct-run import fix
# ---------------------------------------------------------------------------
# When VS Code runs this file directly, Python starts in qarm_core/camera/.
# Add the repository root so imports like "from qarm_core import config" work.
if __package__ is None or __package__ == "":
    import sys
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[2]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

import argparse
import re
import time
from pathlib import Path

import cv2
import numpy as np

from qarm_core import config
from qarm_core.camera.qarm_camera import QArmCamera
from qarm_core.camera.realsense_aligned_camera import (
    RealSenseAlignedCamera,
    get_depth_at_rgb_pixel as get_aligned_depth_at_rgb_pixel,
)


RGB_WINDOW_NAME = "QArm RGB Alignment"
DEPTH_WINDOW_NAME = "QArm Depth Alignment"


def make_depth_2d(depth_m):
    """Return depth as a 2D float array, or None if no depth frame exists."""

    if depth_m is None:
        return None

    depth = np.asarray(depth_m, dtype=np.float32)

    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]

    if depth.ndim != 2:
        raise ValueError(f"Expected depth shape (H, W), got {depth.shape}.")

    return depth


def get_depth_at_rgb_pixel(
    depth_m,
    rgb_x,
    rgb_y,
    depth_x_offset,
    depth_y_offset,
    window_size,
    min_depth_m,
    max_depth_m,
):
    """Sample depth near an RGB pixel after applying the current offset.

    Alignment math:
    The user clicks a point in the RGB image. The matching point in the depth
    image is shifted by the calibration offset:

        depth_x = rgb_x + depth_x_offset
        depth_y = rgb_y + depth_y_offset

    A small window is sampled because depth frames can have noise or missing
    pixels. Invalid values are ignored, and the median valid depth is returned.
    """

    depth = make_depth_2d(depth_m)

    rgb_x = int(round(rgb_x))
    rgb_y = int(round(rgb_y))
    depth_x = int(round(rgb_x + depth_x_offset))
    depth_y = int(round(rgb_y + depth_y_offset))

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

    if depth is None:
        return result

    height, width = depth.shape
    if depth_x < 0 or depth_x >= width or depth_y < 0 or depth_y >= height:
        result["message"] = (
            f"Mapped depth point ({depth_x}, {depth_y}) is outside "
            f"the {width}x{height} depth frame."
        )
        return result

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
    valid_depths = depth_window[valid_mask]

    if valid_depths.size == 0:
        result["message"] = "No valid depth values near the mapped point."
        return result

    result["valid"] = True
    result["distance_m"] = float(np.median(valid_depths))
    result["valid_pixel_count"] = int(valid_depths.size)
    result["message"] = "OK"
    return result


# Older code may import this shorter name. Keep it as a simple alias.
depth_at_rgb_pixel = get_depth_at_rgb_pixel


def color_frame_for_display(rgb_frame):
    """Return the camera color frame in OpenCV display format.

    Quanser Camera3D appears to provide this frame in BGR order already.
    OpenCV also displays BGR, so the simplest correct path is to display the
    frame directly without converting colors.
    """

    frame = np.asarray(rgb_frame)

    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError(f"Expected color frame shape (H, W, 3), got {frame.shape}.")

    return frame.copy()


def make_depth_display(depth_m, min_depth_m, max_depth_m):
    """Create a colorized depth image for display only.

    This changes only the visualization. The real depth values in meters are
    still read from the original depth_m frame when distance is sampled.
    """

    depth = make_depth_2d(depth_m)
    if depth is None:
        return None

    display_gray = np.zeros(depth.shape, dtype=np.uint8)
    valid = np.isfinite(depth) & (depth >= min_depth_m) & (depth <= max_depth_m)

    if np.any(valid):
        depth_range = max_depth_m - min_depth_m
        if depth_range > 0:
            # Closer valid pixels become brighter. Farther valid pixels are darker.
            scaled = 255.0 * (max_depth_m - depth[valid]) / depth_range
            display_gray[valid] = np.clip(scaled, 0, 255).astype(np.uint8)

    return cv2.applyColorMap(display_gray, cv2.COLORMAP_TURBO)


def draw_rgb_view(
    frame_bgr,
    selected_point,
    sample,
    depth_x_offset,
    depth_y_offset,
    backend,
):
    """Draw the clicked point and current depth result on the RGB image."""

    display = frame_bgr.copy()
    cv2.rectangle(display, (8, 8), (display.shape[1] - 8, 68), (0, 0, 0), -1)

    if selected_point is None:
        if backend == "quanser":
            text = "Left click to sample depth | a/d/w/s adjust | p save | q quit"
        else:
            text = "Left click to sample aligned depth | c clear | q quit"
    else:
        rgb_x, rgb_y = selected_point
        cv2.drawMarker(display, (rgb_x, rgb_y), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.circle(display, (rgb_x, rgb_y), 8, (0, 255, 255), 2)

        if sample is not None and sample["valid"]:
            if backend == "quanser":
                text = (
                    f"RGB ({sample['rgb_x']},{sample['rgb_y']}) -> "
                    f"Depth ({sample['depth_x']},{sample['depth_y']}) | "
                    f"{sample['distance_m']:.3f} m | "
                    f"offset ({depth_x_offset:+d},{depth_y_offset:+d})"
                )
            else:
                text = (
                    f"Aligned RGB/depth ({sample['rgb_x']},{sample['rgb_y']}) | "
                    f"{sample['distance_m']:.3f} m"
                )
        elif sample is not None:
            if backend == "quanser":
                text = f"{sample['message']} | offset ({depth_x_offset:+d},{depth_y_offset:+d})"
            else:
                text = sample["message"]
        else:
            if backend == "quanser":
                text = f"Waiting for depth | offset ({depth_x_offset:+d},{depth_y_offset:+d})"
            else:
                text = "Waiting for aligned depth"

    cv2.putText(
        display,
        text,
        (16, 45),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return display


def draw_depth_view(depth_color, selected_point, sample, backend):
    """Draw the RGB point and offset depth point on the depth display."""

    display = depth_color.copy()

    if selected_point is not None:
        rgb_x, rgb_y = selected_point
        cv2.drawMarker(display, (rgb_x, rgb_y), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)

    if sample is not None:
        # Quanser mode uses a shifted depth point. RealSense aligned mode uses
        # the same x, y for the RGB pixel and depth pixel.
        depth_x = sample.get("depth_x", sample["rgb_x"])
        depth_y = sample.get("depth_y", sample["rgb_y"])
        cv2.drawMarker(display, (depth_x, depth_y), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.circle(display, (depth_x, depth_y), 8, (255, 255, 255), 2)

    cv2.rectangle(display, (8, 8), (display.shape[1] - 8, 62), (0, 0, 0), -1)
    if backend == "quanser":
        depth_text = "Yellow = RGB click | White = sampled depth pixel"
    else:
        depth_text = "Aligned RealSense depth: yellow and white should overlap"

    cv2.putText(
        display,
        depth_text,
        (16, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.54,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return display


def save_depth_offset_to_config(depth_x_offset, depth_y_offset):
    """Save DEPTH_X_OFFSET and DEPTH_Y_OFFSET in qarm_core/config.py."""

    config_path = Path(config.__file__).resolve()
    text = config_path.read_text(encoding="utf-8")

    text, found_x = re.subn(
        r"^DEPTH_X_OFFSET\s*=\s*[-+]?\d+",
        f"DEPTH_X_OFFSET = {int(depth_x_offset)}",
        text,
        flags=re.MULTILINE,
    )
    text, found_y = re.subn(
        r"^DEPTH_Y_OFFSET\s*=\s*[-+]?\d+",
        f"DEPTH_Y_OFFSET = {int(depth_y_offset)}",
        text,
        flags=re.MULTILINE,
    )

    if found_x == 0 or found_y == 0:
        lines_to_add = []
        if found_x == 0:
            lines_to_add.append(f"DEPTH_X_OFFSET = {int(depth_x_offset)}")
        if found_y == 0:
            lines_to_add.append(f"DEPTH_Y_OFFSET = {int(depth_y_offset)}")

        extra_text = "\n# Manual RGB-to-depth alignment offset.\n" + "\n".join(lines_to_add) + "\n"
        text = text.rstrip() + "\n" + extra_text

    config_path.write_text(text, encoding="utf-8")
    config.DEPTH_X_OFFSET = int(depth_x_offset)
    config.DEPTH_Y_OFFSET = int(depth_y_offset)

    print(
        "Saved depth offset to qarm_core/config.py: "
        f"DEPTH_X_OFFSET={int(depth_x_offset)}, DEPTH_Y_OFFSET={int(depth_y_offset)}"
    )


def print_controls(backend, depth_x_offset, depth_y_offset):
    """Print controls and the starting offset."""

    print("")
    print("QArm RGB/depth alignment tool running.")
    print(f"Camera backend: {backend}")
    print("Left click an object in the RGB window.")

    if backend == "quanser":
        print("Quanser Camera3D manual offset mode.")
        print("Depth mapping: depth_x = rgb_x + depth_x_offset")
        print("               depth_y = rgb_y + depth_y_offset")
    else:
        print("RealSense aligned-depth mode.")
        print("Depth mapping: aligned_depth_m[rgb_y, rgb_x]")
    print("")
    print("Controls:")
    print("  Left click       select RGB point")
    print("  Right click or c clear selected point")
    if backend == "quanser":
        print("  a / d            decrease/increase depth_x_offset")
        print("  w / s            decrease/increase depth_y_offset")
        print("  r                reset offset to config values")
        print("  p                save current offset to qarm_core/config.py")
    else:
        print("  a/d/w/s/r/p      no manual offset is used in RealSense aligned mode")
    print("  q or Esc         quit")
    print("")
    if backend == "quanser":
        print(f"Starting depth offset: ({depth_x_offset:+d}, {depth_y_offset:+d})")
    print("")


def run_depth_alignment_tool(args):
    """Open the camera and run the click-to-sample alignment tool."""

    selected_point = [None]
    latest_sample = [None]
    last_print_time = [0.0]

    depth_x_offset = int(args.depth_x_offset)
    depth_y_offset = int(args.depth_y_offset)
    backend = args.backend

    def on_mouse(event, x, y, flags, userdata):  # noqa: ARG001
        if event == cv2.EVENT_LBUTTONDOWN:
            selected_point[0] = (int(x), int(y))
            latest_sample[0] = None
            print(f"Selected RGB pixel: ({int(x)}, {int(y)})")
        elif event == cv2.EVENT_RBUTTONDOWN:
            selected_point[0] = None
            latest_sample[0] = None
            print("Cleared selected point.")

    if backend == "quanser":
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
    else:
        camera = RealSenseAlignedCamera(
            width=int(args.width),
            height=int(args.height),
            fps=int(args.fps),
        )

    cv2.namedWindow(RGB_WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(RGB_WINDOW_NAME, on_mouse)

    if args.show_depth:
        cv2.namedWindow(DEPTH_WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        camera.open()
        print_controls(backend, depth_x_offset, depth_y_offset)

        while True:
            rgb_frame, depth_m = camera.read()

            if rgb_frame is None:
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                continue

            frame_bgr = color_frame_for_display(rgb_frame)

            if selected_point[0] is not None:
                rgb_x, rgb_y = selected_point[0]
                if backend == "quanser":
                    sample = get_depth_at_rgb_pixel(
                        depth_m,
                        rgb_x,
                        rgb_y,
                        depth_x_offset,
                        depth_y_offset,
                        int(args.window_size),
                        float(args.min_depth),
                        float(args.max_depth),
                    )
                else:
                    sample = get_aligned_depth_at_rgb_pixel(
                        depth_m,
                        rgb_x,
                        rgb_y,
                        int(args.window_size),
                        float(args.min_depth),
                        float(args.max_depth),
                    )
                latest_sample[0] = sample

                if time.time() - last_print_time[0] >= float(args.print_interval):
                    if sample["valid"]:
                        if backend == "quanser":
                            print(
                                f"RGB ({sample['rgb_x']}, {sample['rgb_y']}) -> "
                                f"Depth ({sample['depth_x']}, {sample['depth_y']}) | "
                                f"{sample['distance_m']:.3f} m | "
                                f"offset ({depth_x_offset:+d}, {depth_y_offset:+d}) | "
                                f"{sample['valid_pixel_count']} valid px"
                            )
                        else:
                            print(
                                f"Aligned depth at RGB ({sample['rgb_x']}, {sample['rgb_y']}) | "
                                f"{sample['distance_m']:.3f} m | "
                                f"{sample['valid_pixel_count']} valid px"
                            )
                    else:
                        print(sample["message"])
                    last_print_time[0] = time.time()

            rgb_display = draw_rgb_view(
                frame_bgr,
                selected_point[0],
                latest_sample[0],
                depth_x_offset,
                depth_y_offset,
                backend,
            )
            cv2.imshow(RGB_WINDOW_NAME, rgb_display)

            if args.show_depth:
                depth_color = make_depth_display(depth_m, args.min_depth, args.max_depth)
                if depth_color is not None:
                    depth_display = draw_depth_view(
                        depth_color,
                        selected_point[0],
                        latest_sample[0],
                        backend,
                    )
                    cv2.imshow(DEPTH_WINDOW_NAME, depth_display)

            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q")):
                break
            elif key == ord("c"):
                selected_point[0] = None
                latest_sample[0] = None
                print("Cleared selected point.")
            elif key == ord("a"):
                if backend == "quanser":
                    depth_x_offset -= 1
                    print(f"Depth offset: ({depth_x_offset:+d}, {depth_y_offset:+d})")
                else:
                    print("RealSense backend is already aligned; manual offset is not used.")
            elif key == ord("d"):
                if backend == "quanser":
                    depth_x_offset += 1
                    print(f"Depth offset: ({depth_x_offset:+d}, {depth_y_offset:+d})")
                else:
                    print("RealSense backend is already aligned; manual offset is not used.")
            elif key == ord("w"):
                if backend == "quanser":
                    depth_y_offset -= 1
                    print(f"Depth offset: ({depth_x_offset:+d}, {depth_y_offset:+d})")
                else:
                    print("RealSense backend is already aligned; manual offset is not used.")
            elif key == ord("s"):
                if backend == "quanser":
                    depth_y_offset += 1
                    print(f"Depth offset: ({depth_x_offset:+d}, {depth_y_offset:+d})")
                else:
                    print("RealSense backend is already aligned; manual offset is not used.")
            elif key == ord("r"):
                if backend == "quanser":
                    depth_x_offset = int(config.DEPTH_X_OFFSET)
                    depth_y_offset = int(config.DEPTH_Y_OFFSET)
                    print(f"Reset depth offset to config: ({depth_x_offset:+d}, {depth_y_offset:+d})")
                else:
                    print("RealSense backend is already aligned; manual offset is not used.")
            elif key == ord("p"):
                if backend == "quanser":
                    save_depth_offset_to_config(depth_x_offset, depth_y_offset)
                else:
                    print("RealSense backend is already aligned; there is no manual offset to save.")

    finally:
        camera.close()
        cv2.destroyAllWindows()


def build_arg_parser():
    """Create command-line options for the alignment tool."""

    parser = argparse.ArgumentParser(
        description="Click an RGB pixel and sample depth at the offset depth pixel."
    )
    parser.add_argument(
        "--backend",
        choices=("quanser", "realsense"),
        default="quanser",
        help="Camera backend: quanser keeps manual offsets, realsense uses aligned depth.",
    )
    parser.add_argument("--hardware", type=int, default=1, choices=(0, 1))
    parser.add_argument("--device-id", type=int, default=config.REAL_QARM_DEVICE_ID)
    parser.add_argument("--video-port", type=int, default=config.VIRTUAL_QARM_CAMERA_PORT)
    parser.add_argument("--width", type=int, default=config.CAMERA_WIDTH)
    parser.add_argument("--height", type=int, default=config.CAMERA_HEIGHT)
    parser.add_argument("--fps", type=int, default=config.CAMERA_FPS)
    parser.add_argument("--window-size", type=int, default=7)
    parser.add_argument("--min-depth", type=float, default=config.DEPTH_MIN_M)
    parser.add_argument("--max-depth", type=float, default=config.DEPTH_MAX_M)
    parser.add_argument("--depth-x-offset", type=int, default=config.DEPTH_X_OFFSET)
    parser.add_argument("--depth-y-offset", type=int, default=config.DEPTH_Y_OFFSET)
    parser.add_argument("--print-interval", type=float, default=0.25)
    parser.add_argument("--hide-depth", dest="show_depth", action="store_false")
    parser.set_defaults(show_depth=True)
    return parser


def main():
    """Run the depth alignment command-line tool."""

    args = build_arg_parser().parse_args()
    run_depth_alignment_tool(args)


if __name__ == "__main__":
    main()
