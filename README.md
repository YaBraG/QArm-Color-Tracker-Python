# QArm Color Tracker Python

Python support code for a Quanser QArm color tracking project. The repository
currently focuses on QArm connection helpers, virtual/physical arm support, and
camera calibration tools. Color tracking is planned, but it is not implemented
yet in the current codebase.

## Current Project State

- `qarm.py` provides physical Quanser QArm support.
- `virtual_qarm.py` provides QLabs Virtual QArm support.
- `qarm_mimic.py` mirrors physical QArm joint positions into QLabs.
- `qarm_core/` is the clean reusable library package for new project code.
- `qarm_core/camera/qarm_camera.py` reads Quanser Camera3D physical or virtual
  camera frames.
- `qarm_core/camera/realsense_aligned_camera.py` reads physical RealSense frames
  with depth aligned to the color image by `pyrealsense2`.
- `qarm_core/camera/depth_alignment.py` tests depth at a clicked RGB pixel.

The root scripts are kept in place so existing classroom or lab workflows still
run. New reusable code should go under `qarm_core/`.

## Camera Notes

The physical RealSense camera is read in Python/OpenCV. Physical RealSense
frames do not appear inside the black camera panels in QLabs; those panels are
for QLabs virtual camera streams.

`qarm_core/camera/qarm_camera.py` wraps the existing `QArmRealSense` class from
Quanser Camera3D and keeps the last valid RGB/depth frames so displays do not
blink during brief camera read dropouts.

`qarm_core/camera/realsense_aligned_camera.py` uses `pyrealsense2` directly for
physical RealSense aligned-depth reads. In this mode, the returned
`aligned_depth_m[y, x]` value matches the color pixel at `x, y`.

## RGB-Depth Alignment

The RealSense RGB stream and depth stream are close, but not perfectly aligned
when read through the Quanser Camera3D/manual-offset path. Manual testing found
that a fixed offset works at one distance but changes significantly at another
distance. This happens because RGB/depth parallax depends on object depth, so a
fixed pixel offset is only a temporary approximation.

The project keeps a manual pixel offset in `qarm_core/config.py` for Quanser
Camera3D debugging:

```python
DEPTH_X_OFFSET = -25
DEPTH_Y_OFFSET = -3
```

The Quanser backend maps a clicked RGB pixel to a depth pixel like this:

```python
depth_x = rgb_x + DEPTH_X_OFFSET
depth_y = rgb_y + DEPTH_Y_OFFSET
```

The current saved/default Quanser manual-offset calibration is
`DEPTH_X_OFFSET=-25` and `DEPTH_Y_OFFSET=-3` unless `qarm_core/config.py` is
changed.

The depth alignment tool has two backend modes:

- `quanser`: uses Quanser Camera3D and the manual offset values above.
- `realsense`: uses `pyrealsense2` with RealSense depth aligned to color pixels.

Recommended physical distance tracking mode:

```bash
python -m qarm_core.camera.depth_alignment --hardware 1 --backend realsense
```

Quanser manual-offset debugging mode:

```bash
python -m qarm_core.camera.depth_alignment --hardware 1 --backend quanser
```

It can also be run directly from VS Code:

```bash
python qarm_core/camera/depth_alignment.py
```

If `pyrealsense2` is not installed and you want to use the RealSense aligned
backend, install it with:

```bash
pip install pyrealsense2
```

### Alignment Controls

- Left mouse click: select an RGB point
- Right mouse click or `c`: clear selected point
- `a`: decrease `depth_x_offset` by 1 in Quanser mode
- `d`: increase `depth_x_offset` by 1 in Quanser mode
- `w`: decrease `depth_y_offset` by 1 in Quanser mode
- `s`: increase `depth_y_offset` by 1 in Quanser mode
- `r`: reset offset to the values in `qarm_core/config.py` in Quanser mode
- `p`: save the current offset to `qarm_core/config.py` in Quanser mode
- `q` or Esc: quit

In RealSense aligned mode, `a`, `d`, `w`, `s`, `r`, and `p` print a reminder
that manual offsets are not used.

The RGB window shows the selected image point. The depth window shows a
colorized depth image for display only; the measured distance still comes from
the real depth values in meters.

## Basic Running

Install the required Quanser tools and Python dependencies first.

Before using the virtual arm, open the QLabs QArm workspace. Before using real
hardware, power on and connect the physical QArm.

Run the current entry point:

```bash
python main.py
```

## Reusable Package Layout

- `qarm_core/config.py` - shared device IDs, ports, camera settings, depth
  limits, Quanser manual RGB-depth offset, poses, gripper values, and LED colors
- `qarm_core/camera/qarm_camera.py` - Quanser Camera3D physical/virtual camera
  frame reader
- `qarm_core/camera/realsense_aligned_camera.py` - physical RealSense aligned
  depth reader
- `qarm_core/camera/depth_alignment.py` - clicked-pixel depth test tool
- `qarm_core/motion/qarm_motion.py` - high-level motion helper for an open QArm
- `qarm_core/safety/qarm_safety.py` - beginner-readable command safety checks

## Safety Notes

- Keep a hand near the power switch while testing hardware.
- Start with small movements.
- Do not send large joint commands.
- `qarm_core.safety` raises errors for unsafe joint commands instead of
  silently clipping them.
- Gripper commands may be clamped to the safe `0.1` to `0.9` range.

## Planned Work

- Color tracking module
- Trajectory recording module
- Main program integration for calibrated color tracking
