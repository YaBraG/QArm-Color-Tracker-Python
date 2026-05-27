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
- `qarm_core/camera/qarm_camera.py` reads physical or virtual camera frames.
- `qarm_core/camera/depth_alignment.py` calibrates the RGB-to-depth pixel offset.

The root scripts are kept in place so existing classroom or lab workflows still
run. New reusable code should go under `qarm_core/`.

## Camera Notes

The physical RealSense camera is read in Python/OpenCV. Physical RealSense
frames do not appear inside the black camera panels in QLabs; those panels are
for QLabs virtual camera streams.

`qarm_core/camera/qarm_camera.py` wraps the existing `QArmRealSense` class and
keeps the last valid RGB/depth frames so displays do not blink during brief
camera read dropouts.

## RGB-Depth Alignment

The RealSense RGB stream and depth stream are close, but not perfectly aligned.
The project stores a manual pixel offset in `qarm_core/config.py`:

```python
DEPTH_X_OFFSET = -25
DEPTH_Y_OFFSET = -3
```

The depth alignment tool maps a clicked RGB pixel to a depth pixel like this:

```python
depth_x = rgb_x + DEPTH_X_OFFSET
depth_y = rgb_y + DEPTH_Y_OFFSET
```

The current saved/default calibration is `DEPTH_X_OFFSET=-25` and
`DEPTH_Y_OFFSET=-3` unless `qarm_core/config.py` is changed.

Run the physical RealSense alignment tool from the repository root:

```bash
python -m qarm_core.camera.depth_alignment --hardware 1
```

It can also be run directly from VS Code:

```bash
python qarm_core/camera/depth_alignment.py
```

### Alignment Controls

- Left mouse click: select an RGB point
- Right mouse click or `c`: clear selected point
- `a`: decrease `depth_x_offset` by 1
- `d`: increase `depth_x_offset` by 1
- `w`: decrease `depth_y_offset` by 1
- `s`: increase `depth_y_offset` by 1
- `r`: reset offset to the values in `qarm_core/config.py`
- `p`: save the current offset to `qarm_core/config.py`
- `q` or Esc: quit

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
  limits, RGB-depth offset, poses, gripper values, and LED colors
- `qarm_core/camera/qarm_camera.py` - physical/virtual camera frame reader
- `qarm_core/camera/depth_alignment.py` - RGB-depth calibration tool
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
