# QArm Color Tracker Python

Python support code for a Quanser QArm color tracking project. The repository
currently focuses on QArm connection helpers, virtual/physical arm support, and
camera distance helpers. Color tracking is planned, but it is not implemented
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
- `qarm_core/camera/depth_alignment.py` samples distance from an already-aligned
  depth frame.

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

## RGB-Depth Distance

The recommended physical-camera path for RGB-depth distance tracking is the
RealSense aligned-depth backend.

`qarm_core/camera/realsense_aligned_camera.py` opens the physical RealSense
camera and returns:

- `color_bgr`: an OpenCV-ready color frame
- `aligned_depth_m`: depth in meters aligned to the color frame

`qarm_core/camera/depth_alignment.py` is now a reusable library for sampling
distance from an already-aligned depth frame. It does not open cameras, create
OpenCV windows, draw crosshairs, show overlays, or handle keyboard controls.
Those UI pieces should be handled later by `main.py` or another
application-level program.

The old manual Quanser pixel-offset workflow was removed from
`depth_alignment.py` because fixed offsets change with distance due to parallax.
`qarm_core/camera/qarm_camera.py` still exists for Quanser Camera3D and QLabs
compatibility.

Example aligned-depth use:

```python
from qarm_core.camera.realsense_aligned_camera import RealSenseAlignedCamera
from qarm_core.camera.depth_alignment import get_depth_at_rgb_pixel

camera = RealSenseAlignedCamera()
camera.open()

color_bgr, aligned_depth_m = camera.read()
result = get_depth_at_rgb_pixel(aligned_depth_m, rgb_x=371, rgb_y=330)

camera.close()
```

## Basic Running

Install the required Quanser tools and Python dependencies first.

Python pip dependencies are listed in the existing requirements folder:

```bash
python -m pip install -r requirements/requirements.txt
```

This installs packages such as `numpy`, `opencv-python`, and `pyrealsense2`.
The RealSense backend uses the Intel RealSense SDK Python bindings and is the
recommended physical-camera mode for RGB-depth distance tracking.

Quanser and QLabs support may still require the separate Quanser software
installation. Those Quanser packages are not treated as ordinary pip
dependencies in this repo.

Before using the virtual arm, open the QLabs QArm workspace. Before using real
hardware, power on and connect the physical QArm.

Run the current entry point:

```bash
python main.py
```

## Reusable Package Layout

- `qarm_core/config.py` - shared device IDs, ports, camera settings, depth
  limits, depth sampling window size, poses, gripper values, and LED colors
- `qarm_core/camera/qarm_camera.py` - Quanser Camera3D physical/virtual camera
  frame reader
- `qarm_core/camera/realsense_aligned_camera.py` - physical RealSense aligned
  depth reader
- `qarm_core/camera/depth_alignment.py` - aligned-depth distance sampling helpers
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
