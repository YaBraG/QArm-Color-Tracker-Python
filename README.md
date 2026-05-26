# QArm Color Tracker Python

A Python project for controlling a physical Quanser QArm, mirroring it into
QLabs Virtual QArm, and reading QArm RealSense camera data for future color
tracking.

## Current Working Features

- Physical QArm connects through `qarm.py`
- QLabs virtual QArm connects through `virtual_qarm.py`
- `qarm_mimic.py` synchronizes real joint positions to the virtual arm
- QLabs can display the virtual QArm and virtual camera
- Physical RealSense can be read through `QArmRealSense` and the future
  `qarm_core.camera.QArmCamera` wrapper

## Important Camera Limitation

Physical RealSense images do not appear inside the built-in black camera panels
in QLabs. Those QLabs panels are tied to the virtual camera stream. Physical
camera frames should be displayed or processed in Python/OpenCV.

## New Library Structure

The current root files are still in place so existing working scripts keep
running. New reusable code is being added under `qarm_core/` for cleaner future
development.

- `qarm_core/config.py` - shared device IDs, ports, camera settings, poses,
  gripper values, and LED colors
- `qarm_core/camera/qarm_camera.py` - lazy-opening wrapper around
  `QArmRealSense` with last-valid-frame handling
- `qarm_core/motion/qarm_motion.py` - high-level motion controller for an
  already-open physical or virtual QArm object
- `qarm_core/safety/qarm_safety.py` - numeric safety checks for joint indexes,
  deltas, full joint vectors, and gripper commands

## Basic Running

Install the required Quanser tools and Python dependencies first.

Before using the virtual arm, open the QLabs QArm Workspace. Before using real
hardware, power and connect the physical QArm.

Run the current entry point:

```bash
python main.py
```

## Safety Notes

- Keep a hand near the power switch while testing hardware.
- Start with small movements.
- Do not send large joint commands.
- `qarm_core.safety` does not silently clip arm joints. Unsafe joint commands
  raise errors instead.
- Gripper commands may be clamped to the safe `0.1` to `0.9` range.

## Future Planned Modules

- `color_tracker.py`
- `trajectory_recorder.py`
- `main.py` integration update
