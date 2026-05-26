# QArm Color Tracker Python

A Python project for experimenting with Quanser QArm control and camera-based
color tracking.

## Overview

This repository contains a small QArm control wrapper, camera utilities, and a
simple `main.py` entry point for testing arm movement and sensor reads. It is
intended for use with Quanser hardware or a compatible Quanser virtual QArm
setup.

## Requirements

- Python 3
- NumPy
- Quanser Python libraries
- Quanser QArm hardware or virtual QArm environment

## Project Structure

- `main.py` - basic entry point for running QArm experiments
- `qarm.py` - QArm hardware interface and RealSense camera wrapper
- `utilities/` - helper modules for vision, timing, streaming, input, and math

## Running

Install the required Quanser tools and Python dependencies, connect or start the
QArm environment, then run:

```bash
python main.py
```

## Notes

This project is an early color-tracking and QArm-control workspace. Update
`main.py` as needed for your specific tracking routine, hardware configuration,
or virtual QArm setup.
