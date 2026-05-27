# Changelog

## 2026-05-27

- Updated requirements dependency file under `requirements/`.
- Added `pyrealsense2` dependency for the RealSense aligned camera backend.
- Documented dependency installation in README.
- Preserved the existing requirements folder workflow.

- Added RealSense aligned-depth camera backend.
- Added backend option to the depth alignment tool.
- Documented that manual RGB-depth offset changes with distance due to parallax.
- Kept manual offset backend for Quanser Camera3D debugging.
- Updated README documentation.

- Added depth alignment calibration tool cleanup.
- Added configurable RGB-to-depth offset values.
- Added save-to-config behavior with the `p` key.
- Added/updated README documentation for the current project state.
- Documented the current calibration offset: `DEPTH_X_OFFSET=-25`,
  `DEPTH_Y_OFFSET=-3`.
