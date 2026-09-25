# vio_tools

Analysis and diagnostic tooling for the GPS-denied VIO project. Not a ROS package itself —
standalone scripts, mostly run against live topics or recorded bags/CSVs. This is the
designated home for any new standalone helper script for the VIO project; don't leave new
tools loose in `~/` or in `~/vio_ar0234_icm20948`.

## What's here

- `check_camera_sync.py` — cross-camera timestamp offset measurement for the dual-AR0234 rig
  (used to diagnose the ~12ms sync jitter root-caused in the sensor stack's own timestamping
  architecture — see that repo's CLAUDE.md).
- `watch_compass.py` — standalone live compass/EKF-yaw monitor, connects directly to
  `/dev/ttyACM0`, prints a heading label. Useful as a known-heading reference before any test
  that depends on a specific rig orientation.
- `check_compass_sanity.py` — live interference monitor: raw field-magnitude deviation from a
  startup baseline, plus compass-vs-gyro yaw-rate disagreement. Auto-detects which compass
  instance is actually feeding the EKF (`COMPASS_USE`/`USE2`/`USE3`) rather than hardcoding —
  slot assignment has changed across sessions before, don't trust it as static.
- Trajectory/GPS comparison plotting scripts (`plot_gps_trajectory.py`,
  `plot_vio_trajectory.py`, `plot_vio_vs_gps_ned.py`, `animate_vio_vs_gps_ned.py`, etc.) — read
  bag sqlite3 data directly, don't need `metadata.yaml`. Safe to run against a live stack or
  replay (read-only), show on screen by default, never save unless `-o` is given.
- `vs_gps.py` / `plot_vs_gps.py` — a related but different comparison: Umeyama-fit alignment
  against a separate ArduPilot xlsx ground-truth log, multi-run overlay, used for SITL/offline
  tuning. Don't confuse with the `_ned` scripts above, which plot one real-hardware run's
  already-time-synced output with no fitting.
- OpenVINS tuning tools: `allan_variance.py`, `estimate_imu_bias.py`, baro-fusion sweep
  scripts, bag-quality checks.

## Known pitfalls (see README.md for the full list)

- `run_subscribe_msckf.cpp`'s hardcoded `use_multi_threading_subs` introduces
  non-determinism — don't expect bit-identical runs.
- IMU sample-rate/aliasing gotchas exist; check the README before adding new spectral analysis.
- MSCKF% is not an accuracy metric — don't use it as a proxy for trajectory correctness.

## OpenVINS `init_max_features` pitfall (generic, not project-specific)

Real OpenVINS bug, not sim-specific: `init_max_features / num_cameras` must be comfortably
`>= grid_x * grid_y`, or `Grider_GRID` silently collapses its feature-extraction grid,
capping real features at ~9 regardless of scene content, with no error. Recompute this
whenever `init_max_features`, `grid_x`, `grid_y`, or the camera-calibration count changes in
any config, sim or real. This tripped up the Gazebo sim thread badly before being root-caused;
the real hardware stereo configs currently sit exactly on this boundary with zero margin.
