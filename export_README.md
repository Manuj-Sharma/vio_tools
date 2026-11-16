# VIO config package (2026-11-06) — cam0 mono OpenVINS, accelerometer-noise experiments

Everything needed to run the configs below on another machine. Built from a long debugging session on the
straight-climb bag `vio_20261103_092757` (70 m vertical climb, camera looking down, IMX-5 IMU).

**Best config in the ensemble tests so far:** @@BEST@@

> Read the caveats at the bottom before trusting any number. Single runs of this system are not repeatable
> (a 20-30 ms shift of the start time flips the result), so everything here is judged as a 5-run ensemble.

## What is in this folder

| path | what |
|---|---|
| `configs/A_baseline_scalar_accel0.2/` | baseline: scalar accelerometer noise 0.2 (runs on a stock build of the code base below) |
| `configs/B_peraxis_accel_x0.1_y0.1_z0.5/` | per-axis accelerometer noise (x 0.1, y 0.1, z 0.5). Needs the patched build |
| `configs/C_vibration_adaptive_base0.02_gain1.0/` | vibration-adaptive accelerometer noise, base 0.02, gain 1.0. Needs the patched build |
| `configs/D_vibration_adaptive_base0.02_gain2.0/` | same, gain 2.0. Needs the patched build |
| `calibration/` | kalibr cam0-IMU result (2026-11-06, reprojection 1.24 px, time offset -23.6 ms) the configs are built from |
| `code/per-axis-accel-noise.patch` | the code change (git diff against commit `1e24f52`), see `code/BASE.txt` |
| `code/open_vins_patched_sources.tgz` | the already patched `ov_core`, `ov_init`, `ov_msckf` source directories (build with colcon) |
| `scripts/` | helper scripts used on the original machine (paths are hard-coded to `/home/unmannd/...`; edit before use) |
| `results/` | ensemble summary tables and the quiet-window IMU init values |

Each config folder has three yaml files: `estimator_config.yaml`, `kalibr_imu_chain.yaml`, `kalibr_imucam_chain.yaml`.
Unknown keys are ignored by an unpatched build, so configs B, C, D also run on a stock build, but then they behave
like the scalar config (the scalar `accelerometer_noise_density` is kept in the file for that reason).

## Settings common to all configs (cam0 mono)

- Calibration: kalibr `cam0_imu_nt` (fx 856.79, fy 846.29, cx 644.00, cy 317.15, radtan, T_cam_imu from the final solve), camera-IMU time offset -23.6 ms
- All online calibration off (extrinsics, intrinsics, time offset); camera pixel noise 2; `track_frequency` 40
- ZUPT on at the start only (`try_zupt: true`, `zupt_only_at_beginning: true`); `gravity_mag` 9.7801 (9.80 made it worse)
- IMU: IMX-5 on `/imu0`, gyro noise 1.4e-3, accel noise as listed per config, update rate 330 (real rate 300-330 Hz)
- Launch as mono: `max_cameras:=1 use_stereo:=false` (the yaml still says 2/true, the launch arguments override it)

## New config keys (patched build only), in `kalibr_imu_chain.yaml`

| key | meaning |
|---|---|
| `accelerometer_noise_density_x/_y/_z` | per-axis accelerometer noise (all three must be > 0). The scalar `accelerometer_noise_density` is then ignored by the propagator |
| `accelerometer_vib_gain` | vibration-adaptive noise gain k (0 or absent = off) |
| `accelerometer_vib_window` | window in seconds for the vibration estimate (default 0.5) |
| `init_wait_for_jerk` (in `estimator_config.yaml`) | -1 auto (default), 0 never wait for a jerk, 1 always wait. Not used by these configs |

Vibration-adaptive rule: for each axis, `sigma^2 = sigma_base^2 + (k * std * sqrt(dt))^2`, with `std` the sample standard
deviation of that accelerometer axis over the last `vib_window` seconds and `dt` the mean sample spacing in that window.

## How to build

1. Install ROS 2 Humble, Eigen, OpenCV, Ceres, Boost (the usual OpenVINS dependencies).
2. `mkdir -p ws/src/open_vins && cd ws/src/open_vins && tar -xzf <this folder>/code/open_vins_patched_sources.tgz`
   (this gives `ov_core`, `ov_init`, `ov_msckf`). Alternatively apply `code/per-axis-accel-noise.patch` to commit `1e24f52`
   of your OpenVINS repository (branch `bagC-feature-diagnostics`).
3. `cd ws && source /opt/ros/humble/setup.bash && colcon build --packages-up-to ov_msckf --cmake-args -DCMAKE_BUILD_TYPE=Release`

## How to run one config on the climb bag

```
source /opt/ros/humble/setup.bash; source ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch ov_msckf subscribe.launch.py config_path:=<config dir>/estimator_config.yaml \
     use_stereo:=false max_cameras:=1 verbosity:=INFO rviz_enable:=false &
sleep 15
ros2 bag play <bag dir> --start-offset 126.0 --read-ahead-queue-size 2000 --topics /imu0 /cam0/image_raw /fc/gps1 /fc/ekf_yaw
```

- Straight-climb bag `vio_20261103_092757`: arming at 135.7 s; use start offsets 126.00-126.17 s (the ensembles shift the start by 0.03 s steps). Stop about 60 s after the filter's first odometry (about 88 s of wall time after launch).
- The 200 m bag `vio_20261104_110545` has only a cam0 stream named `/cam0/image_30hz` (add `--remap /cam0/image_30hz:=/cam0/image_raw`, start offset 0). It is handled and disturbed before takeoff, so it is a poor bag for init tests.
- Keep the machine quiet while a run is going; load changes the timing and the result.
- To score a run against the flight controller height: `scripts/score_run.py <run folder>` (needs the `run_bag.sh` run folder layout, see the scripts).

@@OVERVIEW@@

## Caveats (important)

- **One bag, one flight.** Everything was tuned on `vio_20261103_092757`. Nothing here is validated on another flight yet.
- **Not repeatable run to run.** Identical runs reproduce exactly on a quiet machine, but a 20-30 ms shift of the start time changes the climb scale from about 1.3x to 2x. Judge a config by the spread over several start offsets (the ensemble), not by one run.
- **Climb ratio is height only.** It is VIO height divided by flight-controller height at the moment the FC height reaches 10/20/40/60 m. It says nothing about horizontal error, the descent, or the later flight. The descent was still too slow in earlier tests.
- **Cause found (a lead, not proven):** with exact initial biases the IMU alone already overshoots the early climb by about 2x. The real takeoff acceleration is about 0.1 m/s^2 while the vibration is 3-5 m/s^2 standard deviation, so a tiny accelerometer error accumulates. This is why trusting the accelerometer less (higher or adaptive noise) helps. Check the IMX-5's internal filter and sample rate in the driver.
- The patched code (per-axis noise, vibration-adaptive noise, `init_wait_for_jerk`) is uncommitted work on a local branch. It builds and runs, but it has had only these tests.
