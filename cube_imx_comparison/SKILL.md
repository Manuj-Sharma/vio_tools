---
name: cube-imx-comparison
description: Compare the IMX-5 IMU stream (/imu0 in a rosbag) with the Cube flight controller's own logged IMU for the same flight, on the Jetson VIO rig. Covers getting the Cube dataflash logs over MAVLink, matching each bag to its log, aligning clocks, per-axis gyro/accel agreement, noise spectrum, outlier test, timing lag, and the OpenVINS noise-density consequence. Use when asked "does the IMX data match the FC", "is the IMX fine", "why does OpenVINS diverge", or to analyse a new flight bag against Cube logs.
---

# Cube vs IMX comparison

Everything here was worked out on 2026-11-09 for the rig in `~/vio_ar0234_icm20948` (IMX-5 over SPI + CubeOrange over USB).
Scripts are in `scripts/` next to this file; all take arguments (no hard-coded bag names). Python needs `pymavlink`, `rosbags`, `scipy`,
`matplotlib` (no ROS sourcing needed). `node` is NOT installed (the dataviz palette validator could not be run).

## Safety rules (learned the hard way)
- **Never touch the I2C bus** (i2cdetect/i2cget/open /dev/i2c-*). Scanning `i2c-5` (31b0000.i2c) hard-resets the Jetson (2/2 times). Buses 7, 1, 0 are safe in `-r` mode.
- **One owner per port.** The Cube `-if00` serial port must be exclusively used while listing/downloading logs; `/dev/spidev0.0` (IMX) has one owner (imx5_spi_node OR a tool).
- **Memory:** extracting one 300 MB log needs ~1 GB. At most two extractions at once on the 8 GB Jetson (five at once got to 3 GB free).
- **Do not run CPU-heavy analysis while an OpenVINS replay you intend to score is running** (it is real-time).
- Do not edit the frozen OpenVINS config (`~/rosbags/cfg_frozen_...`); copy it.

## Workflow
0. **Which flight?** A rosbag with `/imu0` (+ camera + `/fc/*`). Note: the IMX header stamps are IMX-clock seconds (not epoch); use bag RECEIVE time
   (it tracks the header stamp to 1-4 ms, drift < 1 ms/min).
1. **List logs on the Cube** (read-only): `python3 scripts/list_cube_logs.py`. The printed time is NOT a reliable start/end time. Log files are ~0.38 MB/s of armed time.
2. **Download** (resumable, ~0.8 MB/s): `python3 scripts/download_logs.py OUTDIR 28 29 30 ...` (sends LOG_REQUEST_END first; `.part` files resume).
   Check each file against the listed size and header bytes `a3 95`.
3. **Extract** per log: `python3 scripts/extract_cube_imu.py log_031_*.BIN OUTDIR/log031_imu_att.npz` (IMU[0..2], ATT, GPS time reference, IMU filter params).
4. **Match bag to log** (do this, do not trust names/times): `python3 scripts/match_bag_to_log.py BAG --logs OUTDIR --start 30`.
   Signed gyro-Z correlation, POST-ARM part of the bag only. Real match: r >= 0.99 at ~0.0 s offset. No match => that flight is not on this Cube / not downloaded.
5. **Compare + figures:** `python3 scripts/compare_cube_vs_imx.py --bag BAG --npz OUTDIR/log031_imu_att.npz --tag 110545 --out OUTDIR/figures --start 25 --stop 280`
   -> prints per-axis slope/r, PSDs, noise density; writes fig1-fig6 `*_TAG.png`.
6. **Noise character:** `python3 scripts/imx_noise_check.py --bag BAG --start 25 --stop 280` (Gaussian? glitches? density vs config).
7. **Timing:** `python3 scripts/lag_windows.py --bag BAG --npz ... --start 30 --stop 300` (constant shift or drifting?).
8. Static checks of the sensor itself (bench, not flight) live in `~/vio_tools` (uncommitted): `imx5_bit_test status|run [--go]` (built-in self-test, ~5 s, RAM command not a flash write),
   `six_pos_capture.sh` + `six_pos_analyze.py` (orientation test; tilted poses need the magnitude fit |diag(s)(a-b)|=g, not face averaging), `imx5_flash_tool backup` (read-only flash backup).

## Pitfalls (each cost time once)
- **Do not compare |accel| magnitudes under vibration**: the norm rectifies noise (near-zero correlation, wrong conclusion). Compare per-axis vectors after fitting the rotation.
- **The Cube's logged accel is low-passed** (`INS_ACCEL_FILTER` = 10 Hz, gyro 25 Hz, harmonic notch on, no raw batch log). Only <= 10 Hz is like-for-like; use the same low-pass on both.
- **Cube IMU records start at arming**; the bag usually starts ~20 s earlier. Matching with the whole bag forced inside the log gives wrong offsets.
- **Gyro-magnitude matching can be off by a half pattern** (periodic +/- yaw pulses): use the signed yaw rate.
- FC "acceleration" derived by differentiating the 10 Hz logged position/velocity is a weak reference: use the Cube's own IMU from the .BIN.
- The Cube clock (GPS time) and the Jetson clock agree to <= 0.1 s; the remaining IMX-vs-Cube lag is 26-116 ms (IMX later) and can drift within a flight; it is NOT the Jetson receive time.
- 2-10 Hz band correlations are timing-limited (40 ms = 144 deg at 10 Hz). Use the 1 Hz view for shape agreement, the PSD for noise.
- `/cam0/image_30hz` bags need a remap to `/cam0/image_raw` for `run_openvins_mono_bag.sh` (patched copy: add `BAG_CAM_TOPIC` and `--remap "$BAG_CAM_TOPIC:=$CAM_TOPIC"`; see the repo script lines for CAM_TOPIC / --topics).

## Reference results (2026-11-09, three matched flights)
Matches: bag 110545 = Cube log 31, 103617 = log 30, 165506 = log 29 (all r >= 0.994, offset <= 0.1 s). Bag 163126 (bad) and the flight_light bags: no match in logs 28-33 (34-36 downloaded, not yet tested).
| bag (IMX rate) | IMX accel noise density at 1 Hz X/Y/Z (m/s^2/sqrtHz) | accel r at 1 Hz X/Y/Z | gyro r at 1 Hz | lag IMX later (mean, std) |
|---|---|---|---|---|
| 110545 (319 Hz) | 0.24 / 0.21 / 0.50 | 0.94 / 0.89 / 0.50 | 1.00 / 0.99 / 1.00 | 116 ms, 39 (drifts) |
| 165506 (280 Hz) | 0.29 / 0.24 / 0.60 | 0.94 / 0.96 / 0.84 | 1.00 | 26 ms, 10 |
| 103617 (207 Hz, 26% missing) | 0.33 / 0.34 / 0.65 | 0.89 / 0.88 / 0.35 | 0.99 / 0.98 / 1.00 | 37 ms, 12 |
Conclusions: gyro is faithful (slope 1.00-1.02, frame rotation Cube->IMX ~1-2 deg); static calibration is good (bias <= 4 mg, scale <= 0.05%, BIT passes);
slow acceleration is tracked; the flight accelerometer noise floor is flat (0.3-50 Hz), Gaussian (excess kurtosis ~0, 0.25% >6 sigma, removing them changes nothing),
100-1000x the Cube's at 1-3 Hz, ~4 orders above bench, Z about 2x X/Y, and rises monotonically as the IMX sample rate falls (319 < 280 < 207 Hz)
=> consistent with unfiltered vibration aliased at ~320 Hz sampling (NOT proven: three uncontrolled flights).
Consequence: OpenVINS `accelerometer_noise_density` should be ~0.2-0.6; the frozen config uses 0.2 (fits X/Y, ~2.5-3x low for Z). Old configs used 1.19e-2.
Deciding experiment still missing: a FLIGHT (or ground run with motors) recorded with the 1 kHz stream + `/imu0_filtered` (branch `imu-1khz-filter`), then compare
the PSD of `/imu0_filtered` with these numbers (the existing filter test bag is a static bench recording and cannot test it).

## Output layout used so far
`~/Manuj/cube_logs_nov3-4_2026/` : `log_0NN_<listedtime>IST.BIN` (logs 28-36), `log0NN_imu_att.npz`, `download.log`, `figures/fig{1..6}_*_{110545,103617,165506}.png`.
Memory notes (Claude auto-memory) `cube-onboard-logs-2026-11-09.md`, `imx5-bit-test-tool-2026-11-09.md`, `feedback-never-probe-i2c-jetson-resets-2026-11-09.md` hold the detail.

## Install as a Claude skill (optional)
`mkdir -p ~/.claude/skills && ln -s ~/Manuj/cube-imx-comparison ~/.claude/skills/cube-imx-comparison`
