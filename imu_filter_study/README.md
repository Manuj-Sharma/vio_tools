# IMU 1 kHz filter study (2026-11-10)
Offline study of causal Butterworth low-pass settings on the raw 1 kHz IMX-5 `/imu0` (for the 300 Hz `/imu0_filtered` output).
All scripts: `python3 SCRIPT BAG_DIR [OUT.png]` with ROS sourced (rclpy), NOT `python3 -I`. Bags are in ~/rosbags.

Bags: `imu1k_lift_20261110_104703` (8 lifts + handling), `imu1k_knock_20261110_110619` (knocks), `filt_o4_fc100_out300` (static, 11-09).
Main tools (one level up, in ~/vio_tools): `record_imu_1khz.sh`, `imu_filter_sweep.py`, `imu_tone_check.py`.

| script | what |
|---|---|
| lift_band_power.py | power in 0-20/20-50/50-100/100-150/>150 Hz bands, lift windows vs still (lift bag) |
| lift_vs_still_spectrum.py BAG OUT.png | lift-window vs still PSD per channel, per-lift f90/f99 (lift bag; hard-coded t>3.55 s start) |
| clean_lifts_filter_error.py | filter error/peak ratio on the 5 clean lifts only (times hard-coded: 9.4 21.7 33.3 56.7 67.2 s) |
| knock_events_and_filter_alias.py | knock detection, saturation check, power kept/aliasing per filter (knock bag; uses t 0.5-29.75 s) |
| knock_spectrum_spectrogram.py BAG OUT.png | knock vs still PSD, ratio, spectrogram (az, ax) |
| knock_spectrum_with_filters.py BAG OUT.png | knock PSD raw vs filtered + filter magnitudes, humps marked |
| knock_ringdown_per_knock.py | per-knock 0.25 s ring-down lines, recurring frequencies |

Findings: see session notes. Fan line ~48-50 Hz (+ harmonics ~192/198, 384/396 Hz) follows fan rpm (unproven: rpm not logged during recordings).
Window times in several scripts are hard-coded for these specific bags.
