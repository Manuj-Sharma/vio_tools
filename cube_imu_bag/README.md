# Making a bag with the Cube IMU in place of the IMX

Written 2026-11-16. This takes a recorded flight bag (IMX on `/imu0`, camera, `/fc/*` topics) and the Cube dataflash log
of the same flight, and writes a new bag where `/imu0` carries the Cube's raw IMU. Everything else in the bag is copied
unchanged (cam1 is dropped), so the existing camera-IMU calibration and OpenVINS configs keep working.

The Cube data are rotated into the IMX frame and stamped on the IMX clock. That is why no new calibration is needed.

## Where things are

- **Scripts (git):** `~/vio_tools/cube_imu_bag/` (this folder). The log tools for steps 1-2 are in
  `~/vio_tools/cube_imx_comparison/scripts/`.
- **Data folder (not in git):** `~/rosbags/claude_tmp/cubeimu/`. It holds the `.npz` files, the built bags, the configs
  and the build logs. Each script here is linked into it under the same name, so the commands below work from there.

The scripts read and write their `.npz` files in the current directory. So for steps 4-6, `cd` into the data folder
first. For a different data folder, call the scripts by their full path from that folder.

## What you need

| Item | Where |
|---|---|
| Flight bag (sqlite3) | `~/rosbags/vio_<date>_<time>` |
| Cube dataflash log (`.BIN`) of the same flight | `~/Manuj/cube_logs_nov3-4_2026/log_0NN_*.BIN` |
| Log tools (list, download, match) | `~/vio_tools/cube_imx_comparison/scripts/` |
| Raw extraction | `~/vio_tools/cube_imu_bag/extract_cube_raw.py` |
| Alignment and bag building | `~/vio_tools/cube_imu_bag/`, run from the data folder `~/rosbags/claude_tmp/cubeimu/` |

Requirements on the log: it must contain `ACC` and `GYR` messages (raw-rate IMU logging enabled on the Cube). Logs 28-36
have them at about 2 kHz. Step 3 prints the sample counts; zero means raw logging was off and this method cannot be used.

Python: steps 1-3 need `pymavlink` (system python has it; `~/Manuj/.venv-cubeimx/bin/python` also works). Steps 4 and 6
need ROS (`source /opt/ros/humble/setup.bash`). Do not use scipy with ROS sourced: it is broken against numpy 2 there.
The scripts here use numpy only.

All commands in steps 4-6 are run **from the data folder**, because the scripts read and write their `.npz` files in
the current directory.

## Step 1. Get the `.BIN` off the Cube

Skip this if the log is already on disk. The Cube's `-if00` serial port must not be in use by anything else.

```
cd ~/vio_tools/cube_imx_comparison/scripts
python3 list_cube_logs.py                              # lists log numbers and sizes (read-only)
python3 download_logs.py ~/Manuj/cube_logs_nov3-4_2026 31      # downloads log 31; resumable; ~0.8 MB/s
```

The time shown in the listing is not a reliable flight time. Use step 2 to find which log belongs to which bag.

## Step 2. Find the log for the bag, and the clock offset

```
cd ~/vio_tools/cube_imx_comparison/scripts
python3 extract_cube_imu.py ~/Manuj/cube_logs_nov3-4_2026/log_031_20261104_111522IST.BIN ~/Manuj/cube_logs_nov3-4_2026/log031_imu_att.npz
python3 match_bag_to_log.py ~/rosbags/vio_20261104_110545 --start 30 --stop 400
```

- `extract_cube_imu.py` makes `logNNN_imu_att.npz` (the Cube's filtered IMU, attitude, GPS time reference). It uses about
  1 GB of memory per 300 MB of log. It is only needed for matching.
- `match_bag_to_log.py` correlates the yaw rate of the bag's `/imu0` with every `logNNN_imu_att.npz` in the logs folder.
  `--start` must be after arming. A true match has r >= 0.99. It prints **"Cube time minus bag time"**: write that number
  down, it is the `OFFSET` used in step 4.

Known values:

| Bag | Log | OFFSET (s) | Tag used here |
|---|---|---|---|
| `vio_20261103_092757_ctrl` | 28 | 19232.69 (Jetson clock was wrong that morning) | `ctrl` |
| `vio_20261103_165506` (spiral) | 29 | -0.02 | `165506` |
| `vio_20261104_110545` (200 m) | 31 | -0.14 | `110545` |

## Step 3. Extract the raw Cube IMU

```
cd ~/Manuj/cube_logs_nov3-4_2026
python3 ~/vio_tools/cube_imu_bag/extract_cube_raw.py log_031_20261104_111522IST.BIN raw/log031_raw.npz        # IMU instance 0 (default)
```

Usage: `extract_cube_raw.py LOG.BIN OUT.npz [instance]`. It prints the output name, the number of accel and gyro
samples and the GPS time reference. Instance 0 on the Cube Orange is the ICM20602 and is the one used for all bags so far.
Instance 2 (ICM20649) is much noisier above 100 Hz.

## Step 4. Align the Cube to the IMX (clock and rotation)

```
cd ~/rosbags/claude_tmp/cubeimu
source /opt/ros/humble/setup.bash
python3 align.py <bag dir> <raw npz> <OFFSET> <tag>
# example
python3 align.py ~/rosbags/vio_20261104_110545 ~/Manuj/cube_logs_nov3-4_2026/raw/log031_raw.npz -0.14 110545
```

`<tag>` is any short name; it names the output files and is passed again in steps 5 and 6.

What it does:
1. Reads `/imu0` (IMX) from the bag and saves it as `imx_<tag>.npz`.
2. Cuts the Cube data to the bag's time span and regularises its timestamps. The Cube's `SampleUS` stamps arrive in
   clumps and are not a true clock, so a straight line is fitted and slow steps from dropped samples are followed.
   Saved as `cube_<tag>.npz`.
3. Fits the Cube-to-IMX rotation from the gyros and the time lag by cross-correlation, then fits how the lag drifts over
   the bag (a straight line). Saved as `align_<tag>.npz`.

Check in the output:
- `cube covers bag s A..B`: the part of the bag that has Cube data. If the log starts at arming, A is the arming time.
- `gyro after align: corr per axis`: should be close to 1 on the axes that move.
- `lag model: ... s + ... s/s * t ; residual ms`: residuals of a few ms are normal; tens of ms mean a bad match.

## Step 5. Rotation with gravity for roll and pitch

The gyro fit alone gets roll/pitch wrong by up to 1.3 degrees, which puts 0.2 m/s2 of gravity into the horizontal axes
and makes the VIO height creep. This step fixes roll/pitch with gravity and keeps the gyro's yaw.

```
python3 gravity_rotation.py <tag> <t_start_s> <t_end_s> [out.npy]
# example: 200 m bag, vehicle still between arming (22.0 s) and spool-up
python3 gravity_rotation.py 110545 22.5 23.9
```

The window is in bag seconds and must be a time when the vehicle stands still and both IMUs have data. A longer still
period is better; the script warns if it finds fewer than 3 quiet seconds. Default output is
`R_cube_to_imx_gravity_gyro_<tag>.npy`.

The mounting does not change between flights, so one good rotation can be reused. All bags so far use
`R_cube_to_imx_gravity_gyro.npy` (from the spiral bag, window 22.8-27.0 s). The three bags agree within 0.5 degrees:

| Bag | Rotation vector, degrees (x, y, z) |
|---|---|
| 092757 (`ctrl`, 1-13 s) | -0.87, 0.77, -2.08 |
| 110545 (22.5-23.9 s) | -0.95, 0.88, -2.60 |
| 165506 (22.8-27.0 s) | -0.85, 0.77, -2.60 |

## Step 6. Build the bag

```
source /opt/ros/humble/setup.bash
CUBE_R=$PWD/R_cube_to_imx_gravity_gyro.npy python3 make_cube_bag.py <variant> <source bag> <tag> <output bag> [t_end_s]
```

| Argument | Meaning |
|---|---|
| `CUBE_R=<file>` | Rotation from step 5. Without it the gyro-only rotation from step 4 is used (not recommended). |
| `<variant>` | Filter applied to the ~2 kHz Cube data before it is sampled at 333.3 Hz (3 ms grid). See below. |
| `<tag>` | Same tag as in step 4. |
| `<output bag>` | New folder; must not exist. |
| `[t_end_s]` | Optional: stop after this many seconds of the source bag. Use the end of the Cube coverage from step 4. |

Variants:

| Variant | Filter | Use |
|---|---|---|
| `lp40` | Zero-phase 4th-order Butterworth at 40 Hz. No delay. Needs the whole recording, so offline only. | Reference. |
| `clp40` | Causal 2-pole Butterworth IIR at 40 Hz, run sample by sample. Delay 5.63 ms. | What a real-time filter would give. |
| `raw` | No filter. Aliased on purpose. | Shows what filtering buys. |

The number can be changed (`lp20`, `clp30`, ...). The cutoff must stay below the vehicle's lowest prop frequency.

Examples (the commands that made the current bags):

```
CUBE_R=$PWD/R_cube_to_imx_gravity_gyro.npy python3 make_cube_bag.py lp40  ~/rosbags/vio_20261104_110545 110545 $PWD/bag_110545_cube_lp40_full_rotfix 575
CUBE_R=$PWD/R_cube_to_imx_gravity_gyro.npy python3 make_cube_bag.py clp40 ~/rosbags/vio_20261104_110545 110545 $PWD/bag_110545_cube_clp40_full_rotfix 575
```

Building the 200 m bag takes a few minutes and writes 13 GB. Do not build while a VIO run is playing.

What the script does:
- Filters the Cube data, moves it onto a uniform 3 ms grid, rotates it into the IMX frame and stamps it on the IMX header
  clock using the lag model from step 4.
- Where the Cube log has no data yet (before arming), it fills in the IMX data, shifted by a constant so its mean matches
  the Cube's over the first 1.5 s of overlap. The log line `pre-arm fill with IMX for first N s` reports this.
- Copies every other topic unchanged, except `/cam1/*`, which is dropped.

Check in the output: `out samples`, the lag range, and the final `done: other msgs N imu written M`. Then
`ros2 bag info <output bag>` should show `/imu0` at about 333 Hz.

## Step 7. Run OpenVINS on it

```
export OV_WS=$HOME/openvins_ws_axis
~/vio_tools/run_ensemble.sh <tag prefix> <bag> <config dir> <cam topic> <stop after s> <start offset> [more offsets]
# example: 200 m bag, full flight, start at 18.00 s
~/vio_tools/run_ensemble.sh ensCube200FULL_rotfix_acc0.005_gyr0.0004 $PWD/bag_110545_cube_lp40_full_rotfix $PWD/cfg_cube_acc0.005_gyr0.0004 /cam0/image_30hz 585 18.00
```

Results go to `~/rosbags/claude_tmp/runs_<date>/<tag prefix>_o<offset>/` (`results.txt`, `gt.log`, `odom_out/`).

Configs (in git under `~/vio_tools/cube_imu_bag/configs/`; the same folders are in the data folder, which is what the commands above use):

| Config | Use with |
|---|---|
| `cfg_cube_acc0.005_gyr0.0004` | `lp40` bags. Accel noise 0.005, gyro noise 0.0004, pixel noise 2, extrinsics fixed. |
| `cfg_cube_acc0.005_gyr0.0004_td5.63` | `clp40` bags. Same, with `timeshift_cam_imu` moved by +5.63 ms (-23.60 to -17.97 ms) for the filter delay. |

For any causal filter the delay must go into `timeshift_cam_imu`: add `sqrt(2) / (2 * pi * cutoff)` seconds for a 2-pole
Butterworth (5.63 ms at 40 Hz). On the 200 m bag, ignoring it made the height sink about 17 m during cruise and land
13 m low; with it the result matched the zero-phase bag.

Cam topic is `/cam0/image_30hz` on 110545 and `/cam0/image_raw` on 092757 and 165506.

To compare a finished run with GPS: `python3 umeyama_report.py <run folder> [more run folders]` (ROS sourced).

## Things that go wrong

- **Reading a run's `odom_out/*.db3` while it is still recording** kills the recorder ("database is locked"). Analyse
  only finished runs.
- **Cube clock drift.** `SampleUS` is not a true clock; on 110545 the lag to the bag drifts 0.3 ms per second. `align.py`
  models this as a straight line. Check its residuals on long bags.
- **Log starts at arming.** Then the first seconds of the new bag are IMX data (see the pre-arm fill line). Start VIO
  close to arming, for example 18 s on 110545 where arming is at 22 s.
- **Lever arm.** The position offset between the Cube and the IMX is ignored.
- **Yaw of the rotation** comes from the gyro fit only and differs by 0.5 degrees between bags. A multi-position static
  recording (rig held still in 6 or more orientations) would fix all three angles from gravity.
- **Disk.** Each 200 m bag is 13 GB.

## Files

Scripts are in `~/vio_tools/cube_imu_bag/`; the `.npz`, logs, queue scripts and bags are in the data folder.

| File | What it is |
|---|---|
| `align.py` | Step 4. |
| `gravity_rotation.py` | Step 5. |
| `make_cube_bag.py` | Step 6. `make_cube_bag.py.bak_20261116` is the version before the causal variant. |
| `imx_<tag>.npz`, `cube_<tag>.npz`, `align_<tag>.npz` | Outputs of step 4. |
| `extract_cube_raw.py` | Step 3. |
| `R_cube_to_imx_gravity_gyro.npy` | Rotation used for all current bags (in git). `align_gravity_combined.npz` holds the three per-bag versions. |
| `mk_*.log` | Build logs. |
| `q_*.sh`, `pipeline*.sh` | Queue scripts with the exact build and run commands used. |
| `umeyama_report.py` | VIO track against GPS. |
| `configs/` | The two OpenVINS configs used for the Cube-IMU bags. |
| `../gt_oneshot_pub.py` | Draws the GPS track in RViz during a run (started by `run_bag.sh`). |
| `strapdown_cmp.py`, `plot_cube_noise.py`, `plot_cube_vs_imx_spectrum.py`, `feat_tap.py` | Analysis tools. |
