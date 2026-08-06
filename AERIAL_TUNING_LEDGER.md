> **SUPERSEDED for config choice (2026-08-04).**
> GPS ground truth now exists (`~/rosbags/seg01_fc.xlsx`) and the recommended
> config has changed to `ov_9281_flight_D_best`. See
> `~/sensor_ws/src/sensor_bringup/config/README.md`.
> Rows below were scored on MSCKF percentage, which turned out to be blind to
> trajectory scale error -- 69.4/66.1/71.1% gave size errors of +35/+2/+36%.
> The measurements here remain valid; the ranking derived from them does not.

# OpenVINS aerial tuning ledger — OV9281 + IMX-5 nadir drone

Every row is a **measured** replay. Kept outside /tmp so it survives reboots
(/tmp was cleared twice during this work and took all the logs with it).

**Bag:** `~/rosbags/rosbag2_2026_07_31-11_23_05`
209.06 s, 12608 images @ 60.3 Hz, 49753 IMU @ 238 Hz, nadir agricultural,
altitude ≈ 40 m (scaled off a ~4.5 m vehicle spanning ~115 px at fx 1021.5).
3763 FAST corners/frame at `fast_threshold 20` — texture is NOT a limitation.

**Calibration** (fixed throughout, 2026-08-01):
intrinsics `[1021.51, 1007.12, 620.22, 428.50]` equidistant,
`t_cam_imu = [-9.74, -7.66, 42.83] mm` (physical z measured 38 mm),
`timeshift -4.439 ms` (valid only at `exposure_us=4000`),
IMU noise `1.67e-4 / 1.19e-3` (≈3.5× the InertialSense datasheet floor).

---

## CURRENT RECOMMENDED CONFIG

**`ov_9281_aerial_staticinit`, run at normal `--rate 1.0`.**

```yaml
fi_max_dist: 500.0          # was 60     aerial triangulation gates
fi_max_baseline: 500.0      # was 40
fi_max_cond_number: 200000  # was 10000
init_dyn_use: false         # was true   removes the nondeterministic init
gravity_mag: 9.7801         # was 9.81   Bangalore local
```

Measured twice at full rate: **45.0% / 46.9% MSCKF usage, 666 / 673 m drift.**

---

## Results

| # | date | config | rate | init frame | MSCKF nonzero | feats | drift | `ba` final |
|---|------|--------|------|-----------|---------------|-------|-------|-----------|
| A | 08-01 | `bag_recal` (indoor gates 60/40/10000) | 1.0 | — | 67 (1.7%) | 891 | 27072 m | −1.67, 1.04, 0.06 |
| B | 08-01 | `bag_recal_aerial` (gates 500/500/200000) | 1.0 | — | 1658 (41.9%) | 16031 | 746 m | 0.11, 0.01, 0.03 |
| C | 08-01 | B + `calib_cam_extrinsics false` | 1.0 | — | 236 (6.9%) | 3178 | 11846 m | 0.22, 0.64, 0.03 |
| D | 08-03 | `aerial_best` (= B) | 1.0 | 231 | 120 (3.0%) | 694 | 68212 m | 3.66, 0.13, 0.47 |
| E | 08-03 | `aerial_best` (= B) | 1.0 | 246 | 399 (10.1%) | 3953 | 8031 m | 0.20, 0.44, 0.07 |
| F | 08-03 | `aerial_best` (= B) | 1.0 | — | 1321 (33.3%) | 11929 | 816 m | 0.019, 0.001, 0.034 |
| G | 08-03 | `aerial_best` (= B) | **0.5** | 229 | 1741 (43.8%) | 16690 | 730 m | 0.032, −0.012, 0.037 |
| H | 08-03 | `aerial_best` (= B) | **0.5** | 229 | 1729 (43.5%) | 16875 | 750 m | 0.038, −0.008, 0.036 |
| I | 08-03 | `aerial_staticinit`, gravity 9.81 | 0.5 | 332 | 1724 (44.6%) | 17481 | 719 m | 0.051, −0.003, 0.027 |
| **J** | 08-03 | **`aerial_staticinit`, gravity 9.7801** | **1.0** | **332** | **1741 (45.0%)** | **17713** | **666 m** | 0.045, −0.010, 0.002 |
| **K** | 08-03 | **`aerial_staticinit`, gravity 9.7801** | **1.0** | **332** | **1813 (46.9%)** | **18322** | **673 m** | 0.045, −0.024, 0.000 |

B/D/E/F are the **same bag and same config** — 41.9 / 3.0 / 10.1 / 33.3 %.

---

## Finding 1 — aerial triangulation gates (A → B)

25× more MSCKF updates, 36× less drift, `ba` from −1.67 to 0.11 (i.e. it stopped
absorbing unexplained drift). Confirmed later by G–K reproducibly reaching ~44%.

**Theory.** Triangulation quality is set by the parallax angle ≈ baseline/depth.
Propagating pixel noise gives `σ_depth ≈ (depth² / (baseline·f))·σ_px`, so depth
error grows as depth **squared** but falls only linearly with baseline. With
**R = depth/baseline**:

```
σ_depth/depth ≈ R · σ_px / f
```

- `fi_max_baseline` is a **ratio**, not a distance — it caps relative depth
  error (40 → 4 %, 500 → 49 %).
- `fi_max_cond_number` scales as **R²**, so it must be raised together with
  max_baseline (500² = 250000, hence 200000).
- `fi_max_dist` must exceed altitude by the depth-uncertainty factor, not
  merely clear it.

---

## Finding 2 — the estimator was non-deterministic (D, E, F vs G, H)

Same bag, same config, same binary, four runs at `--rate 1.0`: **3.0 / 10.1 /
33.3 / 41.9 %**. A 39-point spread makes every single-run A/B meaningless.

**What it was NOT** (each ruled out by measurement):

- bag or config drift — bag fingerprint identical, configs untouched since
  08-01 14:36, binary from 07-21
- bad init orientation — near-identical across runs (−0.7070,0.7073 vs
  −0.7076,0.7065), healthy `ba` at init in both
- dropped frames — KLT calls 4194 / 4201 / 4200 / 4200, MSCKF update counts
  3955–3971. **No frames or updates were ever skipped.**
- sim time / clock source — frame decimation keys off **message header stamps**
  (`ROS2Visualizer.cpp:501`), not wall clock
- falling behind — worst 7.5–12 ms

**What it WAS.** `try_to_initialize` (`VioManagerHelper.cpp:83`) runs the
initialiser in a background thread and **drops frames that arrive while it is
running**. Combined with a Ceres solve that is bounded by *wall-clock time*
(`init_dyn_mle_max_time: 0.05`) and multi-threaded
(`init_dyn_mle_max_threads: 6`), which frames reach the initialiser depends on
machine load. Init landed on frame **231 vs 246**, and in the marginal aerial
geometry that difference amplified into 3 % vs 42 %.

Playing at `--rate 0.5` gave each attempt time to finish → init at frame 229
twice with byte-identical state → 43.8 / 43.5 %.

---

## Finding 3 — static init fixes it at full rate (J, K)

`init_dyn_use: false` removes the Ceres solve entirely. Static init waits for an
accel jerk in the **data**, so it is playback-rate independent.

```
init@frame 332 at 0.5x, and at 1.0x twice -- identical every time
orientation -0.7071, 0.7071, -0.0028, 0.0013
velocity     0.0000,  0.0000,  0.0000        <- asserted, not estimated
```

Velocity being *asserted* is the key: it was the free parameter that varied
across dynamic runs (−0.027 / +0.013 / −0.042) and seeded all the divergence.

Residual spread 1.9 points (45.0 vs 46.9) against dynamic's 39 at the same rate.
Not perfectly deterministic — multi-threaded OpenCV and the async update path
remain — but small enough that config changes are now measurable.

**Static init is also the best performer**: higher feature usage and ~10 % less
drift than dynamic at half rate, at twice the speed.

Caveat: static init needs a genuine jerk. It failed 173–181 times ("no accel
jerk detected") before latching at frame 332, ~100 frames later than dynamic. On
a bag that starts already airborne it may never latch at all.

---

## Finding 4 — gravity constant was being absorbed into the accel bias

Bangalore local gravity, IGF-1980 at 12.9716° latitude minus a free-air
correction for 920 m altitude:

```
9.780327·(1 + 0.0053024·sin²φ − 0.0000058·sin²2φ) = 9.78293
free-air  −3.086e-6 × 920 m                       = −0.00284
LOCAL GRAVITY                                      =  9.7801 m/s²
```

The config had 9.81 — **0.0299 m/s² high**, the same order as the healthy `ba`
values. Correcting it:

```
gravity 9.81    -> init ba_z = 0.0355
gravity 9.7801  -> init ba_z = 0.0058     difference 0.0297  vs  0.0299 expected
```

Removed 0.0297 of the 0.0299 — the bias was carrying the constant error almost
one-for-one. `ba` is now a clean health indicator.

---

## Wall-clock dependencies in OpenVINS (audit)

Four places where wall clock changes **behaviour**, not just timing printouts:

1. `ROS2Visualizer.cpp:453` — `if (thread_update_running) return;` — camera
   updates deferred while a previous update runs. *Does not drop them* (counts
   proved this), but changes when they are applied.
2. `VioManagerHelper.cpp:83` — same pattern for **initialisation**. **This was
   the culprit.**
3. `DynamicInitializer.cpp:629-631` — `max_solver_time_in_seconds` (wall-clock
   bounded) and `num_threads` (nondeterministic reduction order). Line 1010 the
   same for covariance recovery.
4. `ROS2Visualizer.cpp:159` — detached 20 Hz image-publish loop. Visualisation
   only.

**Not** wall-clock dependent: frame decimation (`track_frequency`, header
stamps), the filter maths, sim time / `--clock`.

---

## Nadir geometry (drives the trajectory spec)

`base_line_max` counts only motion **perpendicular to the feature ray**
(`FeatureInitializer.cpp:353`). For a downward camera:

- horizontal translation → full baseline
- **vertical translation → ~zero** — parallel to the ray, so a climbing drone is
  geometrically hovering
- rotation of any kind → exactly zero

```
v_min = altitude / (R_target · T)        T = max_clones/track_frequency = 15/21 = 0.71 s
```

| altitude | R=40 | R=100 | R=500 |
|---|---|---|---|
| 40 m | 1.4 m/s | 0.56 m/s | 0.11 m/s |
| 80 m | 2.8 m/s | 1.1 m/s | 0.23 m/s |

**Camera rate does not help.** At 60 Hz input with `track_frequency: 21` only
33 % of frames are used, and that is correct — using all 60 Hz would shorten the
clone window from 0.71 s to 0.25 s and **triple R**. More frames inside the same
window buys nothing geometrically; baseline is window *duration* × speed.
The useful lever is the opposite: `max_clones 15 → 25` gives T = 1.19 s.

**Scale observability:** monocular scale comes from the accelerometer, so
constant-velocity flight makes it unobservable. A perfect survey lawnmower is
close to the degenerate case — include mild speed variation along legs.

---

## INVALIDATED — drawn from single runs inside a 39-point distribution

- **Run C** (`calib_cam_extrinsics: false` "made things worse") — single 1.0×
  draw. Needs redoing with static init.
- **Box-bag diagnosis** (`~/flights/70x30x50m box mission`, converted to
  `field_data/box_70x30x50m`) — the climb-phase story (baro showed 54 % of the
  flight climbing or descending, and vertical motion gives a nadir camera zero
  baseline) is geometrically sound, but the numbers behind it were single draws.
- Finding 1's **magnitude** (1.7 % → 41.9 %) — direction is certain and later
  confirmed reproducibly at ~44 %, but the exact ratio came from single runs.

---

## Still to test, now that measurement works

- `max_clones: 15 → 25` — lengthens the baseline window 0.71 → 1.19 s. The
  cheapest remaining geometric lever.
- `feat_rep_msckf: GLOBAL_3D → ANCHORED_MSCKF_INVERSE_DEPTH` — better
  conditioned for distant features.
- `up_msckf_sigma_px: 1 → 1.5` — R=500 features have genuinely larger residuals.
- `histogram_method: HISTOGRAM → NONE` — flight config uses NONE, amplifies grain.
- Re-run C and the box bag with static init.

---

## Operational gotchas (each cost real time)

- **`run_openvins_bag.sh` keeps OpenVINS alive after playback by design.** Left
  running, the next invocation is refused by its own duplicate guard — which
  silently turns an A/B into two no-ops that still look like runs.
- **A zombie `[run_subscribe_m] <defunct>` also matches that guard.** It cannot
  be reaped if its `ros2 launch` parent is suspended (state `Tl`) — kill the
  parent.
- **Never `pgrep -f` a pattern matching your own command line** — use a
  bracketed pattern (`[r]un_openvins_bag`) or you kill your own shell.
- **`cv::FileStorage` and colons**: a colon inside a *trailing* comment makes the
  value parse as empty ("invalid boolean type of []") and silently default.
  Full-line comments are fine.
- **`rosbags-convert` writes `offered_qos_profiles: []`** (a list); Humble's
  rosbag2 expects a string. `ros2 bag play` fails outright and OpenVINS sits
  receiving nothing. Fix to `""`.
- **cv2 `.real()` on a boolean node returns DBL_MAX** — useless for verifying
  booleans. Grep the file instead.

## Tools

- `repeat_openvins.sh` — N reps × several configs, appends to CSV
- `rate_reproducibility.sh` — playback-rate comparison
- `staticinit_determinism.sh` — the J/K test above
- `mk_imucam_bag.py` — ROS2 → ROS1 for Kalibr
- `html_to_pdf.py` — report renderer (no pandoc/latex/wkhtmltopdf on this box)
