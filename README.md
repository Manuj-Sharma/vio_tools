# VIO analysis tools — OV9281 + IMX-5, GPS-denied agri-drone

Scoring, plotting and experiment-driver scripts for the OpenVINS replay work.
These operate on data in `~/rosbags` and configs in `~/sensor_ws`; nothing here
is built or flown. Paths are absolute — the tools assume this directory stays at
`~/rosbags/calib/tools`.

## Scoring — start here

    vs_gps.py LOG...          scale + RMS against the GPS track. THE metric.
    plot_vs_gps.py LOG...     same, plotted; pass every rep to see the spread
    score_box.py LOG...       box dimensions vs the flown 267 x 131 m
    score_baro.py LOG...      barometer residual split by flight phase

`vs_gps.py` under-reports scale by ~2.7%: it spreads poses evenly across the GPS
span, but the filter initialises ~3.4 s late and its updates are not uniform.
Comparisons between runs are unaffected; absolute error is overstated. A proper
fix needs a bag timestamp on the `p_IinG` log line. `plot_vs_gps.py` inherits the
same convention deliberately, so the two agree.

MSCKF percentage is NOT an accuracy metric — runs at 69.4/66.1/71.1% produced box
errors of +35/+2/+36%. Below ~40% it only tells you the filter died. Judge on
max-distance as well: run 181121 passed at 42.5% MSCKF while sitting 1225 m off.

## Running experiments

    run_flight_B.sh [REPS]        N reps of a config, scored (refuses no-baro configs)
    run_gold_px2.sh CFG [REPS]    headless reps with a collapse verdict
    run_rviz.sh CFG               replay + GPS truth overlay + scoring on Ctrl+C
    mem_probe.sh CFG [REPS]       reps with memory counters beside the result

This pipeline is NOT deterministic — `flight_D_best` spans scale 0.857-0.984 and
`flight_A` spans 0.539-1.075. Always run 3+ reps, and judge on the FAILURE RATE
rather than the mean.

Eliminated as the cause by direct test — do not re-run these:

    num_opencv_threads: 1     3 distinct hashes over 3 reps
    RANSAC seeding (08-06)    3 distinct; fix kept anyway, VIO commit 1b5c5f4
    --rate 0.5                3 distinct AND worse: MSCKF 42% vs 63%,
                              max dist 175-1450 m vs 176-422 m
    CPU / subscriber load, frame selection, memory, the binary, the bag

WHERE runs diverge: bias traces are identical to 3 decimals through the climb
(0-70 m, frac 0.05-0.10 of the run) and separate at the top of climb, ~100 m,
frac 0.20. The ill-conditioned nadir geometry AMPLIFIES a difference there — but
amplification cannot create one, so a source still exists upstream.

Prime remaining suspect: `run_subscribe_msckf.cpp:85` hardcodes
`use_multi_threading_subs = true` AFTER `print_and_load`, so the config key
cannot switch it off. The IMU (238 Hz) and camera (30 Hz) callbacks then run
concurrently and the scheduler decides the order measurements reach the filter.
Upstream knows: `ros1_serial_msckf.cpp:67` carries
`// params.use_multi_threading_pubs = 0; // uncomment if you want repeatability`.

## Sensors and data

    gps_truth_rviz.py             GPS truth as a cyan Path on /ov_msckf/pathgt
    plot_lidar_baro.py            AGL lidar vs barometer vs GPS
    make_baro_bag.py CSV OUT      IMX-5 baro CSV -> ROS2 bag
    bag_stats.py / bag_quality.py bag timing audits
    measure_feature_depth.sh      triangulated feature heights (perturbs the run —
                                  diagnostic only, never in a scoring loop)

The lidar reads `inf` above its 95 m max_range and the mission cruises at ~107 m,
so 66% of returns are unusable and the entire box has no coverage. It is valid
100% below 60 m — climb and descent only.

## IMU spectral analysis

    imu_spectrum.py BAG           Welch PSD, band powers, aliasing flag
    imu_spectrogram.py BAG        STFT — use this on flight data
    imu_gap_compare.py BAG        gap-closed vs resampled PSD, overlaid
    imu_plot.py BAG               raw time series (--t0/--t1 to zoom)
    allan_variance.py BAG         Allan deviation off a static log
    imu_vibe_vs_alt.py BAG        band level vs altitude and climb rate
    imu_window_probe.py BAG --win yaw, tilt and thrust proxy over a window
    record_imu.sh [SEC] [NAME]    log /imu0 to a bag with fan RPM alongside

READ THE DROPOUT LINE FIRST. The IMU delivers ~238 Hz against a 250 Hz nominal,
so ~5% of the grid is missing. Everything here resamples onto the true grid
before transforming; anything you write that does not will misplace lines by
~0.8 Hz and lose 15x of contrast. That bug produced a false aliasing conclusion
in 7ca33dc, retracted in 49d6f0a.

Welch vs STFT: Welch averages the time axis away, which is optimal on a bench and
wrong in flight. Broadband level swings 20 dB across a single flight, so a Welch
curve of a flight bag is an average of things that never coexisted. Use
`imu_spectrogram.py` when the question contains "when" or "during".

Known contamination — do not re-derive:

    16.669 Hz in any bench log   the Jetson fan (= 1000 rpm). Tracks fan RPM;
                                 confirmed at 30.060 Hz @ 1806 rpm. Absent in
                                 flight. Contaminates Allan noise densities, so
                                 08-06's N values are upper bounds.
    peaks near Nyquist           check imu_gap_compare.py before believing them
    34-45 Hz humps in flight     broadband, NOT smeared swept prop tones

Flight vibration tracks TILT (r = +0.802, vib_dB = 0.52*deg - 20.8), not altitude
(+0.006) or climb rate (+0.123). 0.05 g RMS level, 0.49 g RMS at 29 deg.

## Records

    AERIAL_TUNING_LEDGER.md       08-03 sweeps. Its measurements stand; its
                                  RANKING is superseded (scored on MSCKF%).
    records/2026-08-07_imu_spectral.md
                                  the fan, the gap bug, and flight vibration vs
                                  tilt. Retracts the aliasing claim in 7ca33dc.
    ../results_20260805/          08-05 session: 41 runs, findings, what is
                                  disproven. Read before re-deriving anything.
