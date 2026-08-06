# Records

Written conclusions from the replay sessions. Kept in git because the analysis
directory `~/rosbags/calib/results_20260805/` is NOT versioned and lived on one
disk -- these are the findings, not the raw data.

    2026-08-05_session.md          41 runs: max_slam:0 closed, baro chi2
                                   self-lock found, and a long list of things
                                   DISPROVEN -- read before re-deriving anything
    baro_chi2_sweep_20260806.md    the multiplier sweep. mult 5 self-locks
                                   (4/10 healthy); 100 gave 3/3 at scale
                                   0.942-1.018 and is the provisional candidate

Raw logs, all_runs.csv and the lidar plot stay unversioned in
`~/rosbags/calib/results_20260805/` -- bulky and reproducible.
