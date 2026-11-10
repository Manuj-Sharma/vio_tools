# Replay of real flights against the 110545 mosaic (2026-11-10)
Script: ~/ardupilot_gazebo/scripts/map_matching/replay_real_bag.py (matcher.py unchanged). Map: ../mosaic_110545.png + .json (built by ~/vio_tools/mosaic_from_bag.py).
VIO stand-in = FC position (+ injected linear drift, 0.15 E / -0.10 N m/s); truth = FC position. No OpenVINS in these bags. Map and truth are both GPS-fused, so this is NOT an absolute-accuracy test.
Default matcher settings (win 14 s, sig_p 0.5, sig_b 0.1, nccmin 0.2). Keyframes every 0.66 s above --min-alt.

| run | keyframes (s of data) | registrations applied | NCC | final offset estimate (E, N) m | injected drift at end (E,N) | error after matcher mean / max |
|---|---|---|---|---|---|---|
| 165506 drift 0 (>=60 m)        | 92 (61 s)  | 24/24 | 0.41-0.55 | (-1.7, +2.9)  | 0          | 3.15 / 4.97 |
| 165506 drift 0, h-scale 1.056  | 92 (61 s)  | 24/24 |           | (-1.3, +4.2)  | 0          | 3.36 / 5.05 (not better) |
| 165506 with drift              | 92 (61 s)  | 24/24 |           | (-10.7, +8.8) | (+9.1,-6.1) | 3.15 / 4.66  (VIO alone 5.5 / 10.9) |
| 092757 drift 0 (>=50 m)        | 131 (87 s) | 37/37 |           | (-3.0, +3.0)  | 0          | 3.57 / 7.53 |
| 092757 with drift              | 131 (87 s) | 37/37 |           | (-16.0, +11.6) | (+13.1,-8.7) | 3.59 / 8.52 (VIO alone 7.8 / 15.7) |

Drift recovery = (drift-run offset) - (drift-0 offset): 165506 (-9.0, +5.9) vs expected (-9.1, +6.1); 092757 (-13.0, +8.6) vs expected (-13.1, +8.7). Tracks the injected drift to ~0.2 m.
Systematic bias with no drift: (-1.7, +2.9) and (-3.0, +3.0) m on two different flights/days, ~3.8 m: a mosaic-vs-GPS georeferencing offset (not noise).
Caveats: clean linear drift and GPS-quality attitude/height (real VIO has attitude and scale errors); only two flights, both inside the mosaic area; 092757 only reaches 69 m; camera mount taken from the survey flight.

## Real OpenVINS poses (2026-11-10, later)
OpenVINS: frozen px8 config (~/rosbags/claude_tmp/bagE/cfg_frozen_px8_zupt_rw10_ts0_1core), ~/openvins_ws_new, mono cam0, real-time replay pinned to core 2, ONE run per bag
(~/Manuj/mosaic_110545/openvins/run_openvins_replay.sh; recorded odom_165506, odom_092757). Start offsets 19 s (165506) and 125 s (092757).
VIO -> FC frame: yaw from the FC attitude, origin = FC position at the first VIO pose. Truth = FC position. Matcher default settings.
| run | VIO path vs GPS path | registrations applied | VIO alone mean/max/final (m) | + matcher mean/median/max/final (m) | <10 m |
|---|---|---|---|---|---|
| 165506, heading aligned once at start    | 1035 / 1004 m | 13/24 | 12.0 / 24.7 / 18.2 | 11.2 / 10.5 / 23.1 / 20.1 | 45% (VIO 41%) |
| 165506, heading re-aligned continuously  | same          | 24/24 | 10.3 / 13.3 / 11.7 | 5.9 / 5.4 / 13.0 / 4.1   | 88% (VIO 41%) |
| 092757, once                             | 1099 / 425 m, ended at z -85 m | 15/37 | 58 / 116 / 57 | 42 / 41 / 112 / 21 | 39% (VIO 14%) |
| 092757, continuous                       | same          | 17/37 | 55 / 115 / 54 | 38 / 22 / 110 / 21 | 44% (VIO 14%) |
Diagnosis 165506: VIO scale 0.94-1.04 (good) but the whole VIO track rotates +15..21 deg against the FC heading between takeoff and the passes; tiles are placed by VIO position but oriented
by FC heading, so a 20 deg yaw drift destroys the mosaic (NCC 0.15-0.36). Continuous alignment from the FC compass yaw (--align continuous, uses no GPS) restores NCC 0.32-0.53.
The ~3.8 m constant mosaic-vs-GPS bias (see above) is included in the matcher errors. 092757: VIO scale error ~2.6x and negative height: beyond what a 14 s mosaic matcher can recover.

## Causality check (2026-11-10)
The first continuous-heading runs smoothed the VIO-vs-FC yaw with a CENTRED 10 s median (uses 5 s of future). Default is now a trailing (causal) median; --align-centered keeps the old one.
165506, real VIO: causal: 24/24 applied, + matcher mean 5.90 / median 5.53 / max 13.46 / final 4.33 m, <10 m 88% (VIO alone 10.12 / 13.93 / 12.44);  centred: mean 5.88 / median 5.39 -> no difference.
What is and is not real time: OpenVINS was replayed at 1x real time; the matcher is replayed offline from cached keyframes (1 s wall for 61 s of data) but processes one keyframe at a time from past frames only.
Per keyframe ortho 3 ms + per registration ~15 ms on this x86 PC (Jetson not measured). No live (streaming ROS) run of OpenVINS + matcher together has been done.
