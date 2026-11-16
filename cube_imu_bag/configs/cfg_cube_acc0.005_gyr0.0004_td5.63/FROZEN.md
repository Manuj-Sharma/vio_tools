**CORRECTION 2026-11-06 13:50: NOT REPEATABLE.** Repeat r2 (identical config, MD5 1e759996, same bag, same install) gave height ratio 1.90 @30 s, 2.0-2.07 for the whole climb (VIO 409 m vs GPS 200 m), vs 0.92-0.96 in run r1. Init was good in both (gravity error 0.19 deg and 0.04 deg). The 'best so far' label rests on ONE lucky run; the true distribution is unknown (N=2: 0.94 and 2.05).

FROZEN 2026-11-06 and labelled BEST SO FAR by the user ("freeze this config though...", "best so far"), while this run was going fine.
Basis for 'best': lowest and flattest climb error on the 200 m bag of any config tried (0.92-0.96 vs 1.13-1.24 for the px8 sibling and 1.45-1.55 for earlier configs); N=1, partial, no repeat yet.
Config = cfg_cam0_calib20261106_px2_extoff_tf40_acc02_330hz (cam0 mono). Do not edit; copy to change.

Settings:
- Camera calibration: kalibr cam0_imu_nt 2026-11-06 (reproj 1.24 px): fx 856.79 fy 846.29 cx 644.00 cy 317.15, radtan, T_cam_imu from the final solve, timeshift -23.6 ms (see SOURCE_*.yaml)
- Online calibration: ALL OFF (extrinsics, intrinsics, time offset)
- Camera pixel noise: up_msckf_sigma_px = up_slam_sigma_px = 2
- track_frequency 40 Hz (effective rate = every frame on the 25 Hz 200 m bag)
- IMU (IMX-5, /imu0): accel noise density 0.2, gyro noise density 1.4e-3, accel RW 3.333e-4, gyro RW 4.654e-5, update_rate 330 Hz
- gravity_mag 9.7801 (NOT 9.8000: 9.8 made the climb worse in a cam1 test)
- ZUPT only at the beginning (try_zupt true, zupt_only_at_beginning true, max disparity 3.0). ZUPT for the whole flight FAILS (treats the climb as stationary).
- Mono; launch with max_cameras:=1 use_stereo:=false; OpenVINS source: branch bagC-feature-diagnostics 1e24f52 with the OLD install ~/openvins_ws_new (NOT the per-axis build).

Run: OV_WS unset (default ~/openvins_ws_new):
  ~/vio_tools/run_bag.sh <tag> ~/rosbags/vio_20261104_110545 <this dir> 0 /cam0/image_30hz 0-23
  (200 m bag: offset 0, camera topic /cam0/image_30hz remapped to /cam0/image_raw; unpinned)

Result so far (run b1104_cam0_px2_tf40_acc02_330hz_r1, N=1, trace to 135 s of the 700 s bag, run still going when frozen):
- VIO/GPS height ratio 0.92-0.96 through the climb, hover at 187.5 m vs GPS 199.9 m, horizontal distance within ~7% of GPS (81 vs 88, 190 vs 203 m).
- accel bias z steady about -0.03 m/s2, x,y within +-0.04. Not yet scored over the whole flight; no repeat yet.
Same family with pixel noise 8 (r1, N=1, stopped at 195 s): ratio 1.13-1.24 climb, 1.00 in hover. Earlier configs on this bag: 1.45-1.55 and diverged.
