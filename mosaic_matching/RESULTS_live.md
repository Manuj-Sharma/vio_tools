# Live test on bag 165506 (2026-11-10): bag at 1x -> real OpenVINS (frozen px8 cfg, core 2) + live_real_node.py (cores 4-7) against the 110545 mosaic
Script: ~/ardupilot_gazebo/scripts/map_matching/live_real_node.py, run with ./run_live_real.sh RUN (this folder). Everything causal; arrival time is the clock; truth = FC position (scoring only). One OpenVINS run per row (VIO is not repeatable).
| run | images received | keyframes | registrations applied | VIO alone mean/max (m) | + matcher mean/median/max/final (m) | <10 m | latency (all / with registration) |
|---|---|---|---|---|---|---|---|
| run1 best-effort images | 500 (~6%) | 38 | 10/10 | 10.9 / 14.9 | 6.5 / 5.7 / 14.7 / 5.5 | 84% (VIO 36%) | 13 / 26 ms |
| run2 + threads, depth 30 | 606 (~7%) | 54 | 14/14 | 9.0 / 12.7 | 5.7 / 5.5 / 12.4 / 5.4 | 90% (VIO 70%) | 14 / 28 ms |
| run3 + raw subscription  | 699 (9%)  | 58 | 15/15 | 16.3 / 19.5 | 7.6 / 5.7 / 19.5 / 5.5 | 74% (VIO 0%) | 15 / 28 ms |
| **run4 RELIABLE images** | **8022 (98%)** | **91** | **24/24** | **9.4 / 13.3** | **5.8 / 5.5 / 13.3 / 5.2** | **89% (VIO 62%)** | **15 / 28 ms (p95 32, max 37)** |
Cause of the frame loss in runs 1-3: the bag player publishes /cam0/image_raw RELIABLE; a BEST_EFFORT subscriber receives ~7% of the 0.9 MB frames (bare counter test: best-effort 3.4 Hz, reliable 51.1 Hz). Fixed in run4 (reliable QoS).
Run4 = offline replay result (24/24, mean 5.9 m): the live chain reproduces it. Matcher never busy (0 dropped). No GPU, ~3 ms ortho + ~15-28 ms per registration on this x86 PC (Jetson not measured).
Caveats: one flight, one OpenVINS run per row; truth is GPS-fused FC position; includes the ~3.8 m constant mosaic bias; heading re-aligned from FC compass yaw (trailing 10 s median, causal); height from FC; mount from the survey flight.

## GUI (2026-11-10 later): rviz + OpenCV dashboard, with the raw GPS track
Run: `cd ~/Manuj/mosaic_110545/live && ./run_live_real_gui.sh NAME` (DISPLAY :1; starts OpenVINS, the node with --gui --video, rviz2 with live_real.rviz, then the bag at 1x; rviz is left open afterwards).
rviz (fixed frame map, x east / y north from this flight's takeoff, north up): /mm/map (mosaic, 'map' colour scheme, inverted grey), /mm/path_fc (white), /mm/path_gps (yellow, raw /fc/gps1), /mm/path_vio (orange), /mm/path_corrected (blue),
/mm/markers (current positions, matcher search window, error text), /mm/mosaic_vs_map (camera mosaic | map patch), plus OpenVINS /ov_msckf/trackhist. Dashboard = same story in one OpenCV window, saved as NAME/dashboard.mp4.
gui2 (final): 98% frames, 24/24 applied, VIO alone 13.3 m -> 6.6 m mean (median 5.6), 80% <10 m, latency mean 19 ms (37 ms with a registration, max 52) with rviz + dashboard running.
Screenshots: gui2/rviz_screenshot.png, gui2/dashboard_last_frame.png. rviz logs a harmless Mesa shader error for the Map display; it still draws.

## 3D view (2026-11-10 later)
Paths/markers now carry real height (z = height above takeoff: FC = -D, GPS = altitude minus first fix, VIO = FC height at the first VIO pose + VIO z increments, so it shows the VIO's own scale), plus a vertical line from the aircraft to the ground and the camera footprint on the ground (cyan, flat-ground projection).
live_real.rviz opens in a 3D Orbit view; the Views panel (bottom left) switches to "Top-down (north up)". rviz window title gets a '*' after launch, so capture it as "<config path>* - RViz".
gui3 screenshot: gui3/rviz_3d_screenshot.png.
