# Own-survey mosaic as the map for the map matcher (2026-11-10)
Mosaic builder: ../mosaic_from_bag.py (flat-ground, pose-based, 5 fitted globals). Matcher side: ~/ardupilot_gazebo (branch map-tiff-ground) scripts/map_matching/ {replay_real_bag.py, live_real_node.py, live_real_viz.py}; matcher.py unchanged.
Files here: run_openvins_replay.sh (record OpenVINS odom from a bag at 1x), run_live_real.sh (headless live test), run_live_real_gui.sh / run_live_real_gui_bag.sh (rviz + dashboard; the _bag one takes BAG OFFSET ORIGIN MINALT CFG REMAP PLAYCAM),
live_real.rviz (3D orbit + top-down), shot.py (capture ONE named X window), mosaic_110545.json (sidecar; the 11 MB PNG is NOT committed: rebuild with mosaic_from_bag.py), RESULTS_*.md (numbers and caveats).
Scripts assume the data layout under ~/Manuj/mosaic_110545 and ~/rosbags; adjust paths if moved.
