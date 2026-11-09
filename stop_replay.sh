#!/usr/bin/env bash
# Stop a bag replay through OpenVINS (and the filter node). Lives in a file so the pkill -f patterns cannot match a caller's command line.
pkill -INT -f "[r]un_openvins_mono_bag.sh"
pkill -INT -f "[v]io_replay_with_filter.sh"
sleep 3
pkill -x run_subscribe_m
pkill -x imu_filter_node
sleep 1
pgrep -x run_subscribe_m >/dev/null && echo "run_subscribe_msckf STILL running" || echo "OpenVINS stopped"
pgrep -x imu_filter_node >/dev/null && echo "imu_filter_node STILL running" || echo "filter node stopped"
