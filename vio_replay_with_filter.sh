#!/usr/bin/env bash
# Replay a bag's raw /imu0 + camera through imu_filter_node and OpenVINS (mono) reading /imu0_filtered.
#   vio_replay_with_filter.sh BAG [extra args for run_openvins_mono_bag.sh, e.g. --cam 0 --no-rviz --no-gps -r 1]
# Config: openvins_config/ar0234_mono_cam0_filtered (= ar0234_mono_cam0 with rostopic /imu0_filtered, update_rate 300).
# Filter params via env: FILT_ORDER (4) FILT_CUTOFF (100) FILT_OUT_HZ (300). Needs imu_filter_node built (repo install/).
set -u
REPO="$HOME/vio_ar0234_icm20948"; L=/tmp/vio_logs; mkdir -p "$L"
BAG="$1"; shift
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
"$(ros2 pkg prefix imu_filter_node)/lib/imu_filter_node/imu_filter_node" --ros-args \
   -p order:="${FILT_ORDER:-4}" -p cutoff_hz:="${FILT_CUTOFF:-100.0}" -p out_rate_hz:="${FILT_OUT_HZ:-300.0}" > "$L/imu_filter_node.log" 2>&1 &
FPID=$!
trap 'kill -INT $FPID 2>/dev/null' EXIT
sleep 1
"$REPO/run_openvins_mono_bag.sh" --config "$REPO/openvins_config/ar0234_mono_cam0_filtered/estimator_config.yaml" "$@" "$BAG"
