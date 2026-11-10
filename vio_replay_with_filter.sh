#!/usr/bin/env bash
# Replay a bag's raw /imu0 + camera through imu_filter_node and OpenVINS (mono) reading /imu0_filtered.
#   vio_replay_with_filter.sh BAG [extra args for run_openvins_mono_bag.sh, e.g. --cam 0 --no-rviz --no-gps -r 1]
# Env:
#   FILT_ORDER (4)  FILT_CUTOFF (100)  FILT_OUT_HZ (300)   filter params (plain integers are fine; converted to decimals here)
#   CFG        estimator_config.yaml to use (default openvins_config/ar0234_mono_cam0_filtered = old calibration;
#              the 2026-11-06 Kalibr solve is openvins_config/ar0234_mono_cam0_nt_filtered/estimator_config.yaml)
#   REC_OUT    if set, record /ov_msckf/odomimu of this replay to the bag dir REC_OUT (for ov_tap_metrics.py)
# Needs imu_filter_node built (repo install/). Refuses to run if an imu_filter_node already exists (two nodes interleave on
# /imu0_filtered and corrupt OpenVINS). Ctrl+C stops everything this script started; stop_replay.sh does too.
set -u
REPO="$HOME/vio_ar0234_icm20948"; L=/tmp/vio_logs; mkdir -p "$L"
[ $# -ge 1 ] || { sed -n '2,12p' "$0"; exit 1; }
BAG="$1"; shift
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
pgrep -x imu_filter_node >/dev/null && { echo "an imu_filter_node is already running (pids: $(pgrep -x imu_filter_node | tr '\n' ' ')) -- run ~/vio_tools/stop_filter_node.sh first" >&2; exit 1; }
CFG="${CFG:-$REPO/openvins_config/ar0234_mono_cam0_filtered/estimator_config.yaml}"
[ -f "$CFG" ] || { echo "config missing: $CFG" >&2; exit 1; }
[ -n "${REC_OUT:-}" ] && [ -e "$REC_OUT" ] && { echo "$REC_OUT already exists" >&2; exit 1; }
FC=$(awk -v v="${FILT_CUTOFF:-100}" 'BEGIN{printf "%.3f", v}'); FO=$(awk -v v="${FILT_OUT_HZ:-300}" 'BEGIN{printf "%.3f", v}')
echo "[replay] filter: order ${FILT_ORDER:-4}, cutoff $FC Hz, out $FO Hz | config $CFG | bag $BAG"
"$(ros2 pkg prefix imu_filter_node)/lib/imu_filter_node/imu_filter_node" --ros-args \
   -p order:="${FILT_ORDER:-4}" -p cutoff_hz:="$FC" -p out_rate_hz:="$FO" > "$L/imu_filter_node.log" 2>&1 &
FPID=$!
RPID=""
cleanup() { kill -INT $FPID 2>/dev/null; [ -n "$RPID" ] && kill -INT $RPID 2>/dev/null; wait $FPID 2>/dev/null; [ -n "$RPID" ] && wait $RPID 2>/dev/null; true; }
trap cleanup EXIT
sleep 1
if [ -n "${REC_OUT:-}" ]; then
  ros2 bag record -o "$REC_OUT" /ov_msckf/odomimu > "$REC_OUT.rec.log" 2>&1 & RPID=$!
  echo "[replay] recording /ov_msckf/odomimu -> $REC_OUT"
  sleep 2
fi
"$REPO/run_openvins_mono_bag.sh" --config "$CFG" "$@" "$BAG"
