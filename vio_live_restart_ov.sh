#!/usr/bin/env bash
# Restart ONLY OpenVINS + RViz of a running vio_live_with_filter.sh stack (sensors and imu_filter_node stay up).
#   vio_live_restart_ov.sh [raw]       default = filtered config (/imu0_filtered);  "raw" = unmodified ar0234_mono_cam0 on /imu0
# Lives in a file so the pkill -f patterns cannot match the caller's command line.
set -u
REPO="$HOME/vio_ar0234_icm20948"
pgrep -x imx5_spi_node >/dev/null || { echo "sensors are not running -- start vio_live_with_filter.sh first"; exit 1; }
[ "${1:-}" = raw ] || pgrep -x imu_filter_node >/dev/null || { echo "imu_filter_node is not running"; exit 1; }
pkill -INT -f "[r]un_openvins_mono.sh"
sleep 2
pkill -x run_subscribe_m
pkill -x rviz2
sleep 2
export DISPLAY="${DISPLAY:-:0}"
# CAL=nt (default) = Kalibr solve of 2026-11-06; CAL=old = the Sep-10 chain.
SUF="_nt"; [ "${CAL:-nt}" = old ] && SUF=""
if [ "${1:-}" = raw ]; then CFG="$REPO/openvins_config/ar0234_mono_cam0${SUF}/estimator_config.yaml"
else CFG="$REPO/openvins_config/ar0234_mono_cam0${SUF}_filtered/estimator_config.yaml"; fi
echo "restarting OpenVINS with $CFG"
setsid nohup "$REPO/run_openvins_mono.sh" --cam 0 --config "$CFG" > /tmp/vio_logs/vio_live.log 2>&1 < /dev/null &
sleep 12
pgrep -x run_subscribe_m >/dev/null && echo "OpenVINS up" || echo "OpenVINS NOT running"
pgrep -x rviz2 >/dev/null && echo "RViz up" || echo "RViz NOT running"
