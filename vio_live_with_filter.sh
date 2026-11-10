#!/usr/bin/env bash
# LIVE VIO on the filtered IMU: sensors (IMX 1 kHz stream @ 3 MHz, free-running cameras) -> imu_filter_node -> OpenVINS mono + RViz.
#   vio_live_with_filter.sh [--cam 0|1] [--no-rviz]
# Starts, in order (sensors + filter node are detached, OpenVINS runs in the foreground):
#   1. vio_bringup.launch.py   IMX stream mode (IMU_STREAM=1, 128 B frames, 1000 us loop, 3 MHz) + both cameras (CAM_TRIGGER_MODE=0, PWM must be OFF)
#   2. imu_filter_node         /imu0 (1 kHz raw) -> /imu0_filtered (Butterworth FILT_ORDER=4, FILT_CUTOFF=100 Hz, FILT_OUT_HZ=300)
#   3. run_openvins_mono.sh    config openvins_config/ar0234_mono_cam<N>_filtered (rostopic /imu0_filtered)
# Initialisation (static init): hold the rig still ~3 s, then move it. Watch for "successful initialization" in /tmp/vio_logs/openvins_latest.log.
# Ctrl+C stops OpenVINS/RViz only; vio_live_stop.sh stops everything. Logs: /tmp/vio_logs/{sensors,imu_filter_node,vio_live}.log
set -u
REPO="$HOME/vio_ar0234_icm20948"; L=/tmp/vio_logs; mkdir -p "$L"
CAM=0; RVIZ_ARG=""
while [ $# -gt 0 ]; do
  case "$1" in
    --cam) CAM="$2"; shift ;;
    --no-rviz) RVIZ_ARG="--no-rviz" ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown arg $1"; exit 1 ;;
  esac; shift
done
fail() { echo "[vio_live] ERROR: $*" >&2; exit 1; }
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
SUF="_nt"; [ "${CAL:-nt}" = old ] && SUF=""   # CAL=nt (default) = Kalibr solve of 2026-11-06, CAL=old = Sep-10 chain
CFG="$REPO/openvins_config/ar0234_mono_cam${CAM}${SUF}_filtered/estimator_config.yaml"
[ -f "$CFG" ] || fail "config missing: $CFG (only ar0234_mono_cam0_filtered exists so far)"
pgrep -x imx5_spi_node >/dev/null && fail "sensors already running (vio_live_stop.sh first)"
pgrep -x imu_filter_node >/dev/null && fail "an imu_filter_node is already running (pids: $(pgrep -x imu_filter_node | tr '\n' ' ')) -- two filter nodes interleave on /imu0_filtered and corrupt OpenVINS; run vio_live_stop.sh / pkill -x imu_filter_node first"
[ "$(cat /sys/class/pwm/pwmchip2/pwm0/enable 2>/dev/null)" = 1 ] && fail "camera trigger PWM is ON; this live setup needs it OFF (free-running cameras)"

export CAM_TRIGGER_MODE=0
export IMU_STREAM="${IMU_STREAM:-1}" IMU_FRAME_LEN="${IMU_FRAME_LEN:-128}" IMU_POLL_US="${IMU_POLL_US:-1000}" IMU_SPI_HZ="${IMU_SPI_HZ:-3000000}"
ros2 daemon stop >/dev/null 2>&1 || true
echo "[vio_live] starting sensors (IMU stream mode ${IMU_FRAME_LEN} B / ${IMU_POLL_US} us / ${IMU_SPI_HZ} Hz SPI)"
setsid nohup ros2 launch "$REPO/vio_bringup/vio_bringup.launch.py" > "$L/sensors.log" 2>&1 < /dev/null &
for p in imx5_spi_node vizionsdk_camer; do
  t0=$SECONDS; until pgrep -x "$p" >/dev/null; do sleep 1; (( SECONDS - t0 > 30 )) && fail "$p did not start (see $L/sensors.log)"; done; echo "  $p up"
done
echo "[vio_live] waiting for camera ${CAM} to publish"
t0=$SECONDS
until timeout 4 ros2 topic echo --once --field width "/cam${CAM}/image_raw" >/dev/null 2>&1; do
  sleep 1; (( SECONDS - t0 > 45 )) && fail "cam${CAM} not publishing (see $L/sensors.log)"
done
echo "  cam${CAM} publishing ($((SECONDS - t0)) s); settling 5 s"; sleep 5

echo "[vio_live] starting imu_filter_node (order ${FILT_ORDER:-4}, ${FILT_CUTOFF:-100} Hz, out ${FILT_OUT_HZ:-300} Hz)"
setsid nohup "$(ros2 pkg prefix imu_filter_node)/lib/imu_filter_node/imu_filter_node" --ros-args \
   -p order:="${FILT_ORDER:-4}" -p cutoff_hz:="$(awk -v v="${FILT_CUTOFF:-100}" 'BEGIN{printf "%.3f", v}')" -p out_rate_hz:="$(awk -v v="${FILT_OUT_HZ:-300}" 'BEGIN{printf "%.3f", v}')" > "$L/imu_filter_node.log" 2>&1 < /dev/null &   # the node rejects integer-typed doubles (75 -> must be 75.000)
sleep 3
grep -a "in .* Hz" "$L/imu_filter_node.log" | tail -1 | cut -c1-160

echo "[vio_live] starting OpenVINS mono cam${CAM} on /imu0_filtered ${RVIZ_ARG}"
echo "[vio_live] INITIALISE: hold the rig still ~3 s, then move it."
exec "$REPO/run_openvins_mono.sh" --cam "$CAM" --config "$CFG" $RVIZ_ARG
