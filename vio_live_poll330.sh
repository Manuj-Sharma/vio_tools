#!/usr/bin/env bash
# LIVE VIO on the PREVIOUS driver setup: one-shot polling every 3 ms (~330-350 Hz /imu0, retry 500 us), NO filter node, raw /imu0,
# OpenVINS mono on the new (2026-11-06) calibration. Same sensors/cameras as vio_live_with_filter.sh otherwise.
#   vio_live_poll330.sh [--cam 0] [--no-rviz]          CAL=old for the Sep-10 chain; VERBOSITY=ALL for the full OpenVINS log
# Stop everything with vio_live_stop.sh. Logs: /tmp/vio_logs/{sensors,vio_live}.log
set -u
REPO="$HOME/vio_ar0234_icm20948"; L=/tmp/vio_logs; mkdir -p "$L"
CAM=0; RVIZ_ARG=""
while [ $# -gt 0 ]; do
  case "$1" in
    --cam) CAM="$2"; shift ;;
    --no-rviz) RVIZ_ARG="--no-rviz" ;;
    -h|--help) sed -n '2,6p' "$0"; exit 0 ;;
    *) echo "unknown arg $1"; exit 1 ;;
  esac; shift
done
fail() { echo "[vio_live_poll330] ERROR: $*" >&2; exit 1; }
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
SUF="_nt"; [ "${CAL:-nt}" = old ] && SUF=""
CFG="$REPO/openvins_config/ar0234_mono_cam${CAM}${SUF}/estimator_config.yaml"
[ -f "$CFG" ] || fail "config missing: $CFG"
pgrep -x imx5_spi_node >/dev/null && fail "sensors already running (vio_live_stop.sh first)"
[ "$(cat /sys/class/pwm/pwmchip2/pwm0/enable 2>/dev/null)" = 1 ] && fail "camera trigger PWM is ON; needs to be OFF (free-running cameras)"

export CAM_TRIGGER_MODE=0
export IMU_STREAM=0 IMU_POLL_MS=3 IMU_SPI_HZ="${IMU_SPI_HZ:-3000000}"
unset IMU_FRAME_LEN IMU_POLL_US
ros2 daemon stop >/dev/null 2>&1 || true
echo "[vio_live_poll330] starting sensors (one-shot poll every ${IMU_POLL_MS} ms, retry ${IMU_RETRY_US:-500} us, SPI ${IMU_SPI_HZ} Hz)"
setsid nohup ros2 launch "$REPO/vio_bringup/vio_bringup.launch.py" > "$L/sensors.log" 2>&1 < /dev/null &
for p in imx5_spi_node vizionsdk_camer; do
  t0=$SECONDS; until pgrep -x "$p" >/dev/null; do sleep 1; (( SECONDS - t0 > 30 )) && fail "$p did not start (see $L/sensors.log)"; done; echo "  $p up"
done
t0=$SECONDS
until timeout 4 ros2 topic echo --once --field width "/cam${CAM}/image_raw" >/dev/null 2>&1; do
  sleep 1; (( SECONDS - t0 > 45 )) && fail "cam${CAM} not publishing (see $L/sensors.log)"
done
echo "  cam${CAM} publishing; settling 5 s"; sleep 5
grep -a "polling DID_IMU_RAW" "$L/sensors.log" | tail -1 | cut -c1-200
echo "[vio_live_poll330] starting OpenVINS mono cam${CAM}, raw /imu0, config $CFG"
echo "[vio_live_poll330] INITIALISE: hold the rig still, then move it (with some sideways translation, not just rotation)."
exec "$REPO/run_openvins_mono.sh" --cam "$CAM" --config "$CFG" $RVIZ_ARG
