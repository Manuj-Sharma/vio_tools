#!/usr/bin/env bash
#
# sensors_and_record.sh - start ONLY the sensors (dual camera + IMU + camera trigger)
# and the bag recorder (record_tool/record.sh). No OpenVINS, no RViz.
# Everything is nohup/setsid'd, so closing SSH / the terminal does NOT stop it.
#
# Usage:
#   ~/vio_tools/sensors_and_record.sh start [seconds=180]   # 0 = record until stopped
#   ~/vio_tools/sensors_and_record.sh stop                  # stop bag (clean flush), then sensors
#
# Logs: $LOGDIR/{sensors,trigger,sensors_and_record}.log   (LOGDIR default /tmp/vio_logs)
# Startup sequence copied from run_vio_stack.sh step 1 -- keep the two in sync.

set -u
REPO="${REPO:-$HOME/vio_ar0234_icm20948}"
LOGDIR="${LOGDIR:-/tmp/vio_logs}"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-25}"
CAM_NORMAL_SETTLE="${CAM_NORMAL_SETTLE:-2}"
CAM_TRIGGER_SETTLE="${CAM_TRIGGER_SETTLE:-2}"
mkdir -p "$LOGDIR"

fail() { echo "[sensors_and_record] ERROR: $*" >&2; exit 1; }

wait_for_process() {   # <name> <timeout_s>
  local t0=$SECONDS
  while (( SECONDS - t0 < $2 )); do pgrep -x -- "$1" >/dev/null && { echo "  $1 up ($((SECONDS - t0))s)"; return 0; }; sleep 1; done
  echo "  WARNING: $1 not up within $2 s" >&2; return 1
}

do_start() {
  local dur="${1:-180}"
  pgrep -x imx5_spi_node >/dev/null && fail "sensors already running (stop first)"
  set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u

  echo "[sensors_and_record] cameras -> normal mode"
  vizion-ctl -d 0 -s trigger_mode=0
  vizion-ctl -d 1 -s trigger_mode=0
  sleep "$CAM_NORMAL_SETTLE"
  ros2 daemon stop >/dev/null 2>&1 || true

  setsid nohup ros2 launch "$REPO/vio_bringup/vio_bringup.launch.py" > "$LOGDIR/sensors.log" 2>&1 < /dev/null &
  wait_for_process imx5_spi_node   "$STARTUP_TIMEOUT" || fail "IMU never came up - see $LOGDIR/sensors.log"
  wait_for_process vizionsdk_camer "$STARTUP_TIMEOUT" || fail "camera never came up - see $LOGDIR/sensors.log"

  echo "[sensors_and_record] camera0 -> trigger mode, starting trigger loop"
  vizion-ctl -d 0 -s trigger_mode=2
  sleep "$CAM_TRIGGER_SETTLE"
  setsid nohup "$REPO/trigger_cam.sh" > "$LOGDIR/trigger.log" 2>&1 < /dev/null &

  echo "[sensors_and_record] starting bag recorder (record_tool/record.sh $dur)"
  "$REPO/record_tool/record.sh" "$dur"
  echo "[sensors_and_record] running detached. Stop with: $0 stop"
}

do_stop() {
  "$REPO/record_tool/stop_bag.sh"
  pkill -f "trigger_cam.sh"            2>/dev/null
  pkill vizionsdk_camer                2>/dev/null
  pkill imx5_spi_node                  2>/dev/null
  pkill -f "flight_dataset_recorder_publisher.py" 2>/dev/null
  echo "[sensors_and_record] stopped"
}

case "${1:-}" in
  start) shift
         # re-exec fully detached so SSH/terminal close can't kill the startup sequence either
         if [ -z "${SAR_DETACHED:-}" ]; then
           SAR_DETACHED=1 setsid nohup "$0" start "$@" > "$LOGDIR/sensors_and_record.log" 2>&1 < /dev/null &
           echo "[sensors_and_record] launched detached; progress: tail -f $LOGDIR/sensors_and_record.log"
         else do_start "$@"; fi ;;
  stop)  do_stop ;;
  *)     sed -n '2,13p' "$0"; exit 1 ;;
esac
