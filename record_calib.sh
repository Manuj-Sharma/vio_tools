#!/usr/bin/env bash
# record_calib.sh <cam0|cam1|stereo> <intrinsics|imu> [seconds]
# One camera at a time, two bags per camera (4 recordings), plus optional stereo bags:
#
#   cam0 intrinsics  [120]  records /cam0/image_raw only.        Rig STILL, move the AprilGrid slowly.
#   cam0 imu         [90]   records /cam0/image_raw + /imu0.     Board STILL, move the rig on all 6 axes.
#   cam1 intrinsics  [120]  records /cam1/image_raw only.
#   cam1 imu         [90]   records /cam1/image_raw + /imu0.
#   stereo intrinsics [120] records /cam0/image_raw + /cam1/image_raw. Board must be seen by BOTH cameras at once
#                           (needs overlap) -> kalibr_calibrate_cameras --topics /cam0/... /cam1/... gives T_cn_cnm1.
#   stereo imu       [90]   records both cameras + /imu0 -> joint two-camera cam-IMU run.
#
# Flow: serial check -> sensors start (same commands as sensors_and_record.sh do_start, which is NOT
# modified/used for starting) -> YOU press Enter -> its own `ros2 bag record` (timed, calibration topics only;
# nothing in record_tool/ is modified) + record_health.py report -> bag moved to
# ~/rosbags/calib_data/<cam>_<kind>_YYYYMMDD[_n] with notes.txt -> sensors stopped.
# A failed recording is renamed ..._FAILED. Run with the stack STOPPED, in your own terminal.
set -u
CAM="${1:-}"; KIND="${2:-}"
case "$CAM" in cam0|cam1|stereo) ;; *) sed -n '2,16p' "$0"; exit 1 ;; esac
IMGS="/$CAM/image_raw"; [ "$CAM" = stereo ] && IMGS="/cam0/image_raw /cam1/image_raw"
case "$KIND" in intrinsics) DEF=120; export BAG_TOPICS="$IMGS" ;;
                imu)        DEF=90;  export BAG_TOPICS="$IMGS /imu0" ;;
                *) sed -n '2,16p' "$0"; exit 1 ;; esac
SECS="${3:-$DEF}"
REPO="${REPO:-$HOME/vio_ar0234_icm20948}"
TOOLS="$HOME/vio_tools"
SAR="$REPO/record_tool/sensors_and_record.sh"      # only used for its `stop`
LOGDIR="${LOGDIR:-/tmp/vio_logs}"; mkdir -p "$LOGDIR"
STATUS_FILE="$LOGDIR/record_status.txt"; REC_RESULT="$LOGDIR/recorder.result"; LAST_REPORT="$LOGDIR/last_recording_report.txt"
export IMU_POLL_MS="${IMU_POLL_MS:-3}"             # read by vio_bringup.launch.py (3 ms ~ 300 Hz, the field default)

abort() { echo "ABORT: $*" >&2; "$SAR" stop >/dev/null 2>&1; exit 1; }
wait_for_process() { local t0=$SECONDS; while (( SECONDS - t0 < $2 )); do pgrep -x -- "$1" >/dev/null && return 0; sleep 1; done; return 1; }

echo "== preflight: which physical camera is cam0 / cam1 =="
"$TOOLS/cam_order_check.sh" || { echo "ABORT: camera order check failed"; exit 1; }

echo
echo "== $CAM $KIND: recording $BAG_TOPICS for ${SECS}s =="
if [ "$KIND" = intrinsics ]; then cat <<'EOF'
Checklist (intrinsics): rig fixed, move the board slowly (no blur). 100+ different poses.
  - cover every corner and edge of THIS camera's image (stereo: of BOTH images, while both see the whole board)
  - tilt +-30..45 deg, distance 0.5..1.5 m, keep the whole board in frame
EOF
else cat <<'EOF'
Checklist (cam-IMU): board fixed and always visible to THIS camera (stereo: to both cameras). Keep still ~5 s, then
rotate and translate on all 6 axes (roll/pitch/yaw + x/y/z), moderate speed, smooth, no blur;
keep still ~5 s at the end.
EOF
fi
echo

# ---- sensors (sequence copied from sensors_and_record.sh do_start; keep in sync) ----
pgrep -x imx5_spi_node >/dev/null && { echo "ABORT: sensors already running - stop them first ($SAR stop)"; exit 1; }
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
echo "Starting sensors (~30-40 s)..."
vizion-ctl -d 0 -s trigger_mode=0 >/dev/null; vizion-ctl -d 1 -s trigger_mode=0 >/dev/null
sleep 2
ros2 daemon stop >/dev/null 2>&1 || true
setsid nohup ros2 launch "$REPO/vio_bringup/vio_bringup.launch.py" > "$LOGDIR/sensors.log" 2>&1 < /dev/null &
wait_for_process imx5_spi_node   25 || abort "IMU never came up - see $LOGDIR/sensors.log"
wait_for_process vizionsdk_camer 25 || abort "camera never came up - see $LOGDIR/sensors.log"
vizion-ctl -d 0 -s trigger_mode=2 >/dev/null
sleep 2
(cd "$REPO" && setsid nohup ./trigger_cam.sh > "$LOGDIR/trigger.log" 2>&1 < /dev/null &)
t0=$SECONDS; cams_ok=0
while (( SECONDS - t0 < 40 )); do
  if timeout 4 ros2 topic echo --once --field width /cam0/image_raw >/dev/null 2>&1 &&
     timeout 4 ros2 topic echo --once --field width /cam1/image_raw >/dev/null 2>&1; then cams_ok=1; break; fi
  sleep 1
done
(( cams_ok )) || abort "cameras not publishing after 40 s - see $LOGDIR/sensors.log"
pgrep -f trigger_cam.sh >/dev/null || abort "trigger loop is not running"
sleep 5   # warm-up settle
echo "Sensors up: cameras publishing, trigger running."

echo
echo "Position the board; check the view if you like (ros2 run rqt_image_view rqt_image_view /${CAM/stereo/cam0}/image_raw)."
read -r -p "Press Enter to START the ${SECS}s recording: " _

# ---- recording ----
rm -f "$REC_RESULT" "$STATUS_FILE"
# own timed recorder (record_tool/ is not modified): same command shape as start_bag.sh, topics = $BAG_TOPICS.
# `trap - INT QUIT` is required: a backgrounded non-interactive subshell would otherwise ignore the SIGINT
# that `timeout` sends, and the bag would never be finalised.
BAG="$HOME/rosbags/vio_$(date +%Y%m%d_%H%M%S)"
( trap - INT QUIT
  timeout --preserve-status -s INT "$SECS" ros2 bag record -o "$BAG" --max-cache-size 1000000000 $BAG_TOPICS \
      > "$LOGDIR/recorder.log" 2>&1
  rc=$?
  if [ "$rc" -eq 0 ] && [ -s "$BAG/metadata.yaml" ]; then echo "OK rc=$rc bag=$BAG $(date +%H:%M:%S)" > "$REC_RESULT"
  else echo "FAILED rc=$rc metadata=$([ -s "$BAG/metadata.yaml" ] && echo present || echo MISSING) bag=$BAG $(date +%H:%M:%S)" > "$REC_RESULT"; fi
) &
TTY_ARG=(); T="$(tty 2>/dev/null | grep '^/dev/' || true)"; [ -n "$T" ] && TTY_ARG=(--notify-tty "$T")
setsid nohup python3 "$REPO/record_tool/record_health.py" --interval 5 --status "$STATUS_FILE" --result-file "$REC_RESULT" \
    --expect-sec "$SECS" --report-copy "$LAST_REPORT" --logdir "$LOGDIR" "${TTY_ARG[@]}" \
    > "$LOGDIR/record_health.log" 2>&1 < /dev/null &
t0=$SECONDS
until pgrep -f "[r]os2 bag record" >/dev/null; do
  (( SECONDS - t0 > 25 )) && abort "the bag recorder did not start - see $LOGDIR/recorder.log"
  sleep 1
done
echo "Recording... (${SECS}s)"
t1=$SECONDS
while [ ! -s "$REC_RESULT" ]; do
  (( SECONDS - t1 > SECS + 120 )) && abort "recorder still running long after ${SECS}s"
  sleep 2
done
echo "Recording done, waiting for the health report..."
for _ in $(seq 40); do pgrep -f "[r]ecord_health.py" >/dev/null || break; sleep 1; done
pkill -f "[r]ecord_health.py" 2>/dev/null

[ -d "$BAG" ] || abort "no bag directory $BAG"
OUT="$HOME/rosbags/calib_data"; mkdir -p "$OUT"
D="$(date +%Y%m%d)"; DEST="$OUT/${CAM}_${KIND}_$D"; n=1
while [ -e "$DEST" ]; do n=$((n+1)); DEST="$OUT/${CAM}_${KIND}_${D}_$n"; done
if grep -q '^FAILED' "$REC_RESULT" || [ ! -s "$BAG/metadata.yaml" ] || head -n 1 "$BAG/recording_report.txt" 2>/dev/null | grep -q FAILED; then
  DEST="${DEST}_FAILED"; echo "*** RECORDING FAILED - the bag is NOT usable. Report: $BAG/recording_report.txt ***"
fi
mv "$BAG" "$DEST"

{
  echo "camera: $CAM   kind: $KIND   topics: $BAG_TOPICS   seconds: $SECS   recorded: $(date -Iseconds)"
  echo "cam0/cam1 serials (SDK index 0/1):"; tail -2 "$TOOLS/records/cam_order_log.csv" | awk -F, '{print "  cam" $4 "  " $5 "  " $6}'
  echo "settings: IMU_POLL_MS=$IMU_POLL_MS TRIGGER_HZ=${TRIGGER_HZ:-60(default)} exposure cap 1-8 ms (launch file)"
  echo "target: $HOME/rosbags/april_6x6_80x80cm.yaml"
  echo "git: repo $(git -C "$REPO" rev-parse --short HEAD 2>/dev/null)  vio_tools $(git -C "$TOOLS" rev-parse --short HEAD 2>/dev/null)"
} > "$DEST/notes.txt"
echo; echo "Bag: $DEST"; cat "$DEST/notes.txt"; echo
head -n 12 "$DEST/recording_report.txt" 2>/dev/null

"$SAR" stop >/dev/null 2>&1
echo; echo "Sensors stopped."
