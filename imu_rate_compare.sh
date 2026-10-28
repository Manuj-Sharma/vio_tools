#!/usr/bin/env bash
# imu_rate_compare.sh [SECONDS=40] -- is the /imu0 shortfall the recorder, the cameras, or the driver?
#
# Needs the rig's sensors ALREADY RUNNING (cameras + trigger + FC + IMX-5). Runs three recordings
# back to back, each SECONDS long, changing only WHAT IS RECORDED, never what is running:
#
#     B  imu_only    ros2 bag record /imu0
#     C  cams_imu    ros2 bag record /cam0/image_raw /cam1/image_raw /imu0   (what preflight used)
#     D  full        record_tool/start_bag.sh = the real 37-topic recorder
#
# Every bag is measured by imu_bag_rate.py, together with the rate the IMU DRIVER logged during
# the same window (sensors.log). The two camera bags (~110 MB/s) are deleted after measuring
# unless KEEP_BAGS=1; the IMU-only bag is always kept.
#
# How to read it (all three have the same cameras running, so any difference is the recording):
#     B ~250, C/D lower      -> recording the cameras costs IMU samples (load / disk)
#     B low as well          -> not the recorder: driver, polling, or system load from the cameras
#     driver Hz low too      -> samples are lost in the SPI polling, before DDS
#     driver ~250, bag low   -> lost after publishing (DDS / recorder)
#
# Full procedure:
#   1. idle baseline (cameras OFF):   ~/vio_tools/record_imu.sh 60 base
#      then: python3 ~/vio_tools/imu_bag_rate.py <that bag> --node-log <that bag>.node.log --label A_idle
#   2. start sensors:                 ~/vio_ar0234_icm20948/record_tool/sensors_and_record.sh start 5
#      (wait for it to finish starting; the 5 s bag it makes is irrelevant)
#   3. this script:                   ~/vio_tools/imu_rate_compare.sh 40
#   4. shut down:                     ~/vio_ar0234_icm20948/record_tool/sensors_and_record.sh stop
set -uo pipefail

SECS="${1:-40}"
REPO="${REPO:-$HOME/vio_ar0234_icm20948}"
LOGDIR="${LOGDIR:-/tmp/vio_logs}"
SENSORS_LOG="${SENSORS_LOG:-$LOGDIR/sensors.log}"
OUTROOT="${OUTROOT:-$HOME/rosbags/imu_compare_$(date +%Y%m%d-%H%M%S)}"
RATE="$HOME/vio_tools/imu_bag_rate.py"
PAUSE="${PAUSE:-5}"

die() { echo "[compare] $*" >&2; exit 1; }

for p in imx5_spi_node vizionsdk_camer; do pgrep -x "$p" >/dev/null || die "$p is not running -- start the sensors first (see header)"; done
# trigger_cam.sh is one-shot (PWM on, sleep 10, set camera mode, exit), so test the PWM itself
PWM=/sys/class/pwm/pwmchip2/pwm0
[ "$(cat $PWM/enable 2>/dev/null)" = 1 ] && [ "$(cat $PWM/period 2>/dev/null)" = 16666666 ] \
  || die "camera trigger PWM is not on at 60 Hz ($PWM) -- cameras would not be triggered"
pgrep -f "[r]os2 bag record" >/dev/null && die "a bag recorder is already running -- stop it first, this test needs exactly one"
[ "$(pgrep -xc imx5_spi_node)" = 1 ] || die "more than one imx5_spi_node is running"
[ -f "$RATE" ] || die "missing $RATE"
[ -f "$SENSORS_LOG" ] || echo "[compare] WARNING: $SENSORS_LOG not found -- driver-side rates will be missing" >&2

# ~0.12 GB/s per camera recording, two of them, plus margin
need=$(( SECS * 24 / 100 + 3 ))
free=$(df -BG --output=avail "$HOME/rosbags" | tail -1 | tr -dc 0-9)
(( free >= need )) || die "need ~${need} GB free, have ${free} GB"

set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
mkdir -p "$OUTROOT"
SUMMARY="$OUTROOT/summary.txt"
echo "[compare] ${SECS}s per condition -> $OUTROOT   (free ${free} GB)"
echo "[compare] cameras up, imx5_spi_node pid $(pgrep -x imx5_spi_node), governor $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)" | tee "$SUMMARY"

measure() {   # <label> <bag dir> <first log line> <last log line> [extra imu_bag_rate.py args]
  python3 "$RATE" "$2" --label "$1" --node-log "$SENSORS_LOG" --lines "$3:$4" ${5:-} | tee -a "$SUMMARY"
}

run() {       # <label> <dirname> <topics...>
  local label="$1" name="$2"; shift 2
  local bag="$OUTROOT/$name" l0 l1
  l0=$(wc -l < "$SENSORS_LOG" 2>/dev/null || echo 0)
  echo "[compare] $label: recording ${SECS}s ..."
  # foreground `timeout -s INT`, as preflight() in sensors_and_record.sh does: a clean SIGINT is what
  # makes rosbag2 write metadata.yaml, and a `&` job in a non-interactive script ignores SIGINT.
  # `< /dev/null` is REQUIRED: timeout moves ros2 into a background process group, and rosbag2 reads
  # the keyboard (pause key) from stdin, so on a real terminal it is stopped by SIGTTIN and hangs.
  timeout -s INT "$SECS" ros2 bag record -o "$bag" "$@" > "$OUTROOT/$name.log" 2>&1 < /dev/null
  l1=$(wc -l < "$SENSORS_LOG" 2>/dev/null || echo 0)
  [ -s "$bag/metadata.yaml" ] || echo "[compare] WARNING: $label has no metadata.yaml -- see $OUTROOT/$name.log" >&2
  # C and D must show both cameras near 60 Hz, else the "cameras are loading the system" premise is void
  local extra=""; [ "$label" != "B_imu_only" ] && extra="--min-cam-hz 50"
  measure "$label" "$bag" "$((l0 + 1))" "$l1" "$extra"
  [ "$label" != "B_imu_only" ] && [ "${KEEP_BAGS:-0}" != 1 ] && { rm -rf "$bag"; echo "      (camera bag deleted; KEEP_BAGS=1 to keep)"; }
  sleep "$PAUSE"
}

run B_imu_only imu_only /imu0
run C_cams_imu cams_imu /cam0/image_raw /cam1/image_raw /imu0

# D: the real recorder script, run directly (not via tmux) so BAG_ROOT is honoured
echo "[compare] D_full: recording ${SECS}s with record_tool/start_bag.sh (37 topics) ..."
l0=$(wc -l < "$SENSORS_LOG" 2>/dev/null || echo 0)
BAG_ROOT="$OUTROOT" LOGDIR="$LOGDIR" "$REPO/record_tool/start_bag.sh" "$SECS" 0 > "$OUTROOT/full.log" 2>&1
l1=$(wc -l < "$SENSORS_LOG" 2>/dev/null || echo 0)
dbag=$(ls -dt "$OUTROOT"/vio_* 2>/dev/null | head -1)
if [ -n "$dbag" ]; then
  measure D_full "$dbag" "$((l0 + 1))" "$l1" "--min-cam-hz 50"
  [ "${KEEP_BAGS:-0}" != 1 ] && { rm -rf "$dbag"; echo "      (camera bag deleted; KEEP_BAGS=1 to keep)"; }
else
  echo "[compare] D_full: no bag found, see $OUTROOT/full.log" >&2
fi

echo
echo "[compare] summary: $SUMMARY"
cat "$SUMMARY"
grep -q "CAMERA LOW" "$SUMMARY" && echo "[compare] WARNING: a camera was below 50 Hz in C or D -- those rows do NOT test camera load; check the camera nodes / trigger" >&2
