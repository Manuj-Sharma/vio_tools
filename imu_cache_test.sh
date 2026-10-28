#!/usr/bin/env bash
# imu_cache_test.sh [PAIRS=2] [SECS=180] [SPLIT=60] [CACHE_BYTES=1000000000]
#
# Does a bigger rosbag2 write cache stop the "Cache buffers lost messages" drops?
# Runs PAIRS x (default-cache recording, big-cache recording), alternating, so slow drifts in the
# machine (a copy running, page cache state) hit both variants alike. The recorder is the real
# 37-topic one (record_tool/start_bag.sh); the big-cache variant is a temporary copy of it with ONE
# extra flag, --max-cache-size, inserted -- start_bag.sh itself is never modified.
#
# RAM: rosbag2 double-buffers, so it can need 2x CACHE_BYTES. This Jetson has ~7 GB total, hence 1 GB.
#
# Per run it prints: IMU/camera rates, the driver's own rate, messages the recorder reports lost,
# and the average disk READ rate during the run (a background copy shows up there).
# Every bag is deleted right after it is measured; only the table and logs are kept.
set -uo pipefail

PAIRS="${1:-2}"; SECS="${2:-180}"; SPLIT="${3:-60}"; CACHE="${4:-1000000000}"
REPO="${REPO:-$HOME/vio_ar0234_icm20948}"
SENSORS_LOG="${SENSORS_LOG:-/tmp/vio_logs/sensors.log}"
OUTROOT="${OUTROOT:-$HOME/rosbags/imu_cache_$(date +%Y%m%d-%H%M%S)}"
RATE="$HOME/vio_tools/imu_bag_rate.py"
PAUSE="${PAUSE:-10}"
die() { echo "[cache] $*" >&2; exit 1; }

for p in imx5_spi_node vizionsdk_camer; do pgrep -x "$p" >/dev/null || die "$p is not running -- start the sensors first"; done
PWM=/sys/class/pwm/pwmchip2/pwm0
TRIGGER_HZ="${TRIGGER_HZ:-60}"                      # camera trigger rate the rig is set to (trigger.sh HZ)
MIN_CAM_HZ="${MIN_CAM_HZ:-$(( TRIGGER_HZ * 5 / 6 ))}"  # flag a camera below this (50 at 60 Hz, 33 at 40 Hz)
[ "$(cat $PWM/enable 2>/dev/null)" = 1 ] && [ "$(cat $PWM/period 2>/dev/null)" = $(( 1000000000 / TRIGGER_HZ )) ] \
  || die "camera trigger PWM is not on at ${TRIGGER_HZ} Hz (set it with: ~/vio_ar0234_icm20948/trigger.sh ${TRIGGER_HZ})"
pgrep -f "[r]os2 bag record" >/dev/null && die "a bag recorder is already running"
[ "$(pgrep -xc imx5_spi_node)" = 1 ] || die "more than one imx5_spi_node is running"
[ -f "$RATE" ] && [ -f "$SENSORS_LOG" ] || die "missing $RATE or $SENSORS_LOG"
need=$(( SECS * 13 / 100 + 3 )); free=$(df -BG --output=avail "$HOME/rosbags" | tail -1 | tr -dc 0-9)
(( free >= need )) || die "need ~${need} GB free, have ${free} GB"
avail_mb=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
(( avail_mb >= 2 * CACHE / 1000000 + 1500 )) || die "only ${avail_mb} MB RAM available; 2x cache + 1.5 GB needed"

mkdir -p "$OUTROOT/log"; SUMMARY="$OUTROOT/summary.txt"
python3 -I - "$REPO/record_tool/start_bag.sh" "$OUTROOT/start_bag_bigcache.sh" "$CACHE" <<'EOF' || die "could not build the big-cache variant"
import sys
src,dst,n=sys.argv[1],sys.argv[2],int(sys.argv[3])
s=open(src).read(); key='    -o "$BAG_ROOT/$BAG_NAME" \\\n'
assert s.count(key)==1, "injection point not found exactly once"
open(dst,'w').write(s.replace(key,key+f'    --max-cache-size {n} \\\n',1))
EOF
chmod +x "$OUTROOT/start_bag_bigcache.sh"
grep -q -- "--max-cache-size $CACHE" "$OUTROOT/start_bag_bigcache.sh" || die "variant missing the flag"

set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
rd() { awk '$3=="nvme0n1" {print $6}' /proc/diskstats; }
echo "[cache] $PAIRS pairs x ${SECS}s split ${SPLIT}s, big cache $((CACHE / 1000000)) MB -> $OUTROOT  (free ${free} GB, RAM avail ${avail_mb} MB)" | tee "$SUMMARY"

n=0
for p in $(seq 1 "$PAIRS"); do
  for variant in default big; do
    n=$((n + 1)); run="$OUTROOT/run${n}_$variant"
    script="$REPO/record_tool/start_bag.sh"; [ "$variant" = big ] && script="$OUTROOT/start_bag_bigcache.sh"
    l0=$(wc -l < "$SENSORS_LOG"); r0=$(rd); t0=$(date +%s)
    echo "[cache] run $n ($variant) at $(date +%H:%M:%S) ..."
    BAG_ROOT="$run" LOGDIR="$OUTROOT/log" "$script" "$SECS" "$SPLIT" > "$OUTROOT/run${n}_$variant.log" 2>&1
    t1=$(date +%s); r1=$(rd); l1=$(wc -l < "$SENSORS_LOG")
    bag=$(ls -d "$run"/vio_* 2>/dev/null | head -1)
    if [ -n "$bag" ]; then
      row=$(python3 "$RATE" "$bag" --label "R${n} ${variant}" --min-cam-hz "$MIN_CAM_HZ" --node-log "$SENSORS_LOG" --lines "$((l0 + 1)):$l1")
    else
      row="R${n} ${variant}: NO BAG (see $OUTROOT/run${n}_$variant.log)"
    fi
    lost=$(grep 'Total lost' "$OUTROOT/run${n}_$variant.log" | tail -1 | grep -o '[0-9]*$'); lost=${lost:-0}
    echo "$row  | lost ${lost}  disk-read $(( (r1 - r0) * 512 / (t1 - t0) / 1000000 )) MB/s  sftp=$(pgrep -c sftp-server)" | tee -a "$SUMMARY"
    rm -rf "$run"
    sleep "$PAUSE"
  done
done
echo; echo "[cache] done. $SUMMARY"
