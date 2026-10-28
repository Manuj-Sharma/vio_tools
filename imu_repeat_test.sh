#!/usr/bin/env bash
# imu_repeat_test.sh [N=6] [SECS=100] [SPLIT=60] -- does /imu0 get worse the more we record?
#
# Needs the rig's sensors ALREADY RUNNING (see imu_rate_compare.sh header for how to start them).
# Records N bags back to back with the real 37-topic recorder (record_tool/start_bag.sh), measures
# each one (imu_bag_rate.py: IMU Hz, % missing, camera Hz, the driver's own Hz) and, in the same
# window, samples the Jetson once per 2 s: hottest thermal zone, lowest CPU clock, VDD_IN voltage
# and current, and over-current throttle events. One summary row per run, so a trend (heat soak,
# something accumulating, a slow leak) shows up as rows getting worse down the table.
#
# Each bag (~11 GB for 100 s) is deleted after it is measured unless KEEP_BAGS=1. The table is
# kept in $OUTROOT/summary.txt and the raw 2 s samples in $OUTROOT/telemetry_run<i>.csv.
#
#     ~/vio_tools/imu_repeat_test.sh 6 100 60      # ~13 minutes, one bag on disk at a time
set -uo pipefail

N="${1:-6}"; SECS="${2:-100}"; SPLIT="${3:-60}"
REPO="${REPO:-$HOME/vio_ar0234_icm20948}"
LOGDIR="${LOGDIR:-/tmp/vio_logs}"
SENSORS_LOG="${SENSORS_LOG:-$LOGDIR/sensors.log}"
OUTROOT="${OUTROOT:-$HOME/rosbags/imu_repeat_$(date +%Y%m%d-%H%M%S)}"
RATE="$HOME/vio_tools/imu_bag_rate.py"
PAUSE="${PAUSE:-10}"

die() { echo "[repeat] $*" >&2; exit 1; }

for p in imx5_spi_node vizionsdk_camer; do pgrep -x "$p" >/dev/null || die "$p is not running -- start the sensors first"; done
PWM=/sys/class/pwm/pwmchip2/pwm0
[ "$(cat $PWM/enable 2>/dev/null)" = 1 ] && [ "$(cat $PWM/period 2>/dev/null)" = 16666666 ] \
  || die "camera trigger PWM is not on at 60 Hz ($PWM)"
pgrep -f "[r]os2 bag record" >/dev/null && die "a bag recorder is already running"
[ "$(pgrep -xc imx5_spi_node)" = 1 ] || die "more than one imx5_spi_node is running"
[ -f "$RATE" ] && [ -f "$SENSORS_LOG" ] || die "missing $RATE or $SENSORS_LOG"

need=$(( SECS * 13 / 100 + 3 ))      # ~0.11 GB/s of camera data plus margin; one bag at a time
free=$(df -BG --output=avail "$HOME/rosbags" | tail -1 | tr -dc 0-9)
(( free >= need )) || die "need ~${need} GB free, have ${free} GB"

HW=""; for h in /sys/class/hwmon/hwmon*; do [ "$(cat $h/name 2>/dev/null)" = ina3221 ] && HW=$h; done
OCHW=""; for h in /sys/class/hwmon/hwmon*; do ls $h/oc1_event_cnt >/dev/null 2>&1 && OCHW=$h; done

oc_total() {   # sum of the over-current event counters (timeout-guarded: some sysfs files block)
  local s=0 f v
  [ -n "$OCHW" ] || { echo 0; return; }
  for f in "$OCHW"/oc*_event_cnt; do v=$(timeout 1 cat "$f" 2>/dev/null || echo 0); s=$((s + v)); done
  echo "$s"
}

sample() {     # CSV: epoch,tj_max_c,cpu_min_mhz,vdd_mv,vdd_ma,oc_total
  local tj=0 t z f fmin=99999999 mv=0 ma=0
  for z in /sys/class/thermal/thermal_zone*/temp; do t=$(cat "$z" 2>/dev/null || echo 0); (( t > tj )) && tj=$t; done
  for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq; do t=$(cat "$f" 2>/dev/null || echo 99999999); (( t < fmin )) && fmin=$t; done
  [ -n "$HW" ] && { mv=$(cat $HW/in1_input 2>/dev/null || echo 0); ma=$(cat $HW/curr1_input 2>/dev/null || echo 0); }
  echo "$(date +%s),$((tj / 1000)),$((fmin / 1000)),$mv,$ma,$(oc_total)"
}

set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
mkdir -p "$OUTROOT"; SUMMARY="$OUTROOT/summary.txt"
echo "[repeat] $N runs x ${SECS}s (split ${SPLIT}s) -> $OUTROOT   free ${free} GB" | tee "$SUMMARY"
echo "[repeat] sensors up since: $(ps -o lstart= -p "$(pgrep -x imx5_spi_node)")   ina3221=${HW:-none}  oc=${OCHW:-none}" | tee -a "$SUMMARY"

for i in $(seq 1 "$N"); do
  run="$OUTROOT/run$i"; tel="$OUTROOT/telemetry_run$i.csv"
  echo "epoch,tj_max_c,cpu_min_mhz,vdd_in_mv,vdd_in_ma,oc_events" > "$tel"
  l0=$(wc -l < "$SENSORS_LOG"); oc0=$(oc_total)
  ( while true; do sample >> "$tel"; sleep 2; done ) &
  SP=$!
  echo "[repeat] run $i/$N at $(date +%H:%M:%S) ..."
  BAG_ROOT="$run" LOGDIR="$LOGDIR" "$REPO/record_tool/start_bag.sh" "$SECS" "$SPLIT" > "$OUTROOT/run$i.log" 2>&1
  rc=$?
  kill "$SP" 2>/dev/null; wait "$SP" 2>/dev/null
  l1=$(wc -l < "$SENSORS_LOG"); oc1=$(oc_total)
  bag=$(ls -d "$run"/vio_* 2>/dev/null | head -1)
  if [ -n "$bag" ]; then
    row=$(python3 "$RATE" "$bag" --label "R$i@$(date +%H:%M)" --min-cam-hz 50 --node-log "$SENSORS_LOG" --lines "$((l0 + 1)):$l1")
  else
    row="R$i: NO BAG (recorder rc=$rc, see $OUTROOT/run$i.log)"
  fi
  stats=$(awk -F, 'NR>1 {if($2>tj)tj=$2; if(fm==""||$3<fm)fm=$3; if(mn==""||$4<mn)mn=$4; if($5>mx)mx=$5; n++}
                   END {printf "Tj max %dC  CPU min %d MHz  VDD_IN min %d mV max %d mA", tj, fm, mn, mx}' "$tel")
  echo "$row  | $stats  oc+$((oc1 - oc0))" | tee -a "$SUMMARY"
  [ "${KEEP_BAGS:-0}" != 1 ] && rm -rf "$run"
  (( i < N )) && sleep "$PAUSE"
done

echo
echo "[repeat] done. $SUMMARY"
