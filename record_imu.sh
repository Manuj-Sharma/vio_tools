#!/usr/bin/env bash
# record_imu.sh [SECONDS] [NAME] -- log /imu0 to a ROS2 bag, with fan RPM alongside.
#
#     ./record_imu.sh                    # 300 s, auto-named
#     ./record_imu.sh 300 fantest        # 300 s -> imu_fantest_<stamp>
#     ./record_imu.sh 90                 # quick check
#
# Starts ONLY the IMX-5 IMU node from vio_bringup -- no camera, so the box stays
# as close to idle as it can while recording. No sudo: /dev/spidev* is root:gpio
# and vc is in the gpio group.
#
# Writes two things side by side:
#
#     <out>/                 ROS2 bag, /imu0, sensor_msgs/Imu, ~250 Hz
#     <out>.fan.csv          t_epoch,rpm,pwm,tj_c  once a second
#
# The fan log is the point. To decide whether the 16.669 Hz line in
# imu_static_20260806-185934 is the fan, we need the fan speed DURING the
# capture, which is exactly what we did not have for the original log.
#
# NOTE ON DRIFT: with only the IMU node running the box cools, so the governor
# walks the fan down over five minutes. Check the spread in the .fan.csv summary
# printed at the end -- if RPM moved a lot, a fan-borne line is smeared across a
# band rather than sitting in one bin, and you should judge it against the RPM
# range rather than a single number.
set -uo pipefail

DURATION="${1:-300}"
NAME="${2:-fantest}"
OUTDIR="${OUTDIR:-$HOME/rosbags/field_data}"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$OUTDIR/imu_${NAME}_${STAMP}"
FANLOG="${OUT}.fan.csv"

ROS_SETUP=/opt/ros/humble/setup.bash
IMX5_WS="${IMX5_WS:-$HOME/imx5_ws}"

[ -f "$ROS_SETUP" ]                    || { echo "no $ROS_SETUP" >&2; exit 1; }
[ -f "$IMX5_WS/install/setup.bash" ]   || { echo "imx5_spi_driver not built ($IMX5_WS)" >&2; exit 1; }

RPM_F=/sys/class/hwmon/hwmon2/rpm
PWM_F=/sys/devices/platform/pwm-fan/hwmon/hwmon0/pwm1
TJ_F=/sys/class/thermal/thermal_zone5/temp

PIDS=()
cleanup() {
  echo
  echo "[rec] stopping..."
  for p in "${PIDS[@]:-}"; do kill -INT  "$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]:-}"; do kill -TERM "$p" 2>/dev/null; done
  pkill -x imx5_spi_node 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

pkill -x imx5_spi_node 2>/dev/null && { echo "[rec] killed a stale imx5_spi_node"; sleep 2; }

set +u
source "$ROS_SETUP"
source "$IMX5_WS/install/setup.bash"
set -u

echo "[rec] starting IMX-5 node (use_imu_raw, poll_period_ms 4)"
ros2 run imx5_spi_driver imx5_spi_node --ros-args \
     -p clock_source:=monotonic -p frame_id:=imu_link \
     -p use_imu_raw:=true -p poll_period_ms:=4 \
     > "${OUT}.node.log" 2>&1 &
PIDS+=($!)

echo -n "[rec] waiting for /imu0 "
for i in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -qx /imu0; then echo "-- up (${i}s)"; break; fi
  echo -n "."; sleep 1
  [ "$i" = 30 ] && { echo " TIMEOUT"; echo "see ${OUT}.node.log (rxPkt=0 => check G9/nSPI_EN grounded)" >&2; exit 1; }
done
pgrep -x imx5_spi_node >/dev/null || { echo "[rec] node died, see ${OUT}.node.log" >&2; exit 1; }
sleep 2

echo "t_epoch,rpm,pwm,tj_c" > "$FANLOG"
( while true; do
    printf "%s,%s,%s,%s\n" "$(date +%s.%N)" \
      "$(cat $RPM_F 2>/dev/null || echo NA)" \
      "$(cat $PWM_F 2>/dev/null || echo NA)" \
      "$(awk '{printf "%.1f", $1/1000}' $TJ_F 2>/dev/null || echo NA)" >> "$FANLOG"
    sleep 1
  done ) &
PIDS+=($!)

echo "[rec] fan now: $(cat $RPM_F) rpm  = $(awk -v r="$(cat $RPM_F)" 'BEGIN{printf "%.2f", r/60}') Hz   Tj $(awk '{printf "%.1f", $1/1000}' $TJ_F) C"
echo "[rec] recording ${DURATION}s -> $OUT"
ros2 bag record /imu0 -o "$OUT" > "${OUT}.bag.log" 2>&1 &
PIDS+=($!)

for s in $(seq "$DURATION" -10 1); do
  printf "\r[rec] %4ds left   fan %s rpm  Tj %s C   " \
    "$s" "$(cat $RPM_F)" "$(awk '{printf "%.1f", $1/1000}' $TJ_F)"
  sleep 10
done
echo

cleanup
trap - EXIT INT TERM

echo
echo "[rec] fan over the capture:"
awk -F, 'NR>1 && $2!="NA" {n++; s+=$2; if(min==""||$2<min)min=$2; if($2>max)max=$2}
         END {if(n) printf "   rpm  mean %.0f  min %.0f  max %.0f  spread %.0f\n"\
                            "   Hz   mean %.2f  min %.2f  max %.2f\n",
                            s/n, min, max, max-min, s/n/60, min/60, max/60}' "$FANLOG"
echo
echo "[rec] bag: $OUT"
echo "[rec] fan: $FANLOG"
echo
echo "next:"
echo "   python3 imu_gap_compare.py $OUT --zoom 5 45 -o ~/fantest.png && eog ~/fantest.png"
echo
echo "   line at ~fan Hz  -> it is the fan"
echo "   line at 16.669   -> not the fan; bench or building"
echo "   no line          -> whatever it was yesterday is not running now"
