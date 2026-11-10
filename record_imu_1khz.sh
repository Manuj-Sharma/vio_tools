#!/usr/bin/env bash
# Record the raw 1 kHz IMX-5 /imu0 stream (DID_IMU_RAW, stream mode) to a ros2 bag, for the offline filter sweep.
#   record_imu_1khz.sh [NAME] [MAX_SECONDS=120]       env: REC_CAMS=1 also records both camera topics (~83 MB/s!)
# Output: ~/rosbags/imu1k_<NAME>_<stamp>/   (bag with /imu0 only by default)
#
# Sensors: same setup as vio_live_with_filter.sh -- IMU stream 128 B frames / 1000 us loop / 10 MHz SPI (default; IMU_SPI_HZ=3000000 overrides), cameras free-running
# (CAM_TRIGGER_MODE=0, camera PWM must be off). Cameras stay ON (not recorded) so the SPI/CPU load matches the real VIO run.
# If imx5_spi_node is already running it is reused and NOT stopped at the end; otherwise this script starts and stops it.
# Stop recording with ENTER (or Ctrl+C); it also stops by itself after MAX_SECONDS.
# Run it in a real terminal (hardware processes do not survive the agent sandbox).
set -u
NAME="${1:-lift}"; MAXS="${2:-120}"
REPO="$HOME/vio_ar0234_icm20948"; TOOLS="$HOME/vio_tools"; L=/tmp/vio_logs; mkdir -p "$L" "$HOME/rosbags"
OUT="$HOME/rosbags/imu1k_${NAME}_$(date +%Y%m%d_%H%M%S)"
fail() { echo "[rec1k] ERROR: $*" >&2; exit 1; }
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
[ -e "$OUT" ] && fail "$OUT exists"
free=$(df -BG --output=avail "$HOME/rosbags" | tail -1 | tr -dc 0-9); (( free < 4 )) && fail "only ${free} GB free"

STARTED=0
if pgrep -x imx5_spi_node >/dev/null; then
  echo "[rec1k] imx5_spi_node already running -- reusing it. NOTE: it must be in stream mode (IMU_STREAM=1) for 1 kHz; checking the rate below."
else
  [ "$(cat /sys/class/pwm/pwmchip2/pwm0/enable 2>/dev/null)" = 1 ] && fail "camera trigger PWM is ON; free-running cameras are needed"
  export CAM_TRIGGER_MODE=0 IMU_STREAM=1 IMU_FRAME_LEN="${IMU_FRAME_LEN:-128}" IMU_POLL_US="${IMU_POLL_US:-1000}" IMU_SPI_HZ="${IMU_SPI_HZ:-10000000}"
  ros2 daemon stop >/dev/null 2>&1 || true
  echo "[rec1k] starting sensors (IMU stream ${IMU_FRAME_LEN} B / ${IMU_POLL_US} us / ${IMU_SPI_HZ} Hz SPI, cameras free-running)"
  setsid nohup ros2 launch "$REPO/vio_bringup/vio_bringup.launch.py" > "$L/sensors_rec1k.log" 2>&1 < /dev/null &
  STARTED=1
  for p in imx5_spi_node vizionsdk_camer; do
    t0=$SECONDS; until pgrep -x "$p" >/dev/null; do sleep 1; (( SECONDS - t0 > 30 )) && fail "$p did not start (see $L/sensors_rec1k.log)"; done; echo "  $p up"
  done
fi

stop_sensors() {
  (( STARTED )) || return 0
  echo "[rec1k] stopping sensors"
  pkill -INT -f "[v]io_bringup.launch.py"; sleep 2
  pkill -x vizionsdk_camer; pkill -x imx5_spi_node; sleep 1
}
trap 'stop_sensors' EXIT

echo "[rec1k] waiting for /imu0 at ~1 kHz"
t0=$SECONDS; ok=0
while (( SECONDS - t0 < 40 )); do
  r=$(timeout 6 ros2 topic hz /imu0 --window 300 2>/dev/null | awk '/average rate/ {r=$3} END {print r}')
  [ -n "$r" ] && echo "  /imu0 ${r} Hz" && awk -v r="$r" 'BEGIN{exit !(r>800)}' && { ok=1; break; }
done
(( ok )) || fail "/imu0 never reached ~1 kHz (not in stream mode? see $L/sensors_rec1k.log)"
echo "[rec1k] settling 8 s (cameras warm up)"; sleep 8

TOPICS=(/imu0); [ "${REC_CAMS:-0}" = 1 ] && TOPICS+=(/cam0/image_raw /cam1/image_raw)
ros2 bag record -o "$OUT" "${TOPICS[@]}" > "$OUT.rec.log" 2>&1 &
REC=$!
sleep 2; kill -0 $REC 2>/dev/null || fail "bag record failed (see $OUT.rec.log)"
echo
echo "================  RECORDING  ->  $OUT"
echo "  1) hold the rig STILL for at least 5 s (noise baseline)"
echo "  2) lift it sharply, hold still 3 s, put it down; repeat a few times; add a few table taps / hand shakes"
echo "  3) press ENTER to stop (auto-stop after ${MAXS} s)"
echo "================"
T=$SECONDS
read -r -t "$MAXS" _ || true
echo "[rec1k] stopping recorder after $((SECONDS - T)) s"
kill -INT $REC 2>/dev/null; wait $REC 2>/dev/null

echo; echo "[rec1k] result:"
python3 "$TOOLS/bag_imu_stamps.py" "$OUT" | sed 's/^/   /'
du -sh "$OUT" | sed 's/^/   size: /'
echo "[rec1k] bag: $OUT"
echo "[rec1k] next: python3 $TOOLS/imu_filter_sweep.py $OUT --csv $OUT/sweep.csv"
