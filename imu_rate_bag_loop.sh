#!/usr/bin/env bash
# Record N bags of DUR seconds (1 kHz IMU stream at 3 MHz + both cameras), report the /imu0 rate of each, DELETE each bag.
#   imu_rate_bag_loop.sh [N=4] [DUR=30]          (env IMU_* can be overridden; defaults = stream mode 1 kHz, 128 B, 3 MHz)
# Bag 1 is recorded by sensors_and_record_notrigger.sh (which also starts the sensors); bags 2..N by record.sh with the sensors
# still running. Only bags created by THIS script are deleted. Sensors are stopped at the end.
set -u
N="${1:-4}"; DUR="${2:-30}"
REPO="$HOME/vio_ar0234_icm20948"; BAGS="$HOME/rosbags"; TOOLS="$HOME/vio_tools"
set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
pgrep -x imx5_spi_node >/dev/null && { echo "sensors already running -- stop them first"; exit 1; }
export IMU_STREAM="${IMU_STREAM:-1}" IMU_FRAME_LEN="${IMU_FRAME_LEN:-128}" IMU_POLL_US="${IMU_POLL_US:-1000}" IMU_SPI_HZ="${IMU_SPI_HZ:-3000000}" EXP_IMU_HZ=900 FOLLOW=0
T0=$(date +%s)

analyze() {   # <bag>
  local bag="$1"
  python3 - "$bag" <<'PY'
import sqlite3, glob, sys
bag = sys.argv[1]
db = glob.glob(bag + "/*.db3")[0]
c = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
names = dict(c.execute("select id,name from topics"))
for tid, n in sorted(names.items(), key=lambda kv: kv[1]):
    if n not in ("/imu0", "/cam0/image_raw", "/cam1/image_raw"): continue
    cnt, a, b = c.execute("select count(*),min(timestamp),max(timestamp) from messages where topic_id=?", (tid,)).fetchone()
    if cnt > 1: print(f"   {n:18s} {cnt:6d} msgs  {cnt/((b-a)*1e-9):7.1f} Hz (recv-time span {(b-a)*1e-9:5.1f} s)")
PY
  python3 "$TOOLS/bag_imu_stamps.py" "$bag" | sed 's/^/   /'
}

wait_recording_done() {
  local t=0
  while tmux has-session -t rosbag 2>/dev/null; do sleep 1; t=$((t+1)); (( t > DUR + 90 )) && { echo "recorder did not finish"; return 1; }; done
  sleep 3; return 0
}

for i in $(seq "$N"); do
  free=$(df -BG --output=avail "$BAGS" | tail -1 | tr -dc 0-9)
  (( free < 6 )) && { echo "ABORT: only ${free} GB free"; break; }
  echo; echo "=== bag $i/$N  ($(date +%H:%M:%S), ${free} GB free) ==="
  if (( i == 1 )); then
    "$REPO/record_tool/sensors_and_record_notrigger.sh" start "$DUR" > /tmp/vio_logs/loop_start.log 2>&1
    for _ in $(seq 150); do grep -q STARTUP-ENDED /tmp/vio_logs/sensors_and_record.log 2>/dev/null && break; sleep 1; done
    grep -q 'STARTUP-ENDED rc=0' /tmp/vio_logs/sensors_and_record.log || { echo "startup failed:"; tail -5 /tmp/vio_logs/sensors_and_record.log; break; }
  else
    "$REPO/record_tool/record.sh" "$DUR" > /dev/null 2>&1
  fi
  sleep 4
  bag=$(ls -dt "$BAGS"/vio_* | head -1)
  [ "$(stat -c %Y "$bag")" -ge "$T0" ] || { echo "could not identify the new bag ($bag is older than the loop)"; break; }
  wait_recording_done || break
  echo "   bag: $bag  ($(du -sh "$bag" | cut -f1))"
  analyze "$bag"
  case "$bag" in "$BAGS"/vio_2*) rm -rf "$bag"; echo "   (bag deleted)";; *) echo "   NOT deleting unexpected path $bag";; esac
  sleep 3
done
echo; echo "stopping sensors"; "$REPO/record_tool/sensors_and_record_notrigger.sh" stop > /dev/null 2>&1
echo "done $(date +%H:%M:%S); disk: $(df -h --output=avail "$BAGS" | tail -1 | tr -d ' ') free"
