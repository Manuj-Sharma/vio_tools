#!/usr/bin/env bash
# cam_order_check.sh - log which physical camera (sensor serial) the VizionSDK puts at
# index 0 (-> /cam0) and index 1 (-> /cam1), once per run, with the boot id.
# Appends one CSV row per camera to ~/vio_tools/records/cam_order_log.csv and compares
# against the first row ever logged (the baseline). Run it after each power-on.
# Do not run while the camera nodes are up (they hold the devices).
set -u
REPO="${REPO:-$HOME/vio_ar0234_icm20948}"
LOG="${LOG:-$HOME/vio_tools/records/cam_order_log.csv}"
mkdir -p "$(dirname "$LOG")"

if pgrep -x vizionsdk_camer >/dev/null; then
  echo "camera nodes are running - stop the stack first (stop_vio_stack.sh)"; exit 2
fi

set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash" 2>/dev/null; set -u
OUT="$("$REPO/build/vizionsdk_ros2/list_devices" 2>/dev/null)"
[ -n "$OUT" ] || { echo "list_devices printed nothing"; exit 1; }

BOOT="$(cat /proc/sys/kernel/random/boot_id)"
NOW="$(date -Iseconds)"
UP="$(cut -d. -f1 /proc/uptime)"
[ -s "$LOG" ] || echo "time,boot_id,uptime_s,index,video_path,serial" > "$LOG"

echo "$OUT" | awk -v t="$NOW" -v b="$BOOT" -v u="$UP" '
  /^\[[0-9]+\]/ { gsub(/[\[\]]/,"",$1); idx=$1 }
  /path:/       { path=$2 }
  /serial:/     { printf "%s,%s,%s,%s,%s,%s\n", t, b, u, idx, path, $2 }' >> "$LOG"

# baseline = serials at index 0 and 1 from the first logged boot
BASE0="$(awk -F, 'NR>1 && $4==0 {print $6; exit}' "$LOG")"
BASE1="$(awk -F, 'NR>1 && $4==1 {print $6; exit}' "$LOG")"
NOW0="$(echo "$OUT" | awk '/^\[0\]/{f=1} /serial:/{if(f){print $2; exit}}')"
NOW1="$(echo "$OUT" | awk '/^\[1\]/{f=1} /serial:/{if(f){print $2; exit}}')"
echo "index0 (cam0): $NOW0"; echo "index1 (cam1): $NOW1"
if [ "$NOW0" = "$BASE0" ] && [ "$NOW1" = "$BASE1" ]; then
  echo "SAME as baseline ($(awk -F, 'NR>1{print $2}' "$LOG" | sort -u | wc -l) boot(s) logged)"
else
  echo "*** SWAPPED or changed vs baseline - cam0/cam1 are NOT the same physical cameras ***"; exit 3
fi
