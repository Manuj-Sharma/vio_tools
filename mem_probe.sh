#!/usr/bin/env bash
# mem_probe.sh [config] [reps] -- run replays while sampling memory, then report
# whether the memory state actually correlates with the outcome.
#
#   ./mem_probe.sh best 3        flight_D_best, 3 reps   <- the control
#   ./mem_probe.sh gold 3        gold standard, 3 reps
#
# WHY
#
# `free` drops to ~100 MB during a run, which looks alarming. It is the 6.2 GB
# bag streaming into page cache -- reclaimable, and `available` stays >4 GB.
# But that does NOT rule out RECLAIM PRESSURE: kswapd freeing pages while the
# filter runs could inject latency that the estimator's own "ms behind" figure
# never sees, because that measures queue lag, not stalls inside a callback.
#
# So sample it and correlate. Per rep this records:
#   min available     the real headroom, not `free`
#   swap used delta   >0 means genuine eviction, the thing that would matter
#   pgmajfault delta  major faults = pages fetched from disk = actual stalls
#   pgscan delta      how hard kswapd worked
# against MSCKF% and max distance.
#
# If collapsed reps show materially worse memory numbers, the hypothesis holds.
# If good and bad reps look identical, memory is ruled out with evidence.
set -u

CFG_NAME="${1:-best}"
REPS="${2:-3}"
case "$CFG_NAME" in
  best)      CFG=ov_9281_flight_D_best ;;
  gold)      CFG=ov_9281_aerial_staticinit ;;
  gold_baro) CFG=ov_9281_gold_baro ;;
  gold_px2)  CFG=ov_9281_gold_px2 ;;
  *)         CFG="$CFG_NAME" ;;
esac

S=/home/vc/sensor_ws/src/sensor_bringup/scripts
C=/home/vc/sensor_ws/src/sensor_bringup/config/$CFG/estimator_config.yaml
BAG=/home/vc/rosbags/field_data/seg01_ros2_baro
OUT=/home/vc/rosbags/calib/results_20260805
[ -f "$C" ] || { echo "no config: $C" >&2; exit 1; }
mkdir -p "$OUT"
grep -qE "^use_baro:[[:space:]]*true" "$C" && export OV_WS="${OV_WS:-$HOME/openvins_baro_ws}"

set +u; source /opt/ros/humble/setup.bash; set -u
echo "config: $CFG   reps: $REPS   OV_WS: ${OV_WS:-$HOME/openvins_ws}"
echo

vmstat_get() { awk -v k="$1" '$1==k{print $2}' /proc/vmstat; }

for r in $(seq 1 "$REPS"); do
  echo "########## rep $r/$REPS ##########"
  MJ0=$(vmstat_get pgmajfault); SC0=$(vmstat_get pgscan_kswapd)
  SW0=$(awk '/SwapFree/{print $2}' /proc/meminfo)
  M=$(mktemp); touch "$M"; SAMP=$(mktemp)
  ( while :; do awk '/MemAvailable/{a=$2} /SwapFree/{s=$2} END{print a, s}' /proc/meminfo >> "$SAMP"; sleep 2; done ) &
  SPID=$!
  "$S/run_openvins_bag.sh" "$BAG" --config "$C" --no-rviz > /tmp/mem_rep_$r.out 2>&1
  kill "$SPID" 2>/dev/null
  MJ1=$(vmstat_get pgmajfault); SC1=$(vmstat_get pgscan_kswapd)
  SW1=$(awk '/SwapFree/{print $2}' /proc/meminfo)
  L="$(find /tmp/vio_logs -name 'openvins_bag_*.log' -newer "$M" -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
  rm -f "$M"
  MINAV=$(awk '{if(min==""||$1<min)min=$1}END{printf "%.0f", min/1024}' "$SAMP")
  rm -f "$SAMP"
  [ -n "$L" ] || { echo "  no log"; continue; }
  python3 - "$L" "$MINAV" "$((MJ1-MJ0))" "$((SC1-SC0))" "$(( (SW0-SW1)/1024 ))" <<'EOF'
import re,sys,numpy as np
t=open(sys.argv[1],errors='ignore').read()
ms=np.array([int(x) for x in re.findall(r"MSCKF update \((\d+) feats\)",t)])
P=np.array([[float(x) for x in m.groups()] for m in re.finditer(r"p_IinG = ([-\d.]+),([-\d.]+),([-\d.]+)",t)])
if not len(ms) or not len(P): sys.exit("  too short")
pct=100*np.mean(ms>0); d=np.linalg.norm(P[:,:2],axis=1).max()
v="COLLAPSE" if (pct<40 or d>400) else "ok"
print(f"  MSCKF {pct:5.1f}%  maxdist {d:9.1f} m  -> {v}")
print(f"  memory: min available {sys.argv[2]} MB | majfaults {sys.argv[3]} | "
      f"kswapd pgscan {sys.argv[4]} | swap grew {sys.argv[5]} MB")
EOF
  python3 /home/vc/rosbags/calib/tools/vs_gps.py "$L" 2>&1 \
    | grep -vE "^  (GPS:|scale 1.000|yaw is|since VIO|rms/max)" | grep -v '^$'
  echo
done
echo "Compare the memory line between ok and COLLAPSE reps."
echo "Similar numbers => memory ruled out. Worse on collapses => it is real."
