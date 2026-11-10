#!/usr/bin/env bash
# One headless OpenVINS replay of a recorded bag through imu_filter_node, then clean shutdown.
#   ov_run_replay.sh BAG LABEL CUTOFF_HZ [ORDER=2] [OUT_HZ=300]
# Uses the 2026-11-06 Kalibr solve (ar0234_mono_cam0_nt_filtered), cam0, no RViz, no GPS. Real time (-r 1).
# Records /ov_msckf/odomimu to ~/rosbags/ov_<LABEL>_<time>/ and copies the OpenVINS log to <that dir>.log.
# Waits for "playback finished", then stops OpenVINS + the filter node. Refuses to start if a filter node/OpenVINS/live sensors run.
# Exit 0 = replay completed.
set -u
[ $# -ge 3 ] || { sed -n '2,8p' "$0"; exit 1; }
BAG="$1"; LABEL="$2"; FC="$3"; ORD="${4:-2}"; OUT="${5:-300}"
REPO="$HOME/vio_ar0234_icm20948"; L=/tmp/vio_logs; mkdir -p "$L"
for p in imu_filter_node run_subscribe_m imx5_spi_node; do pgrep -x "$p" >/dev/null && { echo "$p is running -- stop it first" >&2; exit 1; }; done
[ -d "$BAG" ] || { echo "no bag $BAG" >&2; exit 1; }
OVB="$HOME/rosbags/ov_${LABEL}_$(date +%H%M%S)"; RL="$L/replay_${LABEL}.log"; : > "$RL"
DUR=$(python3 - "$BAG" <<'PY'
import sys,re
t=open(sys.argv[1].rstrip('/')+'/metadata.yaml').read(); m=re.search(r'nanoseconds:\s*(\d+)',t); print(int(int(m.group(1))*1e-9) if m else 120)
PY
)
echo "[run] $LABEL: order $ORD, cutoff $FC Hz, out $OUT Hz | bag $BAG (${DUR}s) | odom -> $OVB"
CFG="$REPO/openvins_config/ar0234_mono_cam0_nt_filtered/estimator_config.yaml" \
FILT_ORDER="$ORD" FILT_CUTOFF="$FC" FILT_OUT_HZ="$OUT" REC_OUT="$OVB" \
  setsid nohup "$HOME/vio_tools/vio_replay_with_filter.sh" "$BAG" --cam 0 --no-rviz --no-gps > "$RL" 2>&1 < /dev/null &
RP=$!
t0=$SECONDS; ok=0
while (( SECONDS - t0 < DUR + 150 )); do
  grep -a -q "playback finished" "$RL" && { ok=1; break; }
  kill -0 $RP 2>/dev/null || break
  sleep 2
done
sleep 3
echo "[run] $LABEL: playback finished=$ok after $((SECONDS - t0)) s; stopping"
"$HOME/vio_tools/stop_replay.sh" > /dev/null 2>&1
sleep 2
cp -L "$L/openvins_bag_latest.log" "$OVB.log" 2>/dev/null
cp "$L/imu_filter_node.log" "$OVB.filter.log" 2>/dev/null
for p in imu_filter_node run_subscribe_m; do pgrep -x "$p" >/dev/null && echo "[run] WARNING: $p still running"; done
echo "[run] odom bag: $OVB  log: $OVB.log"
exit $(( ok ? 0 : 1 ))
