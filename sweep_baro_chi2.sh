#!/usr/bin/env bash
# sweep_baro_chi2.sh [mult ...] -- widen the gate WITHOUT weakening the correction.
#
#   ./sweep_baro_chi2.sh            # sweeps 5 20 50
#   ./sweep_baro_chi2.sh 20         # just one
#
# WHY
#
# baro_sigma moves two things at once:
#     gate boundary  |res| > sqrt(chi2_multipler * 3.841 * S),  S ~= sigma^2
#     Kalman gain    K ∝ P/(P+R),                               R  = sigma^2
# so loosening sigma widens the gate AND weakens every correction. Measured:
# sigma 2.0 cut rejections 31.6% -> 1.8% but made cruise, descent, bias and peak
# error all worse, because the barometer no longer had the authority to correct.
#
# chi2_multipler touches ONLY the gate. That is the knob we actually want.
#
# Bias is pinned (prior 0.001, rw 0) because that gave the best cruise number
# (3.60 m vs 5.15), and sigma stays 0.5 so corrections keep their strength. The
# open question is whether a wider gate rescues the descent, which collapsed to
# 11.77 m RMS when pinning pushed residuals past the +-2.2 m boundary.
set -u

MULTS=("$@"); [ ${#MULTS[@]} -eq 0 ] && MULTS=(5 20 50)
BAG=${BAG:-/home/vc/rosbags/field_data/seg01_ros2_baro}
SRC=${SRC:-/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_flight_A_baro_enabled}
export OV_WS=${OV_WS:-$HOME/openvins_baro_ws}
S=/home/vc/sensor_ws/src/sensor_bringup
TMP=/tmp/baro_chi2_cfg
LOGS=/home/vc/rosbags/calib/tools/chi2_logs.txt
: > "$LOGS"

reap () {
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -KILL "$p" 2>/dev/null; done
  sleep 8
}

for M in "${MULTS[@]}"; do
  echo "############### baro_chi2_multipler = $M ###############"
  reap
  rm -rf "$TMP"; cp -r "$SRC" "$TMP"
  sed -i '/^baro_/d' "$TMP/estimator_config.yaml"
  # no trailing comments -- a colon in one makes cv::FileStorage read an empty value
  {
    echo "baro_bias_prior: 0.001"
    echo "baro_bias_rw: 0.0"
    echo "baro_sigma: 0.5"
    echo "baro_chi2_multipler: $M"
  } >> "$TMP/estimator_config.yaml"

  MARK=/tmp/chi2_mark.$$; touch "$MARK"
  O="/tmp/chi2_${M}.out"
  "$S/scripts/run_openvins_bag.sh" "$BAG" --config "$TMP/estimator_config.yaml" --no-rviz > "$O" 2>&1 &
  PID=$!; ok=0
  for _ in $(seq 1 400); do
    grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
    grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED -- see $O"; break; }
    kill -0 "$PID" 2>/dev/null || break
    sleep 4
  done
  sleep 25
  LOG="$(find /tmp/vio_logs -name 'openvins_bag_*.log' -newer "$MARK" -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
  rm -f "$MARK"
  kill -INT "$PID" 2>/dev/null; sleep 3; kill -KILL "$PID" 2>/dev/null; reap
  [ "$ok" -eq 1 ] && [ -n "$LOG" ] || { echo "  -> NO RUN"; continue; }
  echo "$LOG" >> "$LOGS"
  echo "  mult=$M -> $LOG"
  grep -oE "baro_chi2_multipler: [0-9.]+|chi2_multipler: [0-9.]+" "$LOG" | sort -u | sed 's/^/     parsed /'
done

echo
echo "==================== SCORED ===================="
python3 /home/vc/rosbags/calib/tools/score_baro.py $(cat "$LOGS")
