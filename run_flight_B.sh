#!/usr/bin/env bash
# run_flight_B.sh [REPS] -- verify ov_9281_flight_B_imutrust on seg01.
#
#   ./run_flight_B.sh            # one run, no rviz
#   ./run_flight_B.sh 3          # three runs (recommended -- see below)
#   RVIZ=1 ./run_flight_B.sh     # watch it
#   CFG=/path/to/estimator_config.yaml ./run_flight_B.sh    # compare another config
#
# WHY THREE REPS
#
# This pipeline is NOT deterministic. The same config has produced MSCKF 66.1%
# and 18.8% on identical inputs, and box errors of +2.3% and +35%. A single run
# tells you almost nothing. Three at least shows whether a setting is reliable
# or merely lucky.
#
# WHAT IS BEING MEASURED
#
# The flown box is 267 x 131 m at a barometric peak of 107.75 m. That is the only
# ground truth in this dataset -- there is no GPS. score_box.py extracts the box
# by PCA on the cruise-phase track, so heading does not matter.
#
# MSCKF PERCENTAGE IS A BLIND METRIC. Runs at 69.4% / 66.1% / 71.1% produced box
# errors of +35% / +2% / +36%. Judge on errL/errW/errZ, not on MSCKF.
set -u

REPS="${1:-1}"
BAG="${BAG:-/home/vc/rosbags/field_data/seg01_ros2_baro}"
CFG="${CFG:-/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_flight_B_imutrust/estimator_config.yaml}"
export OV_WS="${OV_WS:-$HOME/openvins_baro_ws}"
S=/home/vc/sensor_ws/src/sensor_bringup
LOGS=/tmp/flight_B_logs.txt
RV=""; [ "${RVIZ:-0}" = "1" ] || RV="--no-rviz"

[ -f "$BAG/metadata.yaml" ] || { echo "no bag at $BAG" >&2; exit 1; }
[ -f "$CFG" ] || { echo "no config at $CFG" >&2; exit 1; }
grep -qE "^use_baro:[[:space:]]*(true|1)" "$CFG" || {
  echo "REFUSING: $CFG has no 'use_baro: true' -- the run would silently be" >&2
  echo "  plain VIO with the barometer off, which looks plausible and means nothing." >&2
  exit 1; }

: > "$LOGS"
reap () {   # bracketed patterns cannot match this script's own cmdline
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") $(pgrep -x rviz2); do kill -KILL "$p" 2>/dev/null; done
  sleep 8
}

echo "config : $CFG"
echo "bag    : $BAG"
echo "OV_WS  : $OV_WS"
echo "reps   : $REPS"
echo
grep -E "noise_density|random_walk" "$(dirname "$CFG")/kalibr_imu_chain.yaml" | sed 's/^/  /'
grep -E "^up_(msckf|slam)_sigma_px|^use_baro|^baro_" "$CFG" | sed 's/^/  /'
echo

for rep in $(seq 1 "$REPS"); do
  echo "############### rep $rep/$REPS ###############"
  reap
  M=/tmp/fbmark.$$; touch "$M"; O="/tmp/flight_B_$rep.out"
  "$S/scripts/run_openvins_bag.sh" "$BAG" --config "$CFG" $RV > "$O" 2>&1 &
  PID=$!; ok=0
  for _ in $(seq 1 400); do
    grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
    grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED -- see $O"; break; }
    kill -0 "$PID" 2>/dev/null || break
    sleep 4
  done
  sleep 25                       # let the backlog drain
  # newest log created after the marker -- never trust openvins_bag_latest.log,
  # which points at the wrong file whenever a stale node is around
  L="$(find /tmp/vio_logs -name 'openvins_bag_*.log' -newer "$M" -printf '%T@ %p\n' 2>/dev/null \
       | sort -rn | head -1 | cut -d' ' -f2-)"
  rm -f "$M"; kill -INT "$PID" 2>/dev/null; sleep 3; kill -KILL "$PID" 2>/dev/null; reap
  [ "$ok" -eq 1 ] && [ -n "$L" ] || { echo "  -> NO RUN"; continue; }
  echo "$L" >> "$LOGS"
  echo "  log: $L"
  grep -oE "ba = [-0-9.,]+" "$L" | tail -1 | sed 's/^/     final /'
  grep -oE "reference captured.*" "$L" | head -1 | sed 's/^/     /'
done

echo
echo "=========================== THIS CONFIG ==========================="
python3 /home/vc/rosbags/calib/tools/score_box.py $(cat "$LOGS")
echo
echo "=========================== REFERENCE ============================="
echo "  flight_A inflated IMU, bad mode :  errL +35.2%  errW +16.4%  errZ  -2.5%"
echo "  flight_A inflated IMU, good mode:  errL  +2.3%  errW  +0.1%  errZ  -4.3%"
echo "  bench IMU, sigma_px 2           :  errL +12.0%  errW  -1.0%  errZ +17.1%"
echo
echo "  A healthy accel bias is +-0.09 m/s^2 or less. flight_A reached +-0.36"
echo "  and flipped sign between runs -- that is the bias absorbing error the"
echo "  nadir geometry cannot observe, not a real bias."
