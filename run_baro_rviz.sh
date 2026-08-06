#!/usr/bin/env bash
# run_baro_rviz.sh [BAG] -- one run with RViz, using the committed baro config.
#
#   ./run_baro_rviz.sh                          # seg01_ros2_baro
#   ./run_baro_rviz.sh ~/rosbags/field_data/other_bag
#   CFG=/path/to/estimator_config.yaml ./run_baro_rviz.sh
#
# Ctrl+C when you have seen enough. Scoring still runs -- there is an EXIT trap,
# so an interrupted run is still analysed rather than thrown away.
#
# Note: Ctrl+C while an update is in flight can make run_subscribe_msckf exit
# with signal 11. That is a ROS2 shutdown race, not a crash in the estimator, and
# the log written before it is complete and valid.
set -u

BAG=${1:-/home/vc/rosbags/field_data/seg01_ros2_baro}
CFG=${CFG:-/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_flight_A_baro_enabled/estimator_config.yaml}
export OV_WS=${OV_WS:-$HOME/openvins_baro_ws}
S=/home/vc/sensor_ws/src/sensor_bringup
SCORE=/home/vc/rosbags/calib/tools/score_baro.py

[ -f "$BAG/metadata.yaml" ] || { echo "no bag at $BAG" >&2; exit 1; }
[ -f "$CFG" ] || { echo "no config at $CFG" >&2; exit 1; }

# Refuse a config with the barometer off. Without this the run completes, looks
# plausible, and is simply unfused VIO -- which is exactly what happened when
# ov_9281_flight_A (no baro keys at all) got used by mistake and the residuals
# came out at 19.75 m RMS with nothing correcting them.
if ! grep -qE "^use_baro:[[:space:]]*(true|1)[[:space:]]*$" "$CFG"; then
  echo "REFUSING: $CFG has no 'use_baro: true'." >&2
  echo "  the run would silently be plain VIO with the barometer disabled." >&2
  echo "  did you mean ov_9281_flight_A_baro_enabled ?" >&2
  exit 1
fi

reap () {   # bracketed patterns cannot match this script's own cmdline
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -TERM "$p" 2>/dev/null; done
  sleep 3
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -x rviz2); do kill -KILL "$p" 2>/dev/null; done
  sleep 4
}

MARK=/tmp/rviz_run_mark.$$
finish () {
  # pick the log by mtime against the marker -- never trust the `latest` symlink,
  # which points at the wrong file whenever a stale node is around
  LOG="$(find /tmp/vio_logs -name 'openvins_bag_*.log' -newer "$MARK" -printf '%T@ %p\n' 2>/dev/null \
         | sort -rn | head -1 | cut -d' ' -f2-)"
  rm -f "$MARK"
  reap
  echo
  if [ -z "${LOG:-}" ]; then echo "no log produced"; exit 0; fi
  echo "log: $LOG"
  echo
  echo "config actually parsed:"
  grep -oE "use_baro: [0-9]|baro_bias_prior: [0-9.]+|baro_sigma: [0-9.]+|baro_bias_rw: [0-9.]+" "$LOG" \
    | sort -u | sed 's/^/  /'
  grep -oE "reference captured.*" "$LOG" | head -1 | sed 's/^/  /'
  if ! grep -q "use_baro: 1" "$LOG"; then
    echo
    echo "  *** use_baro parsed as 0 -- the barometer did NOT run. Scores below are"
    echo "      unfused VIO and are not comparable to anything." >&2
  fi
  # A partial run has no cruise or descent, so its numbers cannot be compared to
  # the full-flight reference. Say so rather than let a 15 s clip look excellent.
  NB=$(grep -c '\[BARO\]: t=' "$LOG")
  if [ "$NB" -lt 500 ]; then
    echo
    echo "  *** only $NB barometer updates (a full seg01 run has ~867)."
    echo "      This was stopped early -- likely before cruise/descent, which is"
    echo "      where the interesting errors live. Let it reach 'playback finished'."
  fi
  echo
  echo "================================ SCORE ================================"
  python3 "$SCORE" "$LOG"
  echo
  echo "columns: raw = (z_baro - h_ref) - z_vio, i.e. barometer-vs-VIO with the"
  echo "         bias EXCLUDED, so a bias absorbing error cannot flatter it."
  echo "         climb/cruise/desc are RMS in metres, split by barometer altitude."
  echo "reference (this config, seg01): MSCKF 69.4%  climb 0.92  cruise 5.15  desc 5.50"
}
trap finish EXIT

echo "bag    : $BAG"
echo "config : $CFG"
echo "OV_WS  : $OV_WS"
echo
grep -E "^(use_baro|baro_)" "$CFG" | sed 's/^/  /'
echo
reap
touch "$MARK"
"$S/scripts/run_openvins_bag.sh" "$BAG" --config "$CFG"
