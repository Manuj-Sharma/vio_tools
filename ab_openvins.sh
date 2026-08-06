#!/usr/bin/env bash
# A/B two OpenVINS configs on the SAME bag, one after the other.
#
# WHY THIS IS NOT JUST TWO CALLS IN A ROW:
# run_openvins_bag.sh deliberately keeps OpenVINS alive after playback ends
# ("so you can inspect the final trajectory"), so it never exits by itself.
# Left running it still owns the topics, and the NEXT invocation hits that
# script's own duplicate-replay guard and refuses -- which silently turns an
# A/B into two no-ops that still look like they ran (both logs end up being the
# same stale file). So after each config: wait for "playback finished", SIGINT
# the script, and let its EXIT trap tear ov_msckf down before the next one.
#
#   ab_openvins.sh [BAG]     default: the field_data drone bag
set -u

BAG="${1:-/home/vc/rosbags/field_data/rosbag2_2026_07_31-11_23_05}"
S=/home/vc/sensor_ws/src/sensor_bringup
CONFIGS="ov_9281_bag_recal ov_9281_bag_recal_aerial"

for name in $CONFIGS; do
  echo "########## $name ##########"
  OUT="/tmp/ab_$name.out"
  "$S/scripts/run_openvins_bag.sh" "$BAG" \
      --config "$S/config/$name/estimator_config.yaml" --no-rviz > "$OUT" 2>&1 &
  PID=$!

  REFUSED=0
  for _ in $(seq 1 400); do
    grep -q "playback finished" "$OUT" 2>/dev/null && break
    if grep -q "ERROR:" "$OUT" 2>/dev/null; then
      REFUSED=1; echo "  REFUSED/ERROR:"; tail -3 "$OUT" | sed 's/^/    /'; break
    fi
    kill -0 "$PID" 2>/dev/null || break
    sleep 2
  done

  # resolve the log BEFORE teardown relinks it for the next run
  LOGFILE="$(readlink -f /tmp/vio_logs/openvins_bag_latest.log 2>/dev/null)"
  kill -INT "$PID" 2>/dev/null
  wait "$PID" 2>/dev/null
  # belt and braces: the trap normally handles these, but a refused run has no trap
  pkill -x run_subscribe_m 2>/dev/null
  pkill -f "[b]ag play" 2>/dev/null
  sleep 3

  if [ "$REFUSED" -eq 0 ] && [ -n "$LOGFILE" ]; then
    cp "$LOGFILE" "/tmp/ovlog_$name.log"
    echo "  $name -> $(basename "$LOGFILE")  ($(wc -l < "/tmp/ovlog_$name.log") lines)"
  else
    echo "  $name -> NO LOG (run did not start)"
  fi
done
echo ABDONE
