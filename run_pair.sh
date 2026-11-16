#!/usr/bin/env bash
# run_pair.sh <bag> <offset> <cam_topic> <tagprefix> <cfgA> <cfgB> -- run cfgA then cfgB on one bag, stop each cleanly (by PID), score each.
BAG=$1; OFF=$2; CAM=$3; PFX=$4; shift 4
n=0
for CFG in "$@"; do
  n=$((n+1)); TAG=${PFX}_c$n
  ~/vio_tools/run_bag.sh $TAG $BAG $CFG $OFF $CAM 0-23 > /tmp/claude-1000/runbag_$TAG.out 2>&1 &
  PID=$!
  O=$HOME/rosbags/claude_tmp/runs_$(date +%Y%m%d)/$TAG
  until grep -q PLAY_DONE $O/play.log 2>/dev/null; do sleep 5; done
  sleep 15; kill -INT $PID; wait $PID 2>/dev/null; sleep 3
  source /opt/ros/humble/setup.bash; python3 ~/vio_tools/score_run.py $O > /dev/null 2>&1
  echo "$TAG done" >> /tmp/claude-1000/run_pair_$PFX.status
done
echo ALL_DONE >> /tmp/claude-1000/run_pair_$PFX.status
