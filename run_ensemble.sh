#!/usr/bin/env bash
# run_ensemble.sh <tagprefix> <bag> <cfg> <cam_topic> <stop_after_s> <offset1> [offset2 ...]
# Runs the SAME config at slightly different start offsets (changes frame phase / ZUPT handover timing = a fair "repeat"),
# one at a time, each stopped <stop_after_s> seconds after launch, then scores it (score_run.py -> records/runs.csv).
# Machine must be QUIET. Uses OV_WS if set. Status lines go to /tmp/claude-1000/ens_<tagprefix>.status
PFX=$1; BAG=$2; CFG=$3; CAM=$4; STOP=$5; shift 5
ST=/tmp/claude-1000/ens_$PFX.status; : > $ST
for OFF in "$@"; do
  TAG=${PFX}_o${OFF}
  ~/vio_tools/run_bag.sh $TAG $BAG $CFG $OFF $CAM 0-23 > /tmp/claude-1000/runbag_$TAG.out 2>&1 &
  PID=$!
  sleep $STOP
  kill -TERM $PID; wait $PID 2>/dev/null; sleep 4
  O=$HOME/rosbags/claude_tmp/runs_$(date +%Y%m%d)/$TAG
  source /opt/ros/humble/setup.bash
  python3 ~/vio_tools/score_run.py $O > $O/score_stdout.txt 2>&1
  echo "$TAG done: $(grep -E '^  10:' $O/results.txt | cut -c1-120)" >> $ST
done
echo ALL_DONE >> $ST
