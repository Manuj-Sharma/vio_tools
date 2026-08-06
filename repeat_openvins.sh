#!/usr/bin/env bash
# repeat_openvins.sh -- run each config N times on one bag and record the spread.
#
# WHY REPEATS: on 2026-08-03 three identical runs of the same bag with the same
# config returned 41.9 / 3.0 / 10.1 percent MSCKF usage. The estimator is
# non-deterministic (wall-clock-limited, multi-threaded init MLE), so a single
# run cannot distinguish a config change from a lucky draw. Any effect smaller
# than the run-to-run spread is unmeasurable.
#
# Results append to a CSV as they complete, so an interrupted sweep keeps what
# it already measured.
#
#   repeat_openvins.sh [REPS] [BAG]
#
# Lessons baked in, each from a failure earlier the same day:
#   * run_openvins_bag.sh keeps OpenVINS alive after playback by design, and the
#     next invocation is then REFUSED by its own duplicate-replay guard -- which
#     silently turns a sweep into a series of no-ops that still look like runs.
#     So wait for "playback finished", then SIGINT and verify teardown.
#   * Resolve the log path BEFORE teardown relinks openvins_bag_latest.log.
#   * Never pgrep -f a pattern that matches this script's own command line; use
#     a bracketed pattern so it cannot self-match and kill the harness.
set -u

REPS="${1:-3}"
BAG="${2:-/home/vc/rosbags/rosbag2_2026_07_31-11_23_05}"
S=/home/vc/sensor_ws/src/sensor_bringup
OUT=/home/vc/rosbags/calib/tools/repeat_results.csv
CONFIGS="ov_9281_aerial_best ov_9281_aerial_staticinit ov_9281_aerial_slowmle ov_9281_aerial_det"

[ -f "$OUT" ] || echo "config,rep,msckf_upd,msckf_nonzero,msckf_pct,msckf_feats,slam_nonzero,slam_feats,final_dist_m,ba_x,ba_y,ba_z,init_ok,solver_timeouts,log" > "$OUT"

reap () {   # bracketed patterns cannot match this script itself
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play"); do kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play"); do kill -KILL "$p" 2>/dev/null; done
  sleep 2
}

for cfg in $CONFIGS; do
  for rep in $(seq 1 "$REPS"); do
    echo "########## $cfg  rep $rep/$REPS ##########"
    reap
    O="/tmp/rep_${cfg}_${rep}.out"
    "$S/scripts/run_openvins_bag.sh" "$BAG" \
        --config "$S/config/$cfg/estimator_config.yaml" --no-rviz > "$O" 2>&1 &
    PID=$!
    ok=0
    for _ in $(seq 1 400); do
      grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
      grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED/ERROR"; tail -3 "$O" | sed 's/^/    /'; break; }
      kill -0 "$PID" 2>/dev/null || break
      sleep 3
    done
    LOG="$(readlink -f /tmp/vio_logs/openvins_bag_latest.log 2>/dev/null)"
    kill -INT "$PID" 2>/dev/null; sleep 2; kill -KILL "$PID" 2>/dev/null
    reap
    if [ "$ok" -ne 1 ] || [ -z "$LOG" ] || [ ! -f "$LOG" ]; then
      echo "$cfg,$rep,,,,,,,,,,,NORUN,," >> "$OUT"; echo "  -> NO RUN"; continue
    fi
    python3 - "$cfg" "$rep" "$LOG" "$OUT" <<'PY'
import re, sys
cfg, rep, log, out = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
ms=[]; sl=[]; dist=''; ba=['','','']; init=0; to=0
for line in open(log, errors='ignore'):
    m=re.search(r"MSCKF update \((\d+) feats\)",line)
    if m: ms.append(int(m.group(1)))
    m=re.search(r"SLAM update \((\d+) feats\)",line)
    if m: sl.append(int(m.group(1)))
    m=re.search(r"dist = ([\d.]+)",line)
    if m: dist=m.group(1)
    m=re.search(r"ba = ([-\d.]+),([-\d.]+),([-\d.]+)",line)
    if m: ba=list(m.groups())
    if "successful initialization" in line: init=1
    if "Maximum solver time reached" in line: to+=1
nz=sum(1 for x in ms if x>0)
pct=100*nz/len(ms) if ms else 0
row=[cfg,rep,len(ms),nz,f"{pct:.1f}",sum(ms),sum(1 for x in sl if x>0),sum(sl),dist,*ba,init,to,log.split('/')[-1]]
open(out,'a').write(",".join(str(x) for x in row)+"\n")
print(f"  -> MSCKF {nz}/{len(ms)} ({pct:.1f}%) {sum(ms)} feats | dist {dist} m | ba {','.join(ba)} | timeouts {to}")
PY
  done
done
echo "SWEEPDONE  ->  $OUT"
