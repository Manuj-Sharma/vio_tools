#!/usr/bin/env bash
# rate_reproducibility.sh -- is OpenVINS reproducible if we stop it dropping updates?
#
# At --rate 1.0 the IMU callback skips a camera update whenever the previous one
# is still running (ROS2Visualizer.cpp, "if (thread_update_running) return;").
# Which updates get skipped depends on machine load, so identical input gave
# 41.9 / 3.0 / 10.1 percent MSCKF usage across three runs. Playing slower gives
# each update time to finish, so nothing should be skipped.
#
# Runs the SAME config twice at a reduced rate and once at 1.0 for contrast.
# If the two slow runs agree closely, the async guard is confirmed as the
# non-determinism source and we finally have a reproducible baseline.
#
#   rate_reproducibility.sh [RATE] [REPS]
set -u

RATE="${1:-0.5}"
REPS="${2:-2}"
BAG=/home/vc/rosbags/rosbag2_2026_07_31-11_23_05
CFG=/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_aerial_best/estimator_config.yaml
S=/home/vc/sensor_ws/src/sensor_bringup
OUT=/home/vc/rosbags/calib/tools/rate_results.csv

[ -f "$OUT" ] || echo "rate,rep,msckf_upd,msckf_nonzero,msckf_pct,msckf_feats,slam_feats,final_dist_m,ba_x,ba_y,ba_z,timeouts,log" > "$OUT"

reap () {   # bracketed patterns so this can never match the harness itself
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play"); do kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play"); do kill -KILL "$p" 2>/dev/null; done
  sleep 2
}

run_one () {
  local rate="$1" rep="$2"
  echo "########## rate $rate  rep $rep ##########"
  reap
  local O="/tmp/rate_${rate}_${rep}.out"
  "$S/scripts/run_openvins_bag.sh" "$BAG" --config "$CFG" --no-rviz -r "$rate" > "$O" 2>&1 &
  local PID=$! ok=0
  # 209 s of bag at rate r takes 209/r seconds, plus startup; allow generous margin
  for _ in $(seq 1 900); do
    grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
    grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED"; tail -3 "$O" | sed 's/^/    /'; break; }
    kill -0 "$PID" 2>/dev/null || break
    sleep 3
  done
  local LOG; LOG="$(readlink -f /tmp/vio_logs/openvins_bag_latest.log 2>/dev/null)"
  kill -INT "$PID" 2>/dev/null; sleep 2; kill -KILL "$PID" 2>/dev/null
  reap
  if [ "$ok" -ne 1 ] || [ -z "$LOG" ]; then echo "$rate,$rep,,,,,,,,,,,NORUN" >> "$OUT"; echo "  -> NO RUN"; return; fi
  python3 - "$rate" "$rep" "$LOG" "$OUT" <<'PY'
import re, sys
rate, rep, log, out = sys.argv[1:5]
ms=[]; sl=[]; dist=''; ba=['','','']; to=0
for line in open(log, errors='ignore'):
    m=re.search(r"MSCKF update \((\d+) feats\)",line)
    if m: ms.append(int(m.group(1)))
    m=re.search(r"SLAM update \((\d+) feats\)",line)
    if m: sl.append(int(m.group(1)))
    m=re.search(r"dist = ([\d.]+)",line)
    if m: dist=m.group(1)
    m=re.search(r"ba = ([-\d.]+),([-\d.]+),([-\d.]+)",line)
    if m: ba=list(m.groups())
    if "Maximum solver time reached" in line: to+=1
nz=sum(1 for x in ms if x>0); pct=100*nz/len(ms) if ms else 0
open(out,'a').write(",".join(str(x) for x in
    [rate,rep,len(ms),nz,f"{pct:.1f}",sum(ms),sum(sl),dist,*ba,to,log.split('/')[-1]])+"\n")
print(f"  -> MSCKF {nz}/{len(ms)} ({pct:.1f}%) {sum(ms)} feats | SLAM {sum(sl)} | dist {dist} m | ba {','.join(ba)}")
PY
}

for r in $(seq 1 "$REPS"); do run_one "$RATE" "$r"; done
run_one 1.0 ref
echo "SWEEPDONE -> $OUT"
