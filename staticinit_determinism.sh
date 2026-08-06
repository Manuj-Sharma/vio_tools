#!/usr/bin/env bash
# staticinit_determinism.sh -- does static init give reproducible results at FULL rate?
#
# THE QUESTION. Dynamic init runs a Ceres solve that is bounded by wall-clock
# time (init_dyn_mle_max_time) and multi-threaded, and it runs in a background
# thread while frames keep arriving -- so which frames reach the initialiser
# depends on machine load. Measured on 2026-08-03, identical bag and config at
# --rate 1.0 initialised at frame 231 / 246 and gave 3.0 / 10.1 / 33.3 / 41.9
# percent MSCKF usage. At --rate 0.5 it initialised at frame 229 twice with a
# byte-identical state and gave 43.8 / 43.5 percent.
#
# Static init has no solver at all -- it waits for an accel jerk and asserts
# velocity = 0 -- so it should be reproducible WITHOUT halving the playback
# rate. If that holds, every future replay runs at full speed.
#
# Two identical runs. What matters is agreement between them, not the absolute
# value.
set -u

BAG=/home/vc/rosbags/rosbag2_2026_07_31-11_23_05
CFG=/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_aerial_staticinit/estimator_config.yaml
S=/home/vc/sensor_ws/src/sensor_bringup
OUT=/home/vc/rosbags/calib/tools/staticinit_results.csv

[ -f "$OUT" ] || echo "rep,rate,init_frame,init_ori,init_ba,init_vel,msckf_upd,msckf_nonzero,msckf_pct,msckf_feats,slam_feats,final_dist_m,ba_final,log" > "$OUT"

reap () {  # bracketed patterns cannot match this script itself.
           # Also kill the ros2 launch parent -- a suspended parent leaves an
           # unreapable zombie that still matches the runner's duplicate guard.
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play") $(pgrep -f "[r]os2 launch sensor_bringup"); do
    kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play") $(pgrep -f "[r]os2 launch sensor_bringup"); do
    kill -KILL "$p" 2>/dev/null; done
  sleep 3
}

for rep in 1 2; do
  echo "########## static init, rate 1.0, rep $rep ##########"
  reap
  O="/tmp/si_rep${rep}.out"
  "$S/scripts/run_openvins_bag.sh" "$BAG" --config "$CFG" --no-rviz > "$O" 2>&1 &
  PID=$!; ok=0
  for _ in $(seq 1 400); do
    grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
    grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED"; tail -3 "$O" | sed 's/^/    /'; break; }
    kill -0 "$PID" 2>/dev/null || break
    sleep 3
  done
  LOG="$(readlink -f /tmp/vio_logs/openvins_bag_latest.log 2>/dev/null)"
  kill -INT "$PID" 2>/dev/null; sleep 2; kill -KILL "$PID" 2>/dev/null
  reap
  [ "$ok" -eq 1 ] && [ -n "$LOG" ] || { echo "$rep,1.0,,,,,,,,,,,NORUN," >> "$OUT"; echo "  -> NO RUN"; continue; }
  python3 - "$rep" "$LOG" "$OUT" <<'PY'
import re, sys
rep, log, out = sys.argv[1], sys.argv[2], sys.argv[3]
ms=[]; sl=[]; dist=''; ba=''; klt=0; init_klt=None
init_ori=init_ba=init_vel=''
for line in open(log, errors='ignore'):
    if 'seconds for tracking' in line: klt+=1
    if 'successful initialization' in line and init_klt is None: init_klt=klt
    m=re.search(r"\[init\]: orientation = (.+)",line)
    if m and not init_ori: init_ori=m.group(1).strip()
    m=re.search(r"\[init\]: bias accel = (.+)",line)
    if m and not init_ba: init_ba=m.group(1).strip()
    m=re.search(r"\[init\]: velocity = (.+)",line)
    if m and not init_vel: init_vel=m.group(1).strip()
    m=re.search(r"MSCKF update \((\d+) feats\)",line)
    if m: ms.append(int(m.group(1)))
    m=re.search(r"SLAM update \((\d+) feats\)",line)
    if m: sl.append(int(m.group(1)))
    m=re.search(r"dist = ([\d.]+)",line)
    if m: dist=m.group(1)
    m=re.search(r"ba = ([-\d.]+,[-\d.]+,[-\d.]+)",line)
    if m: ba=m.group(1)
nz=sum(1 for x in ms if x>0); pct=100*nz/len(ms) if ms else 0
row=[rep,"1.0",init_klt,f'"{init_ori}"',f'"{init_ba}"',f'"{init_vel}"',
     len(ms),nz,f"{pct:.1f}",sum(ms),sum(sl),dist,f'"{ba}"',log.split("/")[-1]]
open(out,'a').write(",".join(str(x) for x in row)+"\n")
print(f"  -> init@frame {init_klt} | ori {init_ori} | vel {init_vel}")
print(f"     MSCKF {nz}/{len(ms)} ({pct:.1f}%) {sum(ms)} feats | dist {dist} m | ba {ba}")
PY
done
echo "DONE -> $OUT"
