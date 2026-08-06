#!/usr/bin/env bash
# repeat_A.sh -- is ov_9281_flight_A actually reproducible?
#
# ov_9281_flight_imu_meas gave 69.1 / 66.5 / 15.5 percent MSCKF usage on three
# identical runs, so high-noise configs on this bag are fragile. A scored 68.7%
# on a single draw. Before writing a barometer updater we need to know whether
# any improvement it brings would even be visible above this variance.
#
# Reports the spread, not just the values.
set -u
BAG=/home/vc/rosbags/field_data/seg01_ros2
CFG=/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_flight_A/estimator_config.yaml
S=/home/vc/sensor_ws/src/sensor_bringup
OUT=/home/vc/rosbags/calib/tools/repeat_A.csv
[ -f "$OUT" ] || echo "rep,klt,msckf_pct,msckf_feats,slam_feats,peak_alt,final_alt,max_descent,ba,log" > "$OUT"
reap () {   # patterns whose own cmdline cannot match the runner's guard
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -KILL "$p" 2>/dev/null; done
  sleep 8
}
for rep in 1 2 3; do
  echo "########## ov_9281_flight_A  rep $rep/3 ##########"
  reap
  O="/tmp/repA_$rep.out"
  "$S/scripts/run_openvins_bag.sh" "$BAG" --config "$CFG" --no-rviz > "$O" 2>&1 &
  PID=$!; ok=0
  for _ in $(seq 1 400); do
    grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
    grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED"; break; }
    kill -0 "$PID" 2>/dev/null || break
    sleep 4
  done
  LOG="$(readlink -f /tmp/vio_logs/openvins_bag_latest.log 2>/dev/null)"
  kill -INT "$PID" 2>/dev/null; sleep 3; kill -KILL "$PID" 2>/dev/null; reap
  [ "$ok" -eq 1 ] && [ -n "$LOG" ] || { echo "$rep,,,,,,,,,NORUN" >> "$OUT"; echo "  -> NO RUN"; continue; }
  python3 - "$rep" "$LOG" "$OUT" <<'PY'
import re,sys,numpy as np
rep,log,out=sys.argv[1:4]
ms=[];sl=[];P=[];ba='';klt=0
for line in open(log,errors='ignore'):
    if 'seconds for tracking' in line: klt+=1
    m=re.search(r"MSCKF update \((\d+) feats\)",line)
    if m: ms.append(int(m.group(1)))
    m=re.search(r"SLAM update \((\d+) feats\)",line)
    if m: sl.append(int(m.group(1)))
    m=re.search(r"p_IinG = ([-\d.]+),([-\d.]+),([-\d.]+)",line)
    if m: P.append([float(x) for x in m.groups()])
    m=re.search(r"ba = ([-\d.]+,[-\d.]+,[-\d.]+)",line)
    if m: ba=m.group(1)
P=np.array(P); z=P[:,2]-P[0,2]
w=max(2,len(z)//60); vz=np.diff(np.convolve(z,np.ones(w)/w,'same'))*29.9
nz=sum(1 for x in ms if x>0); pct=100*nz/len(ms) if ms else 0
open(out,'a').write(",".join(str(x) for x in
  [rep,klt,f"{pct:.1f}",sum(ms),sum(sl),f"{z.max():.1f}",f"{z[-1]:.1f}",f"{vz.min():.2f}",f'"{ba}"',log.split("/")[-1]])+"\n")
print(f"  -> MSCKF {pct:.1f}% {sum(ms)} feats | peak {z.max():+.1f} final {z[-1]:+.1f} m | descent {vz.min():.1f} m/s")
PY
done
echo "DONE -> $OUT"
