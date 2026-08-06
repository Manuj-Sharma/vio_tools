#!/usr/bin/env bash
# ab_baro_binary.sh -- does the uncommitted baro state change alter the filter
# when use_baro is OFF?
#
# The first attempt at this comparison was worthless because THREE variables
# moved at once: the binary (openvins_ws -> openvins_baro_ws), the bag
# (seg01_ros2 -> seg01_ros2_baro), and the harness. It reported 18.7% MSCKF
# against a 64.2-72.3% band and I nearly blamed the code.
#
# Here only the binary moves. Same run_openvins_bag.sh, same bag, same config,
# back to back on an idle box. run_openvins_bag.sh takes OV_WS from the
# environment (default ~/openvins_ws), which is the whole lever we need.
#
# The run is only trusted if the config fingerprint printed at startup matches
# ov_9281_flight_A. A percentage from an unverified config is what caused the
# false alarm, so this refuses to report one.
set -u

BAG=/home/vc/rosbags/field_data/seg01_ros2_baro
CFG=/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_flight_A/estimator_config.yaml
S=/home/vc/sensor_ws/src/sensor_bringup
OUT=/home/vc/rosbags/calib/tools/ab_baro_binary.csv

echo "ws,klt,msckf_pct,msckf_feats,slam_feats,peak_alt,final_alt,max_descent,ba,cfg_ok,log" > "$OUT"

reap () {   # bracketed patterns cannot match this script's own cmdline
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -KILL "$p" 2>/dev/null; done
  sleep 8
}

for WS in openvins_ws openvins_baro_ws; do
  echo "########## OV_WS=$WS  bag=$(basename $BAG) ##########"
  reap
  O="/tmp/ab_${WS}.out"
  OV_WS="$HOME/$WS" "$S/scripts/run_openvins_bag.sh" "$BAG" --config "$CFG" --no-rviz > "$O" 2>&1 &
  PID=$!; ok=0
  for _ in $(seq 1 400); do
    grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
    grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED -- see $O"; break; }
    kill -0 "$PID" 2>/dev/null || break
    sleep 4
  done
  LOG="$(readlink -f /tmp/vio_logs/openvins_bag_latest.log 2>/dev/null)"
  # the node is a subscriber: it idles forever after the bag ends, so we must
  # stop it ourselves. Elapsed time here means nothing about health.
  sleep 25                      # let the backlog drain before cutting it off
  kill -INT "$PID" 2>/dev/null; sleep 3; kill -KILL "$PID" 2>/dev/null; reap
  [ "$ok" -eq 1 ] && [ -n "$LOG" ] || { echo "$WS,,,,,,,,,,NORUN" >> "$OUT"; echo "  -> NO RUN"; continue; }
  cp "$LOG" "/tmp/ab_${WS}.log"

  python3 - "$WS" "$LOG" "$OUT" <<'PY'
import re,sys,numpy as np
ws,log,out=sys.argv[1:4]
ms=[];sl=[];P=[];ba='';klt=0
txt=open(log,errors='ignore').read()

# --- config fingerprint: refuse to report a number from the wrong config ----
def g(pat,default=None):
    m=re.search(pat,txt)
    return m.group(1) if m else default
want={'max_dist':'500','max_baseline':'500','max_cond_number':'200000',
      'init_dyn_use':'0','use_baro':'0'}
got={'max_dist':g(r"max_dist:\s*([\d.]+)"),
     'max_baseline':g(r"max_baseline:\s*([\d.]+)"),
     'max_cond_number':g(r"max_cond_number:\s*([\d.]+)"),
     'init_dyn_use':g(r"init_dyn_use:\s*(\d)"),
     'use_baro':g(r"use_baro:\s*(\d)")}
def num(x):
    try: return float(x)
    except: return None
cfg_ok=all(num(got[k]) is not None and num(got[k])==num(want[k]) for k in want if k!='use_baro')
print(f"  config fingerprint: {got}   -> {'OK' if cfg_ok else 'MISMATCH'}")

for line in txt.splitlines():
    if 'seconds for tracking' in line: klt+=1
    m=re.search(r"MSCKF update \((\d+) feats\)",line)
    if m: ms.append(int(m.group(1)))
    m=re.search(r"SLAM update \((\d+) feats\)",line)
    if m: sl.append(int(m.group(1)))
    m=re.search(r"p_IinG = ([-\d.]+),([-\d.]+),([-\d.]+)",line)
    if m: P.append([float(x) for x in m.groups()])
    m=re.search(r"ba = ([-\d.]+,[-\d.]+,[-\d.]+)",line)
    if m: ba=m.group(1)

if not P or not ms:
    open(out,'a').write(f"{ws},{klt},,,,,,,,{cfg_ok},{log.split('/')[-1]}\n")
    print(f"  -> NO METRICS (klt={klt})"); raise SystemExit
P=np.array(P); z=P[:,2]-P[0,2]
w=max(2,len(z)//60); vz=np.diff(np.convolve(z,np.ones(w)/w,'same'))*29.9
nz=sum(1 for x in ms if x>0); pct=100*nz/len(ms)
open(out,'a').write(",".join(str(x) for x in
  [ws,klt,f"{pct:.1f}",sum(ms),sum(sl),f"{z.max():.1f}",f"{z[-1]:.1f}",
   f"{vz.min():.2f}",f'"{ba}"',cfg_ok,log.split("/")[-1]])+"\n")
print(f"  -> MSCKF {pct:.1f}% ({nz}/{len(ms)})  {sum(ms)} feats | KLT {klt} | "
      f"peak {z.max():+.1f} final {z[-1]:+.1f} m | descent {vz.min():.1f} m/s | ba {ba}")
PY
done
echo "DONE -> $OUT"
column -s, -t "$OUT"
