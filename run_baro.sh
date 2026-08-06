#!/usr/bin/env bash
# run_baro.sh [REPS] -- run the openvins_baro_ws build on the merged baro bag.
#
#   ./run_baro.sh        # one run
#   ./run_baro.sh 3      # three runs, appended to the same CSV
#
# Override any of these from the environment if needed:
#   BAG=... CFG=... OV_WS=... ./run_baro.sh
#
# Two things here exist because they bit us:
#
# 1. The run is REAPED first. A stale run_subscribe_msckf left over from an
#    earlier run is invisible (it idles forever -- it is a subscriber, it never
#    exits on its own) but it makes /tmp/vio_logs/openvins_bag_latest.log point
#    at the wrong file. That is how a healthy 66.7% run got read as 18.7% and
#    nearly got the code blamed.
#
# 2. The log is picked by MTIME, not by the `latest` symlink, and the config
#    fingerprint printed at startup is checked before any percentage is
#    reported. A number from an unverified config is worse than no number.
set -u

REPS="${1:-1}"
BAG="${BAG:-/home/vc/rosbags/field_data/seg01_ros2_baro}"
CFG="${CFG:-/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_flight_A/estimator_config.yaml}"
export OV_WS="${OV_WS:-$HOME/openvins_baro_ws}"
S=/home/vc/sensor_ws/src/sensor_bringup
OUT="${OUT:-/home/vc/rosbags/calib/tools/run_baro.csv}"

[ -f "$BAG/metadata.yaml" ] || { echo "no bag at $BAG" >&2; exit 1; }
[ -f "$CFG" ]               || { echo "no config at $CFG" >&2; exit 1; }
[ -f "$OV_WS/install/setup.bash" ] || { echo "not built: $OV_WS" >&2; exit 1; }

[ -f "$OUT" ] || echo "rep,klt,msckf_pct,msckf_feats,slam_feats,peak_alt,final_alt,max_descent,ba,cfg_ok,log" > "$OUT"

reap () {   # bracketed patterns cannot match this script's own cmdline
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -KILL "$p" 2>/dev/null; done
  sleep 8
}

echo "OV_WS  : $OV_WS"
echo "bag    : $BAG"
echo "config : $CFG"
echo "reps   : $REPS   -> $OUT"

for rep in $(seq 1 "$REPS"); do
  echo "########## rep $rep/$REPS ##########"
  reap
  MARK=/tmp/run_baro_mark.$$ ; touch "$MARK"      # only logs newer than this count
  O="/tmp/run_baro_${rep}.out"
  "$S/scripts/run_openvins_bag.sh" "$BAG" --config "$CFG" --no-rviz > "$O" 2>&1 &
  PID=$!; ok=0
  for _ in $(seq 1 400); do
    grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
    grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED -- see $O"; break; }
    kill -0 "$PID" 2>/dev/null || break
    sleep 4
  done
  sleep 25                                        # let the backlog drain
  # newest log created after MARK -- never trust the `latest` symlink
  LOG="$(find /tmp/vio_logs -name 'openvins_bag_*.log' -newer "$MARK" -printf '%T@ %p\n' 2>/dev/null \
         | sort -rn | head -1 | cut -d' ' -f2-)"
  rm -f "$MARK"
  kill -INT "$PID" 2>/dev/null; sleep 3; kill -KILL "$PID" 2>/dev/null; reap
  [ "$ok" -eq 1 ] && [ -n "$LOG" ] || { echo "$rep,,,,,,,,,,NORUN" >> "$OUT"; echo "  -> NO RUN"; continue; }
  echo "  log: $LOG"

  python3 - "$rep" "$LOG" "$OUT" <<'PY'
import re,sys,numpy as np
rep,log,out=sys.argv[1:4]
txt=open(log,errors='ignore').read()
def g(p):
    m=re.search(p,txt); return m.group(1) if m else None
got={'max_dist':g(r"max_dist:\s*([\d.]+)"),'max_baseline':g(r"max_baseline:\s*([\d.]+)"),
     'max_cond_number':g(r"max_cond_number:\s*([\d.]+)"),'init_dyn_use':g(r"init_dyn_use:\s*(\d)"),
     'use_baro':g(r"use_baro:\s*(\d)")}
want={'max_dist':500.,'max_baseline':500.,'max_cond_number':200000.,'init_dyn_use':0.}
def num(x):
    try: return float(x)
    except: return None
cfg_ok=all(num(got[k])==v for k,v in want.items())
print(f"  config: {got}  -> {'OK' if cfg_ok else '*** MISMATCH -- number not trustworthy ***'}")

ms=[];sl=[];P=[];ba='';klt=0
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
    open(out,'a').write(f"{rep},{klt},,,,,,,,{cfg_ok},{log.split('/')[-1]}\n")
    print(f"  -> NO METRICS (klt={klt})"); raise SystemExit
P=np.array(P); z=P[:,2]-P[0,2]
w=max(2,len(z)//60); vz=np.diff(np.convolve(z,np.ones(w)/w,'same'))*29.9
nz=sum(1 for x in ms if x>0); pct=100*nz/len(ms)
open(out,'a').write(",".join(str(x) for x in
  [rep,klt,f"{pct:.1f}",sum(ms),sum(sl),f"{z.max():.1f}",f"{z[-1]:.1f}",
   f"{vz.min():.2f}",f'"{ba}"',cfg_ok,log.split("/")[-1]])+"\n")
print(f"  -> MSCKF {pct:.1f}% ({nz}/{len(ms)})  {sum(ms)} feats | KLT {klt} | "
      f"peak {z.max():+.1f} final {z[-1]:+.1f} m | descent {vz.min():.1f} m/s")
print(f"     ba = {ba}")
PY
done

echo
echo "===== all runs so far ====="
column -s, -t "$OUT"
echo
echo "reference: baseline openvins_ws on this same bag = 69.9%   band (5 runs) = 64.2-72.3%"
