#!/usr/bin/env bash
# sweep_baro_sigma.sh [sigma ...] -- how loose should the barometer noise be?
#
#   ./sweep_baro_sigma.sh                 # sweeps 0.5 1.0 2.0 3.0
#   ./sweep_baro_sigma.sh 1.0 1.5         # only these
#   RVIZ=1 ./sweep_baro_sigma.sh 2.0      # watch it in rviz
#
# WHY THIS SWEEP
#
# baro_sigma = 0.5 m describes only the sensor's sample-to-sample noise (0.29 m
# measured). It ignores the barometer's 2.27% scale error, which is worth 2.4 m
# at 105 m, and it ignores prop wash. The chi2 gate boundary works out at
#
#     |res| > sqrt(chi2_multipler * 3.841 * S)     S ~= baro_sigma^2
#
# which at 0.5 m is only +-2.2 m. Mid-flight residuals reach 8 m, so the biggest
# disagreements -- the ones most worth correcting -- are the ones being thrown
# away. 31.6% of updates were rejected.
#
# Too large and the barometer stops constraining anything. Somewhere in between
# is the honest value. Nothing here needs a rebuild: baro_sigma is config.
set -u

SIGMAS=("$@"); [ ${#SIGMAS[@]} -eq 0 ] && SIGMAS=(0.5 1.0 2.0 3.0)
BAG=${BAG:-/home/vc/rosbags/field_data/seg01_ros2_baro}
SRC=${SRC:-/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_flight_A_baro_enabled}
export OV_WS=${OV_WS:-$HOME/openvins_baro_ws}
S=/home/vc/sensor_ws/src/sensor_bringup
OUT=${OUT:-/home/vc/rosbags/calib/tools/sweep_baro_sigma.csv}
TMP=/tmp/baro_sweep_cfg
RV=""; [ "${RVIZ:-0}" = "1" ] || RV="--no-rviz"

echo "sigma,klt,msckf_pct,msckf_feats,peak_alt,final_alt,max_descent,baro_peak,gap,bias_min,bias_max,accepted,rejected,rej_pct,res_mean,res_std" > "$OUT"

reap () {
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -KILL "$p" 2>/dev/null; done
  sleep 8
}

for SG in "${SIGMAS[@]}"; do
  echo "################ baro_sigma = $SG ################"
  reap
  rm -rf "$TMP"; cp -r "$SRC" "$TMP"
  # drop any existing key, then append. NOTE: no trailing comment -- a colon in a
  # trailing comment makes cv::FileStorage parse the value as EMPTY.
  sed -i '/^baro_sigma:/d' "$TMP/estimator_config.yaml"
  echo "baro_sigma: $SG" >> "$TMP/estimator_config.yaml"

  MARK=/tmp/sweep_mark.$$; touch "$MARK"
  O="/tmp/sweep_sigma_${SG}.out"
  "$S/scripts/run_openvins_bag.sh" "$BAG" --config "$TMP/estimator_config.yaml" $RV > "$O" 2>&1 &
  PID=$!; ok=0
  for _ in $(seq 1 400); do
    grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
    grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED -- see $O"; break; }
    kill -0 "$PID" 2>/dev/null || break
    sleep 4
  done
  sleep 25
  LOG="$(find /tmp/vio_logs -name 'openvins_bag_*.log' -newer "$MARK" -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
  rm -f "$MARK"
  kill -INT "$PID" 2>/dev/null; sleep 3; kill -KILL "$PID" 2>/dev/null; reap
  [ "$ok" -eq 1 ] && [ -n "$LOG" ] || { echo "$SG,,,,,,,,,,,,,,," >> "$OUT"; echo "  -> NO RUN"; continue; }

  python3 - "$SG" "$LOG" "$OUT" <<'PY'
import re,sys,numpy as np
sg,log,out=sys.argv[1:4]
txt=open(log,errors='ignore').read()
sg_seen=re.search(r"baro_sigma:\s*([\d.]+)",txt)
print(f"  config says baro_sigma = {sg_seen.group(1) if sg_seen else '??'}")
ms=[];P=[];klt=0
for line in txt.splitlines():
    if 'seconds for tracking' in line: klt+=1
    m=re.search(r"MSCKF update \((\d+) feats\)",line)
    if m: ms.append(int(m.group(1)))
    m=re.search(r"p_IinG = ([-\d.]+),([-\d.]+),([-\d.]+)",line)
    if m: P.append([float(x) for x in m.groups()])
ZB=[];ZV=[];B=[];R=[]
for m in re.finditer(r"\[BARO\]: t=[\d.]+ z_baro=\s*([-+\d.]+) z_vio=\s*([-+\d.]+) bias=\s*([-+\d.]+) res=\s*([-+\d.]+)",txt):
    a,b,c,d=[float(x) for x in m.groups()]; ZB.append(a);ZV.append(b);B.append(c);R.append(d)
rej=len(re.findall(r"\[BARO\]: rejected",txt))
if not P or not ms or not R:
    open(out,'a').write(f"{sg},{klt},,,,,,,,,,,,,,\n"); print("  -> NO METRICS"); raise SystemExit
P=np.array(P); z=P[:,2]-P[0,2]
w=max(2,len(z)//60); vz=np.diff(np.convolve(z,np.ones(w)/w,'same'))*29.9
ZB=np.array(ZB);ZV=np.array(ZV);B=np.array(B);R=np.array(R)
nz=sum(1 for x in ms if x>0); pct=100*nz/len(ms)
gap=ZV.max()-ZB.max(); rp=100*rej/(len(R)+rej)
open(out,'a').write(",".join(str(x) for x in
 [sg,klt,f"{pct:.1f}",sum(ms),f"{z.max():.1f}",f"{z[-1]:.1f}",f"{vz.min():.2f}",
  f"{ZB.max():.2f}",f"{gap:+.2f}",f"{B.min():+.3f}",f"{B.max():+.3f}",
  len(R),rej,f"{rp:.1f}",f"{R.mean():+.3f}",f"{R.std():.3f}"])+"\n")
print(f"  -> MSCKF {pct:.1f}%  peak {z.max():.1f} m  baro peak {ZB.max():.1f}  GAP {gap:+.2f} m")
print(f"     bias {B.min():+.3f}..{B.max():+.3f}   rejected {rp:.1f}%   res {R.mean():+.3f} +- {R.std():.3f}")
PY
done

echo
echo "======================= SWEEP ======================="
column -s, -t "$OUT"
echo
echo "gap  = VIO peak minus barometer peak. Closer to 0 is better,"
echo "       but the barometer's own scale error is ~2.4 m at 105 m,"
echo "       so anything inside +-2.4 m is at the limit of what we can resolve."
echo "MSCKF should stay in 64.2-72.3%. If it falls, the barometer is"
echo "       out-voting the camera and sigma is too small."
