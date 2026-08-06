#!/usr/bin/env bash
# ratio_sweep.sh -- find an accel/gyro noise ratio that keeps features AND altitude.
#
# imu_meas (accel 1.5e-1, gyro 6e-3) doubled feature usage and removed the
# coasting leg, but broke the vertical channel: altitude descended through zero
# to -105 m at 12.75 m/s. A nadir camera sees no parallax from vertical motion,
# so altitude is double-integrated accelerometer -- distrusting accel by 126x
# removes the only sensor observing z. The gyro must stay at its measured value
# because attitude error leaks gravity into z.
#
# Ground truth from the operator: actual ceiling was BELOW 80 m, and altitude
# must return to ~0 at landing. Both are checked here.
set -u
BAG=/home/vc/rosbags/field_data/seg01_ros2
S=/home/vc/sensor_ws/src/sensor_bringup
OUT=/home/vc/rosbags/calib/tools/ratio_sweep.csv
CONFIGS="ov_9281_flight_A ov_9281_flight_B ov_9281_flight_imu_meas ov_9281_aerial_staticinit"
[ -f "$OUT" ] || echo "config,acc_nd,gyr_nd,klt,msckf_pct,msckf_feats,slam_feats,peak_alt_m,final_alt_m,max_descent_ms,horiz_max_m,ba,log" > "$OUT"
reap () {
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "launch[ ]sensor_bringup") $(pgrep -x rviz2); do kill -KILL "$p" 2>/dev/null; done
  # settle -- a transient pgrep whose OWN cmdline contains the guard pattern
  # will otherwise be seen by run_openvins_bag.sh and the run refused
  sleep 8
}
for cfg in $CONFIGS; do
  AND=$(grep -E '^  accelerometer_noise_density:' "$S/config/$cfg/kalibr_imu_chain.yaml"|awk '{print $2}')
  GND=$(grep -E '^  gyroscope_noise_density:' "$S/config/$cfg/kalibr_imu_chain.yaml"|awk '{print $2}')
  echo "########## $cfg  acc $AND  gyr $GND ##########"
  reap
  O="/tmp/rsw_$cfg.out"
  "$S/scripts/run_openvins_bag.sh" "$BAG" --config "$S/config/$cfg/estimator_config.yaml" --no-rviz > "$O" 2>&1 &
  PID=$!; ok=0
  for _ in $(seq 1 400); do
    grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
    grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED"; break; }
    kill -0 "$PID" 2>/dev/null || break
    sleep 4
  done
  LOG="$(readlink -f /tmp/vio_logs/openvins_bag_latest.log 2>/dev/null)"
  kill -INT "$PID" 2>/dev/null; sleep 3; kill -KILL "$PID" 2>/dev/null; reap
  [ "$ok" -eq 1 ] && [ -n "$LOG" ] || { echo "$cfg,$AND,$GND,,,,,,,,,,NORUN" >> "$OUT"; echo "  -> NO RUN"; continue; }
  python3 - "$cfg" "$AND" "$GND" "$LOG" "$OUT" <<'PY'
import re,sys,numpy as np
cfg,and_,gnd,log,out=sys.argv[1:6]
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
P=np.array(P); z=P[:,2]-P[0,2]; xy=np.linalg.norm(P[:,:2]-P[0,:2],axis=1)
w=max(2,len(z)//60); vz=np.diff(np.convolve(z,np.ones(w)/w,'same'))*29.9
nz=sum(1 for x in ms if x>0); pct=100*nz/len(ms) if ms else 0
row=[cfg,and_,gnd,klt,f"{pct:.1f}",sum(ms),sum(sl),f"{z.max():.1f}",f"{z[-1]:.1f}",
     f"{vz.min():.2f}",f"{xy.max():.0f}",f'"{ba}"',log.split("/")[-1]]
open(out,'a').write(",".join(str(x) for x in row)+"\n")
print(f"  -> MSCKF {pct:.1f}% {sum(ms)} feats | peak alt {z.max():+.1f} m (truth <80) | "
      f"final alt {z[-1]:+.1f} m (truth ~0) | max descent {vz.min():.1f} m/s | ba {ba}")
PY
done
echo "SWEEPDONE -> $OUT"
