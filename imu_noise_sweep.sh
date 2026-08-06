#!/usr/bin/env bash
# imu_noise_sweep.sh -- does raising the IMU noise model stop the coasting leg?
#
# The gold-standard noise values (1.19e-3 / 1.67e-4) came from the cam-IMU
# calibration bag: handheld, bench, motors OFF. Measured spectrally on the
# flight bag they are 66-246x too small on accel (worst on z, the axis motor
# vibration loads). The filter therefore integrates vibration as real
# acceleration, velocity inflates to ~20 m/s, predicted motion stops matching
# the images, no new SLAM feature can be admitted, and the leg coasts.
#
# Baseline to beat, gold standard on this bag: 25.0% MSCKF, 3228 m drift.
# ov_9281_aerial_staticinit is NOT modified by this script.
set -u
BAG=/home/vc/rosbags/field_data/seg01_ros2
S=/home/vc/sensor_ws/src/sensor_bringup
OUT=/home/vc/rosbags/calib/tools/imu_noise_sweep.csv
CONFIGS="ov_9281_aerial_staticinit ov_9281_flight_imu10x ov_9281_flight_imu100x ov_9281_flight_imu_meas"
[ -f "$OUT" ] || echo "config,acc_nd,gyr_nd,klt,init_frame,msckf_upd,msckf_nonzero,msckf_pct,msckf_feats,slam_feats,final_dist_m,ba,log" > "$OUT"
reap () {
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play") \
           $(pgrep -f "[r]os2 launch sensor_bringup") $(pgrep -x rviz2); do kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play") \
           $(pgrep -f "[r]os2 launch sensor_bringup") $(pgrep -x rviz2); do kill -KILL "$p" 2>/dev/null; done
  sleep 3
}
for cfg in $CONFIGS; do
  AND=$(grep -E '^  accelerometer_noise_density:' "$S/config/$cfg/kalibr_imu_chain.yaml"|awk '{print $2}')
  GND=$(grep -E '^  gyroscope_noise_density:' "$S/config/$cfg/kalibr_imu_chain.yaml"|awk '{print $2}')
  echo "########## $cfg   acc $AND  gyr $GND ##########"
  reap
  O="/tmp/imusw_$cfg.out"
  "$S/scripts/run_openvins_bag.sh" "$BAG" --config "$S/config/$cfg/estimator_config.yaml" --no-rviz > "$O" 2>&1 &
  PID=$!; ok=0
  for _ in $(seq 1 400); do
    grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
    grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED"; tail -3 "$O"|sed 's/^/    /'; break; }
    kill -0 "$PID" 2>/dev/null || break
    sleep 4
  done
  LOG="$(readlink -f /tmp/vio_logs/openvins_bag_latest.log 2>/dev/null)"
  kill -INT "$PID" 2>/dev/null; sleep 3; kill -KILL "$PID" 2>/dev/null; reap
  [ "$ok" -eq 1 ] && [ -n "$LOG" ] || { echo "$cfg,$AND,$GND,,,,,,,,,,NORUN" >> "$OUT"; echo "  -> NO RUN"; continue; }
  python3 - "$cfg" "$AND" "$GND" "$LOG" "$OUT" <<'PY'
import re,sys
cfg,and_,gnd,log,out=sys.argv[1:6]
ms=[];sl=[];dist='';ba='';klt=0;ik=None
for line in open(log,errors='ignore'):
    if 'seconds for tracking' in line: klt+=1
    if 'successful initialization' in line and ik is None: ik=klt
    m=re.search(r"MSCKF update \((\d+) feats\)",line)
    if m: ms.append(int(m.group(1)))
    m=re.search(r"SLAM update \((\d+) feats\)",line)
    if m: sl.append(int(m.group(1)))
    m=re.search(r"dist = ([\d.]+)",line)
    if m: dist=m.group(1)
    m=re.search(r"ba = ([-\d.]+,[-\d.]+,[-\d.]+)",line)
    if m: ba=m.group(1)
nz=sum(1 for x in ms if x>0); pct=100*nz/len(ms) if ms else 0
open(out,'a').write(",".join(str(x) for x in [cfg,and_,gnd,klt,ik,len(ms),nz,f"{pct:.1f}",sum(ms),sum(sl),dist,f'"{ba}"',log.split("/")[-1]])+"\n")
print(f"  -> KLT {klt} init@{ik} | MSCKF {nz}/{len(ms)} ({pct:.1f}%) {sum(ms)} feats | SLAM {sum(sl)} | dist {dist} m | ba {ba}")
PY
done
echo "SWEEPDONE -> $OUT"
