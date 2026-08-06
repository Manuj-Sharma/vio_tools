#!/usr/bin/env bash
# timeshift_sweep.sh -- find the cam-IMU time offset that actually works, by test.
#
# WHY. Cross-correlating image rotation against gyro-z on this bag gave a very
# strong peak (0.997) at a magnitude of 8.4 ms, but the mapping of that lag into
# Kalibr's convention (t_imu = t_cam + shift) was not reliably derived. Testing
# -0.0084 collapsed feature usage from 25.0% to 1.2% and drift from 3.2 km to
# 122 km, which says the sign was flipped. Rather than argue about conventions,
# sweep it and let the drift curve pick the answer.
#
# Everything except timeshift_cam_imu is identical across the five configs, and
# ov_9281_aerial_staticinit (the gold standard) is not touched.
#
# Reproducibility note: static init makes runs repeatable to ~1.9 points of
# MSCKF usage, so differences smaller than that are not meaningful.
set -u

BAG=/home/vc/rosbags/field_data/seg01_ros2
S=/home/vc/sensor_ws/src/sensor_bringup
OUT=/home/vc/rosbags/calib/tools/timeshift_sweep.csv
CONFIGS="ov_9281_ts_m84 ov_9281_ts_m44 ov_9281_ts_z00 ov_9281_ts_p44 ov_9281_ts_p84"

[ -f "$OUT" ] || echo "config,timeshift_ms,klt,init_frame,msckf_upd,msckf_nonzero,msckf_pct,msckf_feats,slam_feats,final_dist_m,ba,log" > "$OUT"

reap () {   # bracketed patterns so this can never match the harness itself.
            # Also kill the ros2 launch parent -- a suspended parent leaves an
            # unreapable zombie that still trips the runner's duplicate guard.
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play") \
           $(pgrep -f "[r]os2 launch sensor_bringup") $(pgrep -x rviz2); do
    kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play") \
           $(pgrep -f "[r]os2 launch sensor_bringup") $(pgrep -x rviz2); do
    kill -KILL "$p" 2>/dev/null; done
  sleep 3
}

for cfg in $CONFIGS; do
  TS=$(grep -E '^  timeshift_cam_imu:' "$S/config/$cfg/kalibr_imucam_chain.yaml" | awk '{print $2}')
  echo "########## $cfg   timeshift $TS s ##########"
  reap
  O="/tmp/sweep_$cfg.out"
  "$S/scripts/run_openvins_bag.sh" "$BAG" --config "$S/config/$cfg/estimator_config.yaml" \
      --no-rviz > "$O" 2>&1 &
  PID=$!; ok=0
  for _ in $(seq 1 400); do
    grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
    grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED"; tail -3 "$O" | sed 's/^/    /'; break; }
    kill -0 "$PID" 2>/dev/null || break
    sleep 4
  done
  LOG="$(readlink -f /tmp/vio_logs/openvins_bag_latest.log 2>/dev/null)"
  kill -INT "$PID" 2>/dev/null; sleep 3; kill -KILL "$PID" 2>/dev/null
  reap
  [ "$ok" -eq 1 ] && [ -n "$LOG" ] || { echo "$cfg,$TS,,,,,,,,,,NORUN" >> "$OUT"; echo "  -> NO RUN"; continue; }
  python3 - "$cfg" "$TS" "$LOG" "$OUT" <<'PY'
import re, sys
cfg, ts, log, out = sys.argv[1:5]
ms=[]; sl=[]; dist=''; ba=''; klt=0; ik=None
for line in open(log, errors='ignore'):
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
open(out,'a').write(",".join(str(x) for x in
    [cfg,float(ts)*1000,klt,ik,len(ms),nz,f"{pct:.1f}",sum(ms),sum(sl),dist,f'"{ba}"',log.split("/")[-1]])+"\n")
print(f"  -> KLT {klt} init@{ik} | MSCKF {nz}/{len(ms)} ({pct:.1f}%) {sum(ms)} feats | dist {dist} m")
PY
done
echo "SWEEPDONE -> $OUT"
