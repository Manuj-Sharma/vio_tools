#!/usr/bin/env bash
# ingest_and_run.sh -- wait for a ROS1 bag upload, convert to ROS2, run OpenVINS.
#
#   ingest_and_run.sh [SRC_ROS1_BAG] [DST_ROS2_DIR]
#
# Steps, each with the trap that bit us previously noted inline:
#   1. wait for the ".filepart" suffix to disappear AND the size to stop growing
#   2. rosbags-convert ROS1 -> ROS2 sqlite3
#   3. FIX metadata.yaml -- rosbags-convert writes offered_qos_profiles as a YAML
#      LIST "[]", but Humble's rosbag2 expects a STRING. Left as-is,
#      "ros2 bag play" dies with "yaml-cpp: bad conversion" and OpenVINS sits
#      receiving nothing, which looks exactly like a broken filter.
#   4. verify readback (counts, rates, monotonicity, IMU brackets camera)
#   5. run OpenVINS with the static-init aerial config, --no-rviz
#      (RViz costs camera messages -- BEST_EFFORT QoS drops them under
#       contention, which changes the input and therefore the result)
set -u

SRC="${1:-/home/vc/rosbags/field_data/seg01.bag}"
DST="${2:-/home/vc/rosbags/field_data/seg01_ros2}"
CFG=/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_aerial_staticinit/estimator_config.yaml
S=/home/vc/sensor_ws/src/sensor_bringup
LOG=/home/vc/rosbags/calib/tools/ingest_run.log

say () { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }

# ---- 1. wait for the upload to settle -------------------------------------
say "waiting for upload to finish: $SRC"
last=-1; stable=0
for _ in $(seq 1 2400); do          # up to ~2 h
  if [ -f "$SRC" ]; then
    sz=$(stat -c %s "$SRC")
    if [ "$sz" = "$last" ]; then stable=$((stable+1)); else stable=0; fi
    last=$sz
    [ "$stable" -ge 4 ] && break     # unchanged for ~2 min
  else
    fp=$(ls "${SRC}.filepart" 2>/dev/null || true)
    [ -n "$fp" ] && say "  still copying: $(du -h "$fp" | cut -f1)"
  fi
  sleep 30
done
[ -f "$SRC" ] || { say "ERROR upload never completed"; exit 1; }
say "upload complete: $(du -h "$SRC" | cut -f1)"

# ---- 2. convert ------------------------------------------------------------
FREE=$(df --output=avail -k /home/vc | tail -1)
NEED=$(( $(stat -c %s "$SRC") / 1024 ))
say "convert -> $DST   (need ~$((NEED/1024/1024)) GB, have $((FREE/1024/1024)) GB)"
[ "$FREE" -lt "$((NEED + 2097152))" ] && { say "ERROR not enough disk"; exit 1; }
rm -rf "$DST"
rosbags-convert --src "$SRC" --dst "$DST" --dst-storage sqlite3 --dst-typestore ros2_humble >>"$LOG" 2>&1 \
  || { say "ERROR rosbags-convert failed"; exit 1; }

# ---- 3. the QoS metadata fix ----------------------------------------------
sed -i 's/offered_qos_profiles: \[\]/offered_qos_profiles: ""/' "$DST/metadata.yaml"
say "patched offered_qos_profiles [] -> \"\""

# ---- 4. verify -------------------------------------------------------------
say "verifying..."
python3 - "$DST" 2>&1 | tee -a "$LOG" <<'PY'
import sys, numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
TS=get_typestore(Stores.ROS2_HUMBLE)
ch=[];ih=[];a=[];meta=set()
with Reader(sys.argv[1]) as r:
    for c,ts,raw in r.messages():
        m=TS.deserialize_cdr(raw,c.msgtype)
        hs=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if c.topic=='/cam0/image_raw':
            ch.append(hs); meta.add((m.width,m.height,m.encoding))
        else:
            ih.append(hs); a.append((m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z))
ch=np.array(sorted(ch)); ih=np.array(sorted(ih)); a=np.array(a)
print("   image %s"%meta)
print("   cam %d @ %.2f Hz | imu %d @ %.1f Hz | dur %.1f s"%(
    len(ch),(len(ch)-1)/(ch[-1]-ch[0]),len(ih),(len(ih)-1)/(ih[-1]-ih[0]),ch[-1]-ch[0]))
print("   monotonic cam=%s imu=%s | imu brackets cam=%s"%(
    bool(np.all(np.diff(ch)>0)),bool(np.all(np.diff(ih)>0)),
    ih.min()<=ch.min() and ih.max()>=ch.max()))
print("   |accel| median %.3f (expect ~9.78)"%np.median(np.linalg.norm(a,axis=1)))
PY

# ---- 5. run ----------------------------------------------------------------
reap () {
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play") $(pgrep -f "[r]os2 launch sensor_bringup"); do
    kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "[r]os2 bag play") $(pgrep -f "[r]os2 launch sensor_bringup"); do
    kill -KILL "$p" 2>/dev/null; done
  sleep 3
}
say "running OpenVINS (static init, aerial gates, --no-rviz)"
reap
O=/tmp/ingest_run.out
"$S/scripts/run_openvins_bag.sh" "$DST" --config "$CFG" --no-rviz > "$O" 2>&1 &
PID=$!
for _ in $(seq 1 900); do
  grep -q "playback finished" "$O" 2>/dev/null && break
  grep -q "ERROR:" "$O" 2>/dev/null && { say "RUN REFUSED"; tail -3 "$O" | tee -a "$LOG"; break; }
  kill -0 "$PID" 2>/dev/null || break
  sleep 5
done
RLOG="$(readlink -f /tmp/vio_logs/openvins_bag_latest.log 2>/dev/null)"
kill -INT "$PID" 2>/dev/null; sleep 3; kill -KILL "$PID" 2>/dev/null; reap

say "RESULTS from $(basename "$RLOG")"
python3 - "$RLOG" 2>&1 | tee -a "$LOG" <<'PY'
import re, sys
L=sys.argv[1]; ms=[];sl=[];dist='';ba='';klt=0;ik=None;ori='';vel=''
for line in open(L,errors='ignore'):
    if 'seconds for tracking' in line: klt+=1
    if 'successful initialization' in line and ik is None: ik=klt
    m=re.search(r"\[init\]: orientation = (.+)",line)
    if m and not ori: ori=m.group(1).strip()
    m=re.search(r"\[init\]: velocity = (.+)",line)
    if m and not vel: vel=m.group(1).strip()
    m=re.search(r"MSCKF update \((\d+) feats\)",line)
    if m: ms.append(int(m.group(1)))
    m=re.search(r"SLAM update \((\d+) feats\)",line)
    if m: sl.append(int(m.group(1)))
    m=re.search(r"dist = ([\d.]+)",line)
    if m: dist=m.group(1)
    m=re.search(r"ba = ([-\d.]+,[-\d.]+,[-\d.]+)",line)
    if m: ba=m.group(1)
nz=sum(1 for x in ms if x>0)
print(f"   KLT frames {klt} | init@frame {ik} | ori {ori} | vel {vel}")
print(f"   MSCKF {nz}/{len(ms)} ({100*nz/max(len(ms),1):.1f}%) {sum(ms)} feats | SLAM {sum(sl)} feats")
print(f"   final dist {dist} m | ba {ba}")
PY
say "INGESTDONE"
