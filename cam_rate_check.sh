#!/usr/bin/env bash
#
# cam_rate_check.sh - measure the REAL camera (and IMU) rates on a running stack, without `ros2 topic hz`.
#
# `ros2 topic hz` is a single Python subscriber and cannot keep up with two 1280x720 streams: it read 20-27 Hz
# on cameras that delivered ~52 Hz (2026-11-02). This records /cam0/image_raw, /cam1/image_raw (+ /imu0) with the
# C++ rosbag2 recorder for DURATION seconds, then reports for each topic:
#   header-stamp  : rate and gaps of the frame timestamps the driver wrote (host CLOCK_MONOTONIC at publish).
#                   This is the camera/driver itself -- the number to trust for "is the camera stalling".
#   receive-time  : rate and gaps as seen by the recorder. Gaps here that are NOT in the header-stamp column are
#                   delivery bursts (DDS / CPU load), not the camera.
# Plus kernel capture-vi timeouts during the window. The bag is deleted afterwards (KEEP=1 keeps it).
#
# Usage:   ~/vio_tools/cam_rate_check.sh              # 15 s
#          DURATION=60 ~/vio_tools/cam_rate_check.sh
#          IMU=0 ~/vio_tools/cam_rate_check.sh        # cameras only
#          EXPECT_HZ=60 ~/vio_tools/cam_rate_check.sh # flag rates below 90% of this (default 60)
# The sensors must be running. Needs ~1.1 GB free per 10 s of recording.

set -u
REPO="${REPO:-$HOME/vio_ar0234_icm20948}"
DURATION="${DURATION:-15}"
IMU="${IMU:-1}"
EXPECT_HZ="${EXPECT_HZ:-60}"
OUT="${OUT:-/tmp/vio_logs/cam_rate_check_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$OUT")"

set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

pgrep -x vizionsdk_camer >/dev/null || { echo "ERROR: no camera node running -- start the sensors first" >&2; exit 1; }
free=$(df -BG --output=avail "$(dirname "$OUT")" | tail -1 | tr -dc 0-9)
need=$(( DURATION * 120 / 1000 + 1 ))
[ "${free:-0}" -ge "$need" ] || { echo "ERROR: need ~${need} GB free, have ${free} GB" >&2; exit 1; }

TOPICS=(/cam0/image_raw /cam1/image_raw)
[ "$IMU" = 1 ] && TOPICS+=(/imu0)

echo "[cam_rate_check] recording ${TOPICS[*]} for ${DURATION}s ..."
T0=$(date +%s)
timeout -s INT $((DURATION + 3)) ros2 bag record -o "$OUT" "${TOPICS[@]}" > "$OUT.rec.log" 2>&1 &
RP=$!
sleep "$DURATION"
kill -INT "$RP" 2>/dev/null
wait "$RP" 2>/dev/null
sleep 1

python3 - "$OUT" "$EXPECT_HZ" <<'PY'
import sqlite3, glob, sys
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu

bag, expect = sys.argv[1], float(sys.argv[2])
dbs = glob.glob(bag + "/*.db3")
if not dbs:
    print("NO bag was written -- see %s.rec.log" % bag); sys.exit(1)
c = sqlite3.connect("file:" + dbs[0] + "?immutable=1", uri=True)
msgtype = {"/cam0/image_raw": Image, "/cam1/image_raw": Image, "/imu0": Imu}
print("%-16s %5s  %-34s  %-34s" % ("topic", "msgs", "HEADER STAMP (the camera)", "RECEIVE TIME (recorder)"))
print("%-16s %5s  %-34s  %-34s" % ("", "", "Hz   worst  >40ms >100ms", "Hz   worst  >40ms >100ms"))
bad = 0
for tid, name in c.execute("select id,name from topics order by name"):
    cls = msgtype.get(name)
    if cls is None: continue
    rx, hd = [], []
    for t, data in c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tid,)):
        m = deserialize_message(data, cls)
        rx.append(t / 1e9); hd.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    if len(rx) < 3:
        print("%-16s %5d  (too few messages)" % (name, len(rx))); bad = 1; continue
    rx, hd = np.array(rx), np.array(hd)
    def stats(a):
        d = np.diff(a); span = a[-1] - a[0]
        return (len(a) - 1) / span, d.max() * 1000, int((d > 0.04).sum()), int((d > 0.1).sum())
    h, r = stats(hd), stats(rx)
    flag = ""
    want = expect if name.startswith("/cam") else None
    if want and h[0] < 0.9 * want: flag = "  <-- LOW (header-stamp rate)"; bad = 1
    if h[3] > 0: flag += "  <-- GAP > 100 ms in the camera stamps"; bad = 1
    print("%-16s %5d  %5.1f %5.0f ms %4d %5d      %5.1f %5.0f ms %4d %5d%s" % ((name, len(rx)) + h + r + (flag,)))
print()
print("Trust the HEADER STAMP column for camera health. RECEIVE-TIME gaps that the header stamps do not show are")
print("delivery bursts (DDS / CPU load), not camera stalls. Expected camera rate: %.0f Hz." % expect)
sys.exit(bad)
PY
RC=$?
echo "kernel capture-vi timeouts in the window: $(journalctl -k --since "@$T0" 2>/dev/null | grep -c uncorr_err)"
[ "${KEEP:-0}" = 1 ] && echo "bag kept: $OUT" || rm -rf "$OUT" "$OUT.rec.log"
exit $RC
