#!/usr/bin/env bash
#
# cam_trigger_open_test.sh - test whether opening BOTH cameras directly in trigger mode (PWM already running)
# removes the ~3 s cam1 stall that the normal start-up sequence causes.
#
# Normal sequence (run_vio_stack.sh / sensors_and_record.sh): nodes open the cameras free-running
# (isp.trigger_mode=0), cam0 is switched to trigger mode, the 60 Hz PWM starts, and trigger_cam.sh switches
# cam1 to trigger mode 10 s later while it is streaming -> cam1 stalls ~3 s (worst gap 2.9 s, capture-vi timeout).
# This test: PWM first, then both camera nodes started with isp.trigger_mode=2, NO later mode switch.
#
# Per run: stop everything -> PWM off -> PWM on -> recorder (/cam0 + /cam1 only, C++ rosbag2) -> both camera
# nodes -> wait DURATION -> stop -> analyse the bag (receive-time gaps) + kernel capture-vi timeouts + cam1
# VxGetImage failures. The bag is deleted after the analysis (KEEP=1 to keep it).
#
# Usage:   ~/vio_tools/cam_trigger_open_test.sh                  # 3 runs x 45 s, trigger mode 2 (the test)
#          RUNS=1 DURATION=90 ~/vio_tools/cam_trigger_open_test.sh
#          TRIGGER_MODE=0 ~/vio_tools/cam_trigger_open_test.sh   # both cams free-running, PWM off (control)
# Needs ~5 GB free per 45 s of recording. Run it with the sensors stopped (it stops them itself).
# Result: /tmp/vio_logs/cam_trigger_open_test_<time>/summary.txt (also printed).
#
# NOTE: it uses the same `echo vc | sudo -S` the repo's trigger_cam.sh uses, to drive the PWM.

set -u
REPO="${REPO:-$HOME/vio_ar0234_icm20948}"
RUNS="${RUNS:-3}"
DURATION="${DURATION:-45}"
TRIGGER_MODE="${TRIGGER_MODE:-2}"
TRIGGER_HZ="${TRIGGER_HZ:-60}"
OUT="${OUT:-/tmp/vio_logs/cam_trigger_open_test_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
SUMMARY="$OUT/summary.txt"

set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

say() { echo "$*" | tee -a "$SUMMARY"; }
PIDS=()
stop_all() {
  for p in "${PIDS[@]:-}"; do [ -n "${p:-}" ] && kill -INT "$p" 2>/dev/null; done
  sleep 3
  pkill vizionsdk_camer 2>/dev/null
  pkill -f "ros2 run vizionsdk_ros2" 2>/dev/null
  for p in "${PIDS[@]:-}"; do [ -n "${p:-}" ] && kill -KILL "$p" 2>/dev/null; done
  PIDS=()
}
trap 'stop_all; echo; echo "[cam_trigger_open_test] interrupted"; exit 130' INT TERM

say "== cam_trigger_open_test $(date +%T)  RUNS=$RUNS DURATION=${DURATION}s TRIGGER_MODE=$TRIGGER_MODE TRIGGER_HZ=$TRIGGER_HZ"
free=$(df -BG --output=avail "$OUT" | tail -1 | tr -dc 0-9)
need=$(( RUNS > 0 ? (DURATION * 110 / 1000) + 2 : 0 ))
[ "${free:-0}" -ge "$need" ] || { say "ERROR: need ~${need} GB free, have ${free} GB"; exit 1; }
echo "vc" | sudo -S -v >/dev/null 2>&1 || say "WARNING: could not prime sudo; the PWM step may fail"

# Stop whatever sensors are running (the documented way, then by name).
"$REPO/record_tool/sensors_and_record.sh" stop >/dev/null 2>&1
pkill run_subscribe_m 2>/dev/null; pkill rviz2 2>/dev/null; pkill imx5_spi_node 2>/dev/null
pkill -f trigger_cam.sh 2>/dev/null; pkill vizionsdk_camer 2>/dev/null
sleep 3

camera_node() {   # camera_node <cam 0|1>
  local n="$1" pkg exe node
  if [ "$n" = 0 ]; then pkg=vizionsdk_ros2; exe=vizionsdk_camera_node; node=vizionsdk_camera
  else                  pkg=vizionsdk_ros2_cam1; exe=vizionsdk_camera_node_cam1; node=vizionsdk_camera_cam1; fi
  ros2 run "$pkg" "$exe" --ros-args -r __node:="$node" \
    -p publish_image:=true -p publish_imu:=false -p publish_status:=false \
    -p image_format:=UYVY -p image_width:=1280 -p image_height:=720 -p grayscale:=true \
    -p image_rate_hz:=0.0 -p clock_source:=monotonic -p image_frame_id:=cam"$n" \
    -p isp.trigger_mode:="$TRIGGER_MODE" -p isp.exposure_min_time:=1000 -p isp.exposure_max_time:=8000 \
    -r image_raw:=/cam"$n"/image_raw -r camera_info:=/cam"$n"/camera_info
}

for run in $(seq 1 "$RUNS"); do
  R="$OUT/run$run"; mkdir -p "$R"; BAG="$R/bag"
  say ""; say "---- run $run/$RUNS  $(date +%T)"
  stop_all
  ( cd "$REPO" && echo "vc" | sudo -S ./trigger.sh off ) >"$R/pwm_off.log" 2>&1
  sleep 2
  if [ "$TRIGGER_MODE" = 2 ]; then
    ( cd "$REPO" && echo "vc" | sudo -S ./trigger.sh "$TRIGGER_HZ" ) >"$R/pwm_on.log" 2>&1
    say "PWM: $(grep -o 'trigger ON.*' "$R/pwm_on.log" | head -1)"
    sleep 2
  fi
  T0_NS=$(date +%s%N)
  ( trap - INT QUIT; exec ros2 bag record -o "$BAG" /cam0/image_raw /cam1/image_raw ) >"$R/recorder.log" 2>&1 &
  PIDS+=($!)
  sleep 2
  ( trap - INT QUIT; camera_node 0 ) >"$R/cam0_node.log" 2>&1 &
  PIDS+=($!)
  ( trap - INT QUIT; camera_node 1 ) >"$R/cam1_node.log" 2>&1 &
  PIDS+=($!)
  NODES_START=$(date +%T)
  sleep "$DURATION"
  stop_all
  sleep 2
  say "nodes started $NODES_START, ran ${DURATION}s"
  say "kernel capture-vi timeouts in window: $(journalctl -k --since "@$((T0_NS/1000000000))" 2>/dev/null | grep -c 'uncorr_err')"
  say "VxGetImage failed  cam0: $(grep -c 'VxGetImage failed' "$R/cam0_node.log")   cam1: $(grep -c 'VxGetImage failed' "$R/cam1_node.log")   | StartStreaming failed: $(cat "$R"/cam*_node.log | grep -c 'Failed to start streaming')"
  python3 - "$BAG" "$T0_NS" <<'PY' 2>&1 | tee -a "$SUMMARY"
import sqlite3, glob, sys
import numpy as np
bag, t0_ns = sys.argv[1], int(sys.argv[2])
dbs = glob.glob(bag + "/*.db3")
if not dbs:
    print("  NO bag written (recorder or nodes failed)"); sys.exit(0)
c = sqlite3.connect("file:" + dbs[0] + "?immutable=1", uri=True)
for name in ("/cam0/image_raw", "/cam1/image_raw"):
    row = c.execute("select id from topics where name=?", (name,)).fetchone()
    if not row: print("  %s: topic missing" % name); continue
    ts = np.array([r[0] for r in c.execute("select timestamp from messages where topic_id=? order by timestamp", (row[0],))]) / 1e9
    if len(ts) < 3: print("  %s: only %d msgs" % (name, len(ts))); continue
    first = ts[0] - t0_ns / 1e9
    d = np.diff(ts); rel = ts[1:] - ts[0]
    def win(a, b):
        b = min(b, rel.max() + 1e-9)
        m = (rel >= a) & (rel < b)
        return (m.sum() / max(b - a, 1e-9), d[m].max() if m.any() else float("nan"))
    big = [(rel[i], d[i]) for i in range(len(d)) if d[i] > 0.1]
    print("  %s: %d msgs, first frame %.1f s after recorder start, avg %.1f Hz, worst gap %.0f ms" %
          (name, len(ts), first, (len(ts) - 1) / (ts[-1] - ts[0]), d.max() * 1000))
    for a, b in ((0, 15), (15, 30), (30, 1e9)):
        hz, g = win(a, b)
        print("      t+%2d..%s s: %.1f Hz, worst gap %.0f ms" % (a, "end" if b > 1e8 else "%2d" % b, hz, g * 1000))
    if big:
        print("      gaps > 100 ms: " + ", ".join("%.1fs:+%.2fs" % (r, g) for r, g in big[:8]))
PY
  [ "${KEEP:-0}" = 1 ] || rm -rf "$BAG"
done
( cd "$REPO" && echo "vc" | sudo -S ./trigger.sh off ) >/dev/null 2>&1
say ""; say "== done. Summary: $SUMMARY   (PWM is OFF, sensors are stopped; restart with record_tool/sensors_and_record.sh start N)"
