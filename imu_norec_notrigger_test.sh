#!/usr/bin/env bash
# imu_norec_notrigger_test.sh [SECONDS=150]   -- NO-TRIGGER twin of imu_norec_test.sh: cameras free-running, PWM trigger OFF
# (Only difference: no trigger-mode switch, no trigger_cam.sh loop; makes sure the PWM is off. Matches the
#  record_map_matching / *_notrigger setups.)
# Do the ~25 s /imu0 rate dips (seen in vio_20261102_103813) still happen with NO recording?
#
# Starts exactly what sensors_and_record.sh starts (same commands, same settings: cameras in trigger
# mode at 60 Hz, imx5 driver with IMU_POLL_MS, FC node, trigger loop) but NOTHING else: no
# `ros2 bag record`, no record_health.py, no preflight dry-run recording, no `ros2 topic` probes.
# It then just waits SECONDS and reads the imx5 driver's own "/imu0 N Hz" log lines (one per 5 s,
# published rate BEFORE DDS/recorder), prints them as a bar strip and checks for a periodic dip.
#
# Reading it:  dips of ~25 s period still there -> not the recorder / disk; look at CPU scheduling
#              no dips                           -> the recording (disk writeback / DDS load) is the trigger
# RT_PRIO=N   after the imx5 node is up, give ALL its threads SCHED_FIFO priority N (sudo chrt -a -f -p N <pid>;
#             asks for your sudo password up front). Compare against a run without it: dips gone -> the
#             poll thread was losing the CPU to other work. RT_PRIO=50 is a sane first try.
# ONLY_IMU=1  skip cameras/trigger/FC node: run the imx5 driver alone (is the camera load involved at all?).
# Run in your own terminal, with the stack STOPPED (sensors_and_record.sh stop). Leaves the stack
# STOPPED when done. Log: $OUT/sensors.log (OUT=/tmp/vio_logs/norec_<time>).
set -u
SECS="${1:-150}"
REPO="${REPO:-$HOME/vio_ar0234_icm20948}"
OUT="${OUT:-/tmp/vio_logs/norec_nt_$(date +%Y%m%d-%H%M%S)}"
PWM="${PWM:-/sys/class/pwm/pwmchip2/pwm0}"
export IMU_POLL_MS="${IMU_POLL_MS:-3}"
CAM_NORMAL_SETTLE="${CAM_NORMAL_SETTLE:-2}"; CAM_TRIGGER_SETTLE="${CAM_TRIGGER_SETTLE:-2}"
CAM_WAIT_TIMEOUT="${CAM_WAIT_TIMEOUT:-40}"; WARMUP_SEC="${WARMUP_SEC:-5}"; STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-25}"
RT_PRIO="${RT_PRIO:-}"
case "${IMU_DEDUPE:-0}" in 1|true|True) DEDUPE_ARG=true ;; *) DEDUPE_ARG=false ;; esac   # launch arg must be a bool word, not 1/0
mkdir -p "$OUT"
if [ -n "$PWM" ] && [ "$(cat "$PWM/enable" 2>/dev/null)" = 1 ]; then
  echo "[norec] PWM trigger is ON -- switching it off"; (cd "$REPO" && echo "vc" | sudo -S ./trigger.sh off) >/dev/null 2>&1
  [ "$(cat "$PWM/enable" 2>/dev/null)" = 1 ] && { echo "[norec] could not switch the PWM trigger off" >&2; exit 1; }
fi
export CAM_TRIGGER_MODE=0
echo "[norec] settings: ONLY_IMU=${ONLY_IMU:-0} IMU_POLL_MS=$IMU_POLL_MS IMU_DEDUPE=${IMU_DEDUPE:-0} IMU_RETRY_US=${IMU_RETRY_US:-0} RT_PRIO=${RT_PRIO:-none} SECS=$SECS"
[ -n "$RT_PRIO" ] && { sudo -v || { echo "[norec] sudo needed for RT_PRIO" >&2; exit 1; }; }
die() { echo "[norec] $*" >&2; exit 1; }
for p in imx5_spi_node vizionsdk_camer ros2; do pgrep -x "$p" >/dev/null && die "$p already running -- sensors_and_record.sh stop first"; done
pgrep -f "[r]ecord_health.py|[r]os2 bag record|[r]osbag2_recorder" >/dev/null && die "a recorder/health monitor is running"

stop_all() {
  pkill -f "trigger_cam.sh" 2>/dev/null; pkill vizionsdk_camer 2>/dev/null; pkill imx5_spi_node 2>/dev/null
  pkill -f "flight_dataset_recorder_publisher.py" 2>/dev/null
}
trap 'echo; echo "[norec] interrupted -- stopping sensors"; stop_all; exit 1' INT TERM

set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash"; set -u
wait_for() { local t0=$SECONDS; while (( SECONDS - t0 < $2 )); do pgrep -x -- "$1" >/dev/null && return 0; sleep 1; done; return 1; }

if [ "${ONLY_IMU:-0}" = 1 ]; then
  echo "[norec] ONLY_IMU=1: imx5 driver alone (no cameras, no trigger, no FC node)"
  ros2 daemon stop >/dev/null 2>&1 || true
  setsid nohup ros2 launch imx5_spi_driver imx5.launch.py poll_period_ms:=$IMU_POLL_MS dedupe_imx_time:=$DEDUPE_ARG retry_empty_us:=${IMU_RETRY_US:-0} > "$OUT/sensors.log" 2>&1 < /dev/null &
  wait_for imx5_spi_node "$STARTUP_TIMEOUT" || { stop_all; die "IMU never came up -- $OUT/sensors.log"; }
  CAM_WAIT_TIMEOUT=0; WARMUP_SEC=3
else
echo "[norec] cameras -> normal mode"
vizion-ctl -d 0 -s trigger_mode=0; vizion-ctl -d 1 -s trigger_mode=0
sleep "$CAM_NORMAL_SETTLE"
ros2 daemon stop >/dev/null 2>&1 || true
setsid nohup ros2 launch "$REPO/vio_bringup/vio_bringup.launch.py" > "$OUT/sensors.log" 2>&1 < /dev/null &
wait_for imx5_spi_node "$STARTUP_TIMEOUT" || { stop_all; die "IMU never came up -- $OUT/sensors.log"; }
wait_for vizionsdk_camer "$STARTUP_TIMEOUT" || { stop_all; die "camera never came up -- $OUT/sensors.log"; }
fi
if [ -n "$RT_PRIO" ]; then
  sudo chrt -a -f -p "$RT_PRIO" "$(pgrep -x imx5_spi_node)" && echo "[norec] imx5_spi_node threads -> SCHED_FIFO $RT_PRIO"
  chrt -p "$(pgrep -x imx5_spi_node)" | sed 's/^/[norec]   /'
fi | tee "$OUT/rt.txt"
if [ "${ONLY_IMU:-0}" != 1 ]; then
echo "[norec] cameras free-running (NO trigger)"
echo "[norec] settling $((CAM_WAIT_TIMEOUT)) s for cam1 to come up (no probes), then ${WARMUP_SEC}s"
fi
sleep "$CAM_WAIT_TIMEOUT"; sleep "$WARMUP_SEC"
echo "[norec] measuring for ${SECS}s with nothing recording ($(date +%T)) ..."
sleep "$SECS"
cp "$OUT/sensors.log" "$OUT/sensors.final.log"
stop_all
echo "[norec] sensors stopped. Analysing $OUT/sensors.log"

python3 - "$OUT/sensors.log" "$SECS" <<'PY'
import re, sys
import statistics as st
rows = [(float(m.group(1)), m.group(2)) for l in open(sys.argv[1]) for m in [re.search(r"\[imx5_spi_node\].*?/imu0 ([\d.]+) Hz(.*)", l)] if m]
if len(rows) < 8: sys.exit(f"only {len(rows)} driver rate lines -- check the log")
rows = rows[2:]                   # drop start-up ramp (first 10 s)
hz = [r[0] for r in rows]
def extra(t):                     # poll counters added on branch imu-poll-diagnostics
    m = re.search(r"polls=(\d+) empty=(\d+) late>4.5ms=(\d+) maxgap=([\d.]+)ms spi_max=([\d.]+)ms(?: dedup_dropped=(\d+))?", t)
    return f"  polls={m.group(1)} empty={m.group(2)} late={m.group(3)} maxgap={m.group(4)}ms spi_max={m.group(5)}ms" + (lambda r: f" retries={r.group(1)} ok={r.group(2)}" if r else "")(re.search(r"retries=(\d+) retry_ok=(\d+)", t)) + (f" dedup_dropped={m.group(6)}" if m.group(6) else "") if m else ""
med = st.median(hz)
print(f"\n{len(hz)} five-second bins, median {med:.0f} Hz, min {min(hz):.0f}, max {max(hz):.0f}")
print("bin  Hz   (| = low: <93% of median)")
for i, h in enumerate(hz):
    print(f"{i*5+10:4d}s {h:6.1f} {'|' * int(max(0, (h - 250)) / 4):21s}{'<-- DIP' if h < 0.93 * med else '       '}{extra(rows[i][1])}")
dips = [i * 5 + 10 for i, h in enumerate(hz) if h < 0.93 * med]
print("\ndip bins at t(s) since driver start:", dips if dips else "none")
if len(dips) >= 2:
    # collapse adjacent bins into events
    ev = [d for k, d in enumerate(dips) if k == 0 or d - dips[k - 1] > 5]
    print("dip events:", ev, " spacing:", [b - a for a, b in zip(ev, ev[1:])])
print("\n(RT run? see rt.txt in the log dir)")
print("\nVERDICT:", "periodic dips PRESENT without recording -> not the recorder/disk" if len(dips) >= 3
      else "few/no dips without recording -> recording (disk/DDS load) is likely the trigger" if len(dips) <= 1 else "ambiguous (2 dip bins) -- rerun with longer SECONDS")
PY
