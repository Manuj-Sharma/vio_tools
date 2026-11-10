#!/usr/bin/env bash
# Stop EVERY imu_filter_node (several nodes publishing to /imu0_filtered interleave and corrupt OpenVINS: zero-DT IMU readings, NaN state).
#   stop_filter_node.sh
# SIGINT first (clean shutdown), then SIGTERM, then SIGKILL; prints what is still alive. Exit 0 only if none is left.
# Matches the exact process name (pgrep -x), so it cannot match this script's own command line.
pids() { pgrep -x imu_filter_node | tr '\n' ' '; }
p=$(pids)
[ -z "$p" ] && { echo "no imu_filter_node running"; exit 0; }
echo "imu_filter_node pids: $p"
for sig in INT TERM KILL; do
  pkill -"$sig" -x imu_filter_node
  for _ in 1 2 3 4 5 6; do sleep 0.5; [ -z "$(pids)" ] && break; done
  [ -z "$(pids)" ] && { echo "stopped (SIG$sig)"; exit 0; }
  echo "still alive after SIG$sig: $(pids)"
done
echo "FAILED: still running: $(pids)" >&2
exit 1
