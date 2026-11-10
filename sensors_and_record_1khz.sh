#!/usr/bin/env bash
# sensors_and_record_1khz.sh - sensors_and_record_notrigger.sh with the IMX-5 in 1 kHz STREAM mode (raw /imu0, no filtering).
#   sensors_and_record_1khz.sh start [seconds=180] [split_sec=0]     # 0 = record until stopped
#   sensors_and_record_1khz.sh status
#   sensors_and_record_1khz.sh stop
# Same sensors, topics, preflight, health monitor and report as record_tool/sensors_and_record_notrigger.sh (cameras free-running,
# PWM trigger off, bag in ~/rosbags/vio_<stamp>/ with /cam0 /cam1 /imu0 + the Cube topics); the only difference is the IMU:
#   IMU_STREAM=1  IMU_FRAME_LEN=128  IMU_POLL_US=1000  IMU_SPI_HZ=3000000   (same settings as vio_live_with_filter.sh)
# /imu0 is the RAW 1 kHz stream: expect ~950 Hz recorded (about 5% of 1 ms ticks are missed in stream mode; the filter node
# gap-fills them). No imu_filter_node is started here: filter OFFLINE with imu_filter_sweep.py or vio_replay_with_filter.sh.
# Override any of the IMU_* variables or EXP_IMU_HZ in front of the command if needed.
# Start/stop go through the original script, so `status` and `stop` of either name act on the same recording.
set -u
REPO="${REPO:-$HOME/vio_ar0234_icm20948}"
export IMU_STREAM="${IMU_STREAM:-1}" IMU_FRAME_LEN="${IMU_FRAME_LEN:-128}" IMU_POLL_US="${IMU_POLL_US:-1000}" IMU_SPI_HZ="${IMU_SPI_HZ:-3000000}"
export EXP_IMU_HZ="${EXP_IMU_HZ:-900}"     # health/preflight expectation (default 250 would flag nothing, but 900 catches a fallback to poll mode)
# Per-tick (5 s) health rates by MESSAGE TIMESTAMP, not by "rows that appeared in the sqlite file since the last poll": rosbag2 flushes in
# batches, so the old numbers swung 560 <-> 1500 Hz on a steady ~990 Hz stream. Needs record_tool/record_health.py from branch
# imu-1khz-filter (uncommitted change; without the patch the variable is ignored and the old behaviour stays). HEALTH_RATE_BY_STAMP=0 turns it off.
export HEALTH_RATE_BY_STAMP="${HEALTH_RATE_BY_STAMP:-1}"
[ "${1:-}" = start ] && echo "[1khz] IMU stream mode: ${IMU_FRAME_LEN} B frames / ${IMU_POLL_US} us loop / ${IMU_SPI_HZ} Hz SPI, expected /imu0 >= ${EXP_IMU_HZ} Hz"
exec "$REPO/record_tool/sensors_and_record_notrigger.sh" "$@"
