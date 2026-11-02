#!/usr/bin/env bash
#
# kalibr_cam1_pipeline.sh - cam1 intrinsics, then cam1-IMU, with Kalibr in Docker (one detached run).
#
#   1. imu_300hz.yaml  : copy of kalibr-docker/data/imu.yaml with update_rate 300 (the IMU now runs ~313 Hz); imu.yaml untouched
#   2. convert the cam1 intrinsics ROS 2 bag -> ROS 1 bag (rosbags-convert from ~/.local/bin)
#   3. kalibr_calibrate_cameras (pinhole-radtan, /cam1/image_raw)            -> data/<intr>-camchain.yaml
#   4. convert the cam1-IMU bag -> ROS 1 bag
#   5. kalibr_calibrate_imu_camera with that camchain + imu_300hz.yaml       -> data/<imu>-camchain-imucam.yaml
#
# Usage: ~/vio_tools/kalibr_cam1_pipeline.sh                       # defaults below (2026-11-02 bags)
#        INTR=~/rosbags/calib_data/cam1_intrinsics_XXXX IMUB=~/rosbags/calib_data/cam1_imu_XXXX RUNTAG=cam1_try2 ~/vio_tools/kalibr_cam1_pipeline.sh
# Detached:  setsid nohup ~/vio_tools/kalibr_cam1_pipeline.sh > /tmp/vio_logs/kalibr_cam1_pipeline.out 2>&1 < /dev/null &
# Logs: /tmp/vio_logs/kalibr_cam1_<RUNTAG>/step*.log. Results (data/): <RUNTAG>_calib-*, <RUNTAG>_imu-*. Stops at the first failed step.
# Commands follow data/good_cmds (--focal-length-guess 900 via KALIBR_PATCH=1, --timeoffset-padding 0.1, --verbose).
# run.sh uses `docker run -it`, so each Kalibr call is wrapped in `script` to give it a pseudo-terminal.

set -u
KD="${KD:-$HOME/projects/kalibr-docker}"
INTR="${INTR:-$HOME/rosbags/calib_data/cam1_intrinsics_20261102}"
IMUB="${IMUB:-$HOME/rosbags/calib_data/cam1_imu_20261102}"
RUNTAG="${RUNTAG:-cam1_new}"
CONV="${CONV:-$HOME/.local/bin/rosbags-convert}"
MODEL="${MODEL:-pinhole-radtan}"   # camera model for the intrinsics (pinhole-equi, omni-radtan, ds ... also possible)
SKIP_IMU="${SKIP_IMU:-0}"          # 1 = stop after the camera calibration
SRCBAG="${SRCBAG:-}"               # reuse the already converted intrinsics bag of another RUNTAG (symlink), e.g. SRCBAG=cam1_new
CAM_FREQ="${CAM_FREQ:-4}"        # Hz of images Kalibr takes from the intrinsics bag (without it Kalibr uses ALL ~7000 frames: ~2.5 h)
IMUCAM_FREQ="${IMUCAM_FREQ:-20}"  # Hz of images for the cam-IMU run (the bag has ~58 Hz)
LOG="${LOG:-/tmp/vio_logs/kalibr_cam1_$RUNTAG}"
mkdir -p "$LOG"
say() { echo "[$(date +%T)] $*" | tee -a "$LOG/pipeline.log"; }
die() { say "FAILED: $*"; exit 1; }

[ -d "$INTR" ] && [ -d "$IMUB" ] || die "bag folder missing: $INTR / $IMUB"
[ -x "$CONV" ] || die "rosbags-convert not found at $CONV"
cd "$KD" || die "no $KD"
docker image inspect kalibr:noetic >/dev/null 2>&1 || die "docker image kalibr:noetic not available"

say "== kalibr cam1 pipeline: RUNTAG=$RUNTAG"
sed -E 's/^(update_rate:[[:space:]]*)[0-9.]+/\1300.0/' data/imu.yaml > data/imu_300hz.yaml
grep -q "update_rate:.*300.0" data/imu_300hz.yaml || die "could not write data/imu_300hz.yaml"
say "step 1 done: data/imu_300hz.yaml (update_rate 300)"

convert() {   # convert <ros2 bag dir> <out.bag>
  [ -s "$2" ] && { say "reusing existing $2"; return 0; }
  rm -f "$2"
  say "converting $(basename "$1") -> $2 ($(du -sh "$1" | cut -f1) in)"
  "$CONV" --src "$1" --dst "$2" > "$LOG/convert_$(basename "$2").log" 2>&1 || return 1
  [ -s "$2" ] && say "  done: $(du -sh "$2" | cut -f1)"
}

[ -n "$SRCBAG" ] && [ -s "data/${SRCBAG}_calib.bag" ] && ln -sfn "${SRCBAG}_calib.bag" "data/${RUNTAG}_calib.bag"
convert "$INTR" "data/${RUNTAG}_calib.bag" || die "conversion of the intrinsics bag failed (see $LOG)"
say "step 3: kalibr_calibrate_cameras at ${CAM_FREQ} Hz of images"
# Kalibr asks for a focal length on stdin when its own initialisation fails (KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 in the image);
# without an answer it hangs. Feed it the same guess as --focal-length-guess (a few lines, in case it asks more than once).
printf "${FOCAL_GUESS:-900}\n%.0s" {1..20} | KALIBR_PATCH=1 script -qefc "./run.sh kalibr_calibrate_cameras --bag /data/${RUNTAG}_calib.bag --topics /cam1/image_raw --models ${MODEL} --target /data/april_6x6_80x80cm.yaml --focal-length-guess ${FOCAL_GUESS:-900} --bag-freq ${CAM_FREQ} --verbose --dont-show-report" /dev/null > "$LOG/step3_cameras.log" 2>&1
CC="data/${RUNTAG}_calib-camchain.yaml"
[ -s "$CC" ] || die "no camchain produced ($CC) -- see $LOG/step3_cameras.log"
say "step 3 done: $CC"
cat "data/${RUNTAG}_calib-results-cam.txt" 2>/dev/null | tee -a "$LOG/pipeline.log" | head -20

[ "$SKIP_IMU" = 1 ] && { say "== SKIP_IMU=1: stopping after the camera calibration. Results in $KD/data/ (${RUNTAG}_calib-*)"; exit 0; }
[ -n "$SRCBAG" ] && [ -s "data/${SRCBAG}_imu.bag" ] && ln -sfn "${SRCBAG}_imu.bag" "data/${RUNTAG}_imu.bag"
convert "$IMUB" "data/${RUNTAG}_imu.bag" || die "conversion of the cam-IMU bag failed (see $LOG)"
say "step 5: kalibr_calibrate_imu_camera at ${IMUCAM_FREQ} Hz of images"
script -qefc "./run.sh kalibr_calibrate_imu_camera --bag /data/${RUNTAG}_imu.bag --cam /data/${RUNTAG}_calib-camchain.yaml --imu /data/imu_300hz.yaml --target /data/april_6x6_80x80cm.yaml --timeoffset-padding 0.1 --bag-freq ${IMUCAM_FREQ} --verbose --dont-show-report" /dev/null > "$LOG/step5_imucam.log" 2>&1
IC="data/${RUNTAG}_imu-camchain-imucam.yaml"
[ -s "$IC" ] || die "no camera-IMU result produced ($IC) -- see $LOG/step5_imucam.log"
say "step 5 done: $IC"
cat "data/${RUNTAG}_imu-results-imucam.txt" 2>/dev/null | tee -a "$LOG/pipeline.log" | head -60
say "== ALL DONE. Results in $KD/data/ (${RUNTAG}_calib-*, ${RUNTAG}_imu-*)"
