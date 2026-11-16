#!/usr/bin/env bash
# run_bag.sh -- replay one bag through OpenVINS (cam0 mono) and keep EVERYTHING in a permanent run folder.
# usage: run_bag.sh <tag> <bag_dir> <config_dir> <start_offset_s> [cam_topic_in_bag] [core]
#   cam_topic_in_bag defaults to /cam0/image_raw (remapped to /cam0/image_raw); core defaults to 2 (OpenVINS pinned there)
# Output: ~/rosbags/claude_tmp/runs_$(date +%Y%m%d)/<tag>/  {config/, odom_out/ (bag), openvins.log, play.log, gt.log, meta.txt}
# Does NOT kill anything itself except what it started (by PID). Ctrl-C / SIGINT stops it cleanly.
set +u
TAG=$1; BAG=$2; CFG=$3; OFFSET=$4; CAM=${5:-/cam0/image_raw}; CORE=${6:-2}
OUT=$HOME/rosbags/claude_tmp/runs_$(date +%Y%m%d)/$TAG
[ -e "$OUT" ] && { echo "run folder exists: $OUT (pick a new tag)"; exit 1; }
mkdir -p "$OUT"; cp -r "$CFG" "$OUT/config"
{ echo "tag $TAG"; echo "date $(date -Is)"; echo "bag $BAG"; echo "config_src $CFG"; echo "offset $OFFSET"; echo "cam_topic $CAM"; echo "core $CORE"; echo "baropub ${BAROPUB:-none}"; echo "playextra ${PLAYEXTRA:-none}";
  SRCD=$(readlink -f ${OV_WS:-$HOME/openvins_ws_new}/src/open_vins); echo "ov_ws ${OV_WS:-$HOME/openvins_ws_new}"; echo "open_vins_git $(git -C $SRCD rev-parse --short HEAD) branch=$(git -C $SRCD branch --show-current) dirty=$(git -C $SRCD status --short | wc -l)";
  echo "config_md5 $(cat $CFG/*.yaml | md5sum | cut -c1-8)"; } > "$OUT/meta.txt"
export DISPLAY=:1 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash; OV_WS=${OV_WS:-$HOME/openvins_ws_new}; source $OV_WS/install/setup.bash
PIDS=()
ros2 bag record -o "$OUT/odom_out" /ov_msckf/odomimu /ov_msckf/poseimu > "$OUT/record.log" 2>&1 & PIDS+=($!)
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 setsid taskset -c $CORE ros2 launch ov_msckf subscribe.launch.py config_path:=$OUT/config/estimator_config.yaml use_stereo:=false max_cameras:=1 verbosity:=INFO rviz_enable:=false > "$OUT/openvins.log" 2>&1 & LPID=$!; PIDS+=($!)
rviz2 -d $HOME/rosbags/claude_tmp/display_ros2_thick.rviz > "$OUT/rviz.log" 2>&1 & PIDS+=($!)
python3 $HOME/vio_tools/gt_oneshot_pub.py > "$OUT/gt.log" 2>&1 & PIDS+=($!)
if [ -n "${BAROPUB:-}" ]; then python3 $HOME/rosbags/claude_tmp/bagC/baro_pub.py > "$OUT/baro.log" 2>&1 & PIDS+=($!); fi
cleanup() {
  # setsid made the launcher PID a process-group leader: signal the whole group so run_subscribe_msckf dies too
  kill -INT -- -$LPID 2>/dev/null; kill -INT "${PIDS[@]}" 2>/dev/null; sleep 6
  kill -- -$LPID 2>/dev/null; kill "${PIDS[@]}" 2>/dev/null; sleep 1
  kill -9 -- -$LPID 2>/dev/null
}
trap cleanup EXIT INT TERM
sleep 15
REMAP=""; [ "$CAM" != "/cam0/image_raw" ] && REMAP="--remap $CAM:=/cam0/image_raw"
ros2 bag play "$BAG" --start-offset "$OFFSET" --read-ahead-queue-size 2000 $PLAYEXTRA --topics /imu0 $CAM /fc/gps1 /fc/ekf_yaw /fc/baro1 $REMAP > "$OUT/play.log" 2>&1 &
PP=$!; PIDS+=($PP); wait $PP
echo PLAY_DONE >> "$OUT/play.log"; sleep 12
echo "run finished; folder: $OUT"
