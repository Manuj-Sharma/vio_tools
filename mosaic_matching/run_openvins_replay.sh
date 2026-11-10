#!/usr/bin/env bash
# usage: run_openvins_replay.sh BAG_DIR START_OFFSET_S TAG [CONFIG_DIR]
# Replays a bag in REAL TIME through OpenVINS (mono cam0, frozen config, pinned to core 2) and records /ov_msckf/odomimu + poseimu to ./odom_TAG.
# Does not modify the config. No rviz, no GPS publisher.
BAG=$1; OFF=$2; TAG=$3
CFG=${4:-$HOME/rosbags/claude_tmp/bagE/cfg_frozen_px8_zupt_rw10_ts0_1core}
OUT=$(cd "$(dirname "$0")" && pwd); export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash; source $HOME/openvins_ws_new/install/setup.bash
rm -rf $OUT/odom_$TAG
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 taskset -c 2 ros2 launch ov_msckf subscribe.launch.py config_path:=$CFG/estimator_config.yaml use_stereo:=false max_cameras:=1 verbosity:=INFO rviz_enable:=false > $OUT/ov_$TAG.log 2>&1 &
ros2 bag record -o $OUT/odom_$TAG /ov_msckf/odomimu /ov_msckf/poseimu > $OUT/record_$TAG.log 2>&1 &
sleep 15
ros2 bag play $BAG --start-offset $OFF --read-ahead-queue-size 2000 --topics /imu0 /cam0/image_raw /fc/gps1 /fc/ekf_yaw > $OUT/play_$TAG.log 2>&1
echo PLAY_DONE >> $OUT/play_$TAG.log
sleep 5; pkill -INT -f "[r]os2 bag record"; sleep 3; pkill -INT -f "[s]ubscribe.launch.py"; sleep 2; pkill -f "[r]un_subscribe_msckf"; sleep 1; echo FINISHED $TAG >> $OUT/play_$TAG.log
