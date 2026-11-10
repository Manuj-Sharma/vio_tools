#!/usr/bin/env bash
# usage: run_live_real.sh RUN_NAME   -- LIVE test on bag 165506: bag played at 1x -> real OpenVINS (frozen px8 config, core 2) + live map-matching node (cores 4-7) against the 110545 mosaic.
# Everything is causal; results in ./RUN_NAME (live.pkl, live.png, logs). Nothing is written to the FC, no hardware is touched.
RUN=${1:-run1}; HERE=$(cd "$(dirname "$0")" && pwd); OUT=$HERE/$RUN; mkdir -p $OUT
BAG=$HOME/rosbags/vio_20261103_165506; CFG=$HOME/rosbags/claude_tmp/bagE/cfg_frozen_px8_zupt_rw10_ts0_1core
MM=$HOME/ardupilot_gazebo/scripts/map_matching; MOS=$HOME/Manuj/mosaic_110545/mosaic_110545.json
export ROS_LOCALHOST_ONLY=1; source /opt/ros/humble/setup.bash; source $HOME/openvins_ws_new/install/setup.bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 taskset -c 2 ros2 launch ov_msckf subscribe.launch.py config_path:=$CFG/estimator_config.yaml use_stereo:=false max_cameras:=1 verbosity:=INFO rviz_enable:=false > $OUT/ov.log 2>&1 &
taskset -c 4-7 python3 $MM/live_real_node.py --mosaic $MOS --origin 13.7514565 77.6226947 --out $OUT > $OUT/node.log 2>&1 &
sleep 15
ros2 bag play $BAG --start-offset 19 --read-ahead-queue-size 2000 --topics /imu0 /cam0/image_raw /fc/gps1 /fc/ekf_yaw /fc/local_position_ned /fc/attitude > $OUT/play.log 2>&1
echo PLAY_DONE >> $OUT/play.log
sleep 4; pkill -INT -f "[l]ive_real_node.py"; for i in $(seq 1 30); do pgrep -f "[l]ive_real_node.py" >/dev/null || break; sleep 1; done
pkill -INT -f "[s]ubscribe.launch.py"; sleep 2; pkill -f "[r]un_subscribe_msckf"; echo FINISHED >> $OUT/play.log
