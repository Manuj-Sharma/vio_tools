#!/usr/bin/env bash
# usage: run_live_real_gui.sh RUN_NAME  -- same live test as run_live_real.sh, WITH rviz2 and the OpenCV dashboard on DISPLAY :1 (and the dashboard saved to RUN_NAME/dashboard.mp4).
# rviz and the dashboard add CPU load (rviz is not pinned), so latency here is slightly pessimistic compared with the headless run.
RUN=${1:-gui1}; HERE=$(cd "$(dirname "$0")" && pwd); OUT=$HERE/$RUN; mkdir -p $OUT
BAG=${BAG:-$HOME/rosbags/vio_20261103_165506}; OFFSET=${OFFSET:-19}; ORIGIN=${ORIGIN:-"13.7514565 77.6226947"}; MINALT=${MINALT:-60}; CFG=${CFG:-$HOME/rosbags/claude_tmp/bagE/cfg_frozen_px8_zupt_rw10_ts0_1core}
MM=$HOME/ardupilot_gazebo/scripts/map_matching; MOS=$HOME/Manuj/mosaic_110545/mosaic_110545.json
export ROS_LOCALHOST_ONLY=1 DISPLAY=${DISPLAY:-:1}; source /opt/ros/humble/setup.bash; source $HOME/openvins_ws_new/install/setup.bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 taskset -c 2 ros2 launch ov_msckf subscribe.launch.py config_path:=$CFG/estimator_config.yaml use_stereo:=false max_cameras:=1 verbosity:=INFO rviz_enable:=false > $OUT/ov.log 2>&1 &
taskset -c 4-7 python3 $MM/live_real_node.py --mosaic $MOS --origin $ORIGIN --min-alt $MINALT --out $OUT --gui --video $OUT/dashboard.mp4 > $OUT/node.log 2>&1 &
sleep 6
rviz2 -d $HERE/live_real.rviz > $OUT/rviz.log 2>&1 &
sleep 9
ros2 bag play $BAG ${REMAP:+--remap $REMAP} --start-offset $OFFSET --read-ahead-queue-size 2000 --topics /imu0 ${PLAYCAM:-/cam0/image_raw} /fc/gps1 /fc/ekf_yaw /fc/local_position_ned /fc/attitude > $OUT/play.log 2>&1
echo PLAY_DONE >> $OUT/play.log
sleep 6; pkill -INT -f "[l]ive_real_node.py"; for i in $(seq 1 30); do pgrep -f "[l]ive_real_node.py" >/dev/null || break; sleep 1; done
pkill -INT -f "[s]ubscribe.launch.py"; sleep 2; pkill -f "[r]un_subscribe_msckf"; echo FINISHED >> $OUT/play.log
echo "(rviz2 and the dashboard window are left open - close them yourself)" >> $OUT/play.log
