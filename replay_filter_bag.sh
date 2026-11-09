#!/usr/bin/env bash
# Replay the /imu0 of a recorded bag at real time through imu_filter_node and record /imu0 + /imu0_filtered to OUT_BAG.
#   replay_filter_bag.sh BAG OUT_BAG [filter params as name:=value ...]
#   e.g. replay_filter_bag.sh ~/rosbags/vio_20261109_121635 ~/rosbags/filt_o4_fc100 order:=4 cutoff_hz:=100 out_rate_hz:=300
# Params: order cutoff_hz out_rate_hz fs_hz max_fill_ticks compensate_delay (see imu_filter_node.cpp).
# Refuses to run while a live imx5_spi_node is up (its /imu0 would mix with the replay).
set -u
[ $# -ge 2 ] || { sed -n '2,6p' "$0"; exit 1; }
BAG="$1"; OUT="$2"; shift 2
set +u; source /opt/ros/humble/setup.bash; source "$HOME/vio_ar0234_icm20948/install/setup.bash"; set -u
pgrep -x imx5_spi_node >/dev/null && { echo "a live imx5_spi_node is running -- stop it first"; exit 1; }
[ -e "$OUT" ] && { echo "$OUT already exists"; exit 1; }
PARAMS=(); for p in "$@"; do PARAMS+=(-p "$p"); done
"$(ros2 pkg prefix imu_filter_node)/lib/imu_filter_node/imu_filter_node" --ros-args "${PARAMS[@]}" > "$OUT.filter.log" 2>&1 & NODE=$!   # exe directly: SIGINT through `ros2 run` left the node running
ros2 bag record -o "$OUT" /imu0 /imu0_filtered > "$OUT.rec.log" 2>&1 & REC=$!
sleep 4
ros2 bag play "$BAG" --topics /imu0 -r 1 > "$OUT.play.log" 2>&1
sleep 2
kill -INT $REC 2>/dev/null; wait $REC 2>/dev/null
kill -INT $NODE 2>/dev/null; wait $NODE 2>/dev/null
echo "filter log:"; head -n 3 "$OUT.filter.log" | cut -c1-260; grep -a "in .* Hz" "$OUT.filter.log" | tail -2 | cut -c1-200
echo "output bag: $OUT"
