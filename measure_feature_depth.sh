#!/usr/bin/env bash
# measure_feature_depth.sh [CFG] -- where does the filter think the ground is?
#
# The field is flat, so every triangulated feature should sit at z ~ 0, the
# launch plane. That makes the distribution of feature z a direct readout of
# what the depth/translation degeneracy is doing:
#
#   mean z ~   0      depth is right
#   mean z ~ -11 m    depth inflated ~10% -- exactly the scale error measured
#                     against GPS (Umeyama scale 0.907 = trajectory 10% too big)
#   wide spread       triangulation is noise, not geometry
#
# A nadir camera cannot separate depth from translation (pixel flow constrains
# only t/d), so if the filter has settled on the wrong depth, the trajectory is
# wrong by the same factor. This measures the depth side of that ratio directly
# instead of inferring it from trajectory size.
#
# OpenVINS only publishes the point clouds when something subscribes, so the
# recorder must be running for this to produce anything.
set -u

CFG="${1:-/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_flight_D_best/estimator_config.yaml}"
BAG="${BAG:-/home/vc/rosbags/field_data/seg01_ros2_baro}"
export OV_WS="${OV_WS:-$HOME/openvins_baro_ws}"
S=/home/vc/sensor_ws/src/sensor_bringup
OUT=/tmp/feat_cloud

[ -f "$CFG" ] || { echo "no config at $CFG" >&2; exit 1; }
grep -qE "^use_baro:[[:space:]]*(true|1)" "$CFG" || { echo "REFUSING: $CFG has no use_baro: true" >&2; exit 1; }

reap () {
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "bag[ ]record") $(pgrep -x rviz2); do kill -TERM "$p" 2>/dev/null; done
  sleep 4
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play") \
           $(pgrep -f "bag[ ]record") $(pgrep -x rviz2); do kill -KILL "$p" 2>/dev/null; done
  sleep 6
}

echo "config: $CFG"
reap
rm -rf "$OUT"

set +u; source /opt/ros/humble/setup.bash; source "$OV_WS/install/setup.bash"; set -u

MARK=/tmp/fdmark.$$; touch "$MARK"
O=/tmp/feat_depth_run.out
"$S/scripts/run_openvins_bag.sh" "$BAG" --config "$CFG" --no-rviz > "$O" 2>&1 &
PID=$!

# wait for the node to come up, then subscribe -- publication is gated on
# get_subscription_count(), so nothing is emitted until the recorder attaches
for _ in $(seq 1 60); do
  ros2 topic list 2>/dev/null | grep -q "/ov_msckf/points_msckf" && break
  sleep 2
done
ros2 bag record -o "$OUT" /ov_msckf/points_msckf /ov_msckf/points_slam > /tmp/feat_rec.out 2>&1 &
RECPID=$!
echo "recording feature clouds -> $OUT"

ok=0
for _ in $(seq 1 400); do
  grep -q "playback finished" "$O" 2>/dev/null && { ok=1; break; }
  grep -q "ERROR:" "$O" 2>/dev/null && { echo "  REFUSED -- see $O"; break; }
  kill -0 "$PID" 2>/dev/null || break
  sleep 4
done
sleep 20
LOG="$(find /tmp/vio_logs -name 'openvins_bag_*.log' -newer "$MARK" -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
rm -f "$MARK"
kill -INT "$RECPID" 2>/dev/null; sleep 4
kill -INT "$PID" 2>/dev/null; sleep 3; reap

echo
[ "$ok" -eq 1 ] || echo "  WARNING: playback did not finish cleanly"
echo "vio log : $LOG"
echo "clouds  : $OUT"
echo
python3 /home/vc/rosbags/calib/tools/analyse_feature_depth.py "$OUT" "$LOG"
