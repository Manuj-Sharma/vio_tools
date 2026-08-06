#!/usr/bin/env bash
# run_rviz.sh -- replay seg01 in RViz with the GPS ground-truth track overlaid.
#
#   ./run_rviz.sh              flight_D_best  (px2 + barometer)  <- default
#   ./run_rviz.sh gold         ov_9281_aerial_staticinit
#   ./run_rviz.sh gold_baro    gold + barometer
#   ./run_rviz.sh gold_px2     gold + sigma_px 2
#   ./run_rviz.sh best --no-gps    skip the truth overlay
#
# IN RVIZ:  green = VIO estimate     cyan = GPS ground truth
#
# The truth track is rotated to the VIO's arbitrary yaw but NOT rescaled, so a
# green box visibly larger than the cyan one IS the scale error, to scale.
# GPS truth spans 267 x 131 m, max 171.8 m from the origin, peak ~107.5 m.
#
# Ctrl+C to stop -- NOT Ctrl+Z. Ctrl+Z only suspends the launcher, which then
# never reaps its children; the zombies it leaves behind block every later run
# and cannot be killed (see vio_kill.sh).
set -u

CFG_NAME="${1:-best}"
GPS=true
[ "${2:-}" = "--no-gps" ] && GPS=false

case "$CFG_NAME" in
  best)      CFG=ov_9281_flight_D_best ;;
  gold)      CFG=ov_9281_aerial_staticinit ;;
  gold_baro) CFG=ov_9281_gold_baro ;;
  gold_px2)  CFG=ov_9281_gold_px2 ;;
  *)         CFG="$CFG_NAME" ;;
esac

S=/home/vc/sensor_ws/src/sensor_bringup/scripts
C=/home/vc/sensor_ws/src/sensor_bringup/config/$CFG/estimator_config.yaml
# Override with BAG=... to replay a different recording of the same flight, e.g.
#   BAG=~/rosbags/field_data/seg01_from_ros1   converted straight from seg01.bag
# NOTE that bag carries NO baro topics, so a use_baro config will start happily
# and simply never receive a measurement -- plain VIO wearing a barometer label.
BAG="${BAG:-/home/vc/rosbags/field_data/seg01_ros2_baro}"
[ -f "$C" ] || { echo "no config: $C" >&2; exit 1; }

# a use_baro config needs the barometer build; the default OV_WS predates it and
# would silently ignore the key, giving a plausible-looking plain-VIO run
if grep -qE "^use_baro:[[:space:]]*true" "$C"; then
  export OV_WS="${OV_WS:-$HOME/openvins_baro_ws}"
fi

set +u; source /opt/ros/humble/setup.bash; set -u

GT_PID=""
cleanup() {
  trap - EXIT INT TERM HUP
  [ -n "$GT_PID" ] && kill -TERM "$GT_PID" 2>/dev/null
  sleep 1
  [ -n "$GT_PID" ] && kill -KILL "$GT_PID" 2>/dev/null
  echo
  L="$(ls -t /tmp/vio_logs/openvins_bag_*.log 2>/dev/null | head -1)"
  [ -n "$L" ] || return 0
  echo "=== $L ==="
  python3 - "$L" <<'EOF'
import re,sys,numpy as np
t=open(sys.argv[1],errors='ignore').read()
ms=np.array([int(x) for x in re.findall(r"MSCKF update \((\d+) feats\)",t)])
P=np.array([[float(x) for x in m.groups()] for m in re.finditer(r"p_IinG = ([-\d.]+),([-\d.]+),([-\d.]+)",t)])
if not len(P) or not len(ms): sys.exit("  too short to score")
pct=100*np.mean(ms>0); d=np.linalg.norm(P[:,:2],axis=1).max()
print(f"  MSCKF {pct:.1f}%   max dist {d:.1f} m (truth 171.8)   peak z {P[:,2].max():.1f} m (truth ~107.5)")
print("  VERDICT: " + ("COLLAPSED, do not score" if pct < 40 or d > 400 else "usable"))
EOF
  python3 /home/vc/rosbags/calib/tools/vs_gps.py "$L" 2>&1 \
    | grep -vE "^  (GPS:|scale 1.000|yaw is|since VIO|rms/max)"
}
trap cleanup EXIT INT TERM HUP

echo "config : $CFG"
echo "bag    : $BAG"
if grep -qE "^use_baro:[[:space:]]*true" "$C" && \
   ! grep -qE "baro" "$BAG/metadata.yaml" 2>/dev/null; then
  echo "WARNING: config wants the barometer but $BAG has no baro topic --" >&2
  echo "         this will run as plain VIO. Use a no-baro config instead." >&2
fi
echo "OV_WS  : ${OV_WS:-$HOME/openvins_ws}"
grep -E "^up_msckf_sigma_px|^use_baro|^max_slam:|^init_dyn_use" "$C" | sed 's/^/  /'
echo

if [ "$GPS" = true ]; then
  python3 /home/vc/rosbags/calib/tools/gps_truth_rviz.py > /tmp/gps_truth.log 2>&1 &
  GT_PID=$!
  echo "GPS truth track: /ov_msckf/pathgt (cyan)  -- log /tmp/gps_truth.log"
  echo
fi

"$S/run_openvins_bag.sh" "$BAG" --config "$C"
