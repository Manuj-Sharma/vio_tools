#!/usr/bin/env bash
# run_gold_px2.sh -- replay seg01 with the gold standard + sigma_px 2, on konsole.
#
#   ./run_gold_px2.sh              gold_px2, with RViz
#   ./run_gold_px2.sh gold         the untouched gold standard, for comparison
#   ./run_gold_px2.sh gold_px2 3   3 reps, headless, scored (no RViz)
#
# WHY THIS CONFIG EXISTS
#
# ov_9281_gold_px2 is ov_9281_aerial_staticinit with ONE change:
#   up_msckf_sigma_px: 1 -> 2
#   up_slam_sigma_px:  1 -> 2
#
# Measured over 32 replays on 2026-08-05:
#
#   up_msckf_sigma_px   runs   collapsed   MSCKF median
#         1               12      11          21.1%
#         2               20       0          63.5%
#
# and among BAROMETER-OFF runs alone, which rules out the barometer as the cause:
#   px=1: 11 runs, 10 collapsed, median 21.6%
#   px=2:  2 runs,  0 collapsed, median 62.5%
#
# A tighter pixel sigma shrinks the chi2 gate, so most features are rejected,
# MSCKF usage falls to ~20%, and the filter dead-reckons away -- trajectories of
# 3 km to 100 km. The gold standard is the last config still on 1; every flight_*
# config moved to 2 and none of them has ever collapsed.
#
# The run PRINTS ITS OWN VERDICT at the end. MSCKF% is otherwise a blind metric
# (69.4/66.1/71.1% gave box errors of +35/+2/+36%) -- but below ~40% it is not
# measuring accuracy, it is telling you the filter died. Judge accuracy with
# vs_gps.py; use MSCKF% only to decide whether the run is worth scoring at all.
set -u

CFG_NAME="${1:-gold_px2}"
REPS="${2:-1}"
case "$CFG_NAME" in
  gold_px2)  CFG=ov_9281_gold_px2 ;;
  gold)      CFG=ov_9281_aerial_staticinit ;;
  gold_baro) CFG=ov_9281_gold_baro ;;
  best)      CFG=ov_9281_flight_D_best ;;
  *)         CFG="$CFG_NAME" ;;
esac

# A config with use_baro needs the barometer workspace; the default OV_WS is the
# pre-barometer build and would silently ignore the key. run_openvins_bag.sh now
# refuses that combination, but set it here so the caller never has to think.
if grep -qE "^use_baro:[[:space:]]*true" \
     "/home/vc/sensor_ws/src/sensor_bringup/config/$CFG/estimator_config.yaml" 2>/dev/null; then
  export OV_WS="${OV_WS:-$HOME/openvins_baro_ws}"
fi

C=/home/vc/sensor_ws/src/sensor_bringup/config/$CFG/estimator_config.yaml
BAG=/home/vc/rosbags/field_data/seg01_ros2_baro
S=/home/vc/sensor_ws/src/sensor_bringup/scripts
[ -f "$C" ] || { echo "no config: $C" >&2; exit 1; }

RV=""; [ "$REPS" = "1" ] || RV="--no-rviz"

echo "config : $CFG"
grep -E "^up_(msckf|slam)_sigma_px|^up_(msckf|slam)_chi2_multipler|^max_slam|^init_dyn_use" "$C" | sed 's/^/  /'
echo

for r in $(seq 1 "$REPS"); do
  [ "$REPS" = "1" ] || echo "########## rep $r/$REPS ##########"
  "$S/vio_kill.sh" >/dev/null 2>&1 || true
  sleep 3
  M=$(mktemp); touch "$M"
  "$S/run_openvins_bag.sh" "$BAG" --config "$C" $RV
  L="$(find /tmp/vio_logs -name 'openvins_bag_*.log' -newer "$M" -printf '%T@ %p\n' 2>/dev/null \
       | sort -rn | head -1 | cut -d' ' -f2-)"
  rm -f "$M"
  [ -n "$L" ] || { echo "  no log produced"; continue; }
  echo
  echo "=== $L ==="
  python3 - "$L" <<'EOF'
import re,sys,numpy as np
t=open(sys.argv[1],errors='ignore').read()
ms=np.array([int(x) for x in re.findall(r"MSCKF update \((\d+) feats\)",t)])
sl=np.array([int(x) for x in re.findall(r"SLAM update \((\d+) feats\)",t)])
P=np.array([[float(x) for x in m.groups()] for m in re.finditer(r"p_IinG = ([-\d.]+),([-\d.]+),([-\d.]+)",t)])
if not len(P): sys.exit("  no poses")
pct=100*np.mean(ms>0); d=np.linalg.norm(P[:,:2],axis=1).max()
print(f"  MSCKF {pct:.1f}%   mean SLAM feats {sl.mean():.1f}   peak z {P[:,2].max():.1f} m   max dist {d:.1f} m")
print(f"  reference: healthy MSCKF 55-70%, GPS truth max dist 171.8 m, true peak z ~107.5 m")
print("  VERDICT: " + ("COLLAPSED -- filter died, do not score this run" if pct < 40 else "ok -- worth scoring"))
EOF
  python3 /home/vc/rosbags/calib/tools/vs_gps.py "$L" 2>&1 | grep -vE "^  (GPS:|scale 1.000|yaw is|since VIO|rms/max)"
done
