#!/usr/bin/env bash
# six_pos_capture.sh [SECS=20] [SETTLE=6] [OUTDIR]
# Six-position static test of the IMX-5 accelerometer/gyro. Place the rig with ANY face up, keep it still, press Enter;
# the pose (+X -X +Y -Y +Z -Z up) is detected automatically, so the order does not matter. Repeat until all six faces are captured,
# then the analysis runs. Uses imx5_read_pimu (read-only SPI, ONE SPI owner: stop imx5_spi_node / imx5_noros_poll first).
# Hands off while recording; no vibration (fans, typing on the same table).
set -u
SECS="${1:-20}"; SETTLE="${2:-6}"
OUT="${3:-$HOME/vio_tools/imu_data/six_pos_$(date +%Y%m%d_%H%M%S)}"
HERE="$(cd "$(dirname "$0")" && pwd)"
TOOL="$HERE/imx5_read_pimu"
[ -x "$TOOL" ] || { echo "missing $TOOL (run build_imx5_read_pimu.sh)"; exit 1; }
for p in imx5_spi_node imx5_noros_poll imx5_poll_rate; do
  pgrep -x "$p" >/dev/null 2>&1 && { echo "ABORT: $p is running (one SPI owner only). Stop it first."; exit 3; }
done
mkdir -p "$OUT"; echo "output dir: $OUT"
have=""; n=0
while :; do
  missing=""; for f in +X -X +Y -Y +Z -Z; do case " $have " in *" $f "*) ;; *) missing="$missing $f";; esac; done
  [ -z "$missing" ] && break
  echo; echo "captured: ${have:-none}   still needed:$missing"
  read -r -p "Place the rig on a still-needed face (or the same face rotated to a NEW HEADING), press Enter (q = stop and analyse what we have): " ans
  [ "$ans" = "q" ] && break
  echo "  settling ${SETTLE}s -- hands off"; sleep "$SETTLE"
  n=$((n+1)); f="$OUT/pose_$n.csv"
  echo "  recording ${SECS}s ..."
  "$TOOL" --secs "$SECS" --raw-csv "$f" > "$OUT/pose_$n.log" 2>&1 || { echo "  imx5_read_pimu failed, see $OUT/pose_$n.log"; n=$((n-1)); continue; }
  info="$(python3 "$HERE/six_pos_analyze.py" --label "$f" 2>&1)" || { echo "  analysis failed: $info"; continue; }
  echo "  $info"
  lab="$(echo "$info" | sed -n 's/.*LABEL=\([+-][XYZ]\).*/\1/p')"; still="$(echo "$info" | sed -n 's/.*STILL=\([01]\).*/\1/p')"
  if [ "$still" != "1" ]; then echo "  -> NOT still / not a clean face-up pose: pose $n will be ignored. Re-place the rig and try again."; continue; fi
  case " $have " in *" $lab "*) echo "  -> $lab already captured: this pose is averaged with the earlier one (good if the heading changed)";; *) have="$have $lab";; esac
done
echo; echo "=================== ANALYSIS ==================="
python3 "$HERE/six_pos_analyze.py" "$OUT" | tee "$OUT/summary.txt"
echo; echo "saved: $OUT (pose_*.csv, summary.txt)"
