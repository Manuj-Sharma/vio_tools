#!/usr/bin/env bash
# cam_cycle_test.sh [N=10] - stop/start the sensor stack N times (no reboot) and log, per cycle,
# whether cam0/cam1 are still the same physical cameras.
#
# Per cycle:  stop (stop_vio_stack.sh + kill tmux 'dualcam') -> wait until nodes are gone ->
#             gap -> cam_order_check.sh (serial at SDK index 0/1) -> start_dual_cam.sh ->
#             wait for both nodes + a frame on /cam0 and /cam1 -> read which /dev/videoN each
#             node holds open (from /proc/<pid>/fd) and compare with the path the check saw.
# Gaps: first half of the cycles 10 s, second half 2 s (quick restart = the risky case).
# Run in your own terminal, with the stack stopped. Leaves the stack STOPPED at the end.
# Log: ~/vio_tools/records/cam_cycle_log.csv (cam_order_check.sh also appends its own rows).
set -u
N="${1:-10}"
REPO="${REPO:-$HOME/vio_ar0234_icm20948}"
TOOLS="$HOME/vio_tools"
LOG="$TOOLS/records/cam_cycle_log.csv"
ORDER_LOG="$TOOLS/records/cam_order_log.csv"
mkdir -p "$TOOLS/records"
[ -s "$LOG" ] || echo "time,cycle,gap_s,stop_s,order_check,check_cam0_path,check_cam1_path,node_cam0_holds,node_cam1_holds,nodes_match_check,cam0_frame,cam1_frame,note" > "$LOG"

set +u; source /opt/ros/humble/setup.bash; source "$REPO/install/setup.bash" 2>/dev/null; set -u

nodes_up()   { pgrep -x vizionsdk_camer >/dev/null || pgrep -x imx5_spi_node >/dev/null; }

stop_all() {
  "$REPO/stop_vio_stack.sh" >/dev/null 2>&1
  tmux kill-session -t dualcam 2>/dev/null
  local t0=$SECONDS
  while nodes_up; do
    [ $((SECONDS - t0)) -gt 30 ] && { pkill -9 vizionsdk_camer; pkill -9 imx5_spi_node; sleep 1; break; }
    sleep 0.5
  done
  STOP_S=$((SECONDS - t0))
}

# /dev/videoN held open by the node whose cmdline matches $1 (cam0: node, cam1: node_cam1)
held_video() {
  local pat="$1" pid dev
  for pid in $(pgrep -x vizionsdk_camer); do
    local exe; exe="$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null | awk '{print $1}')"
    case "$pat" in
      cam1) [[ "$exe" == *camera_node_cam1* ]] || continue ;;
      cam0) [[ "$exe" == *camera_node_cam1* ]] && continue ;;
    esac
    dev="$(ls -l /proc/$pid/fd 2>/dev/null | grep -o '/dev/video[0-9]*' | sort -u | tr '\n' '+' | sed 's/+$//')"
    echo "${dev:-none}"; return
  done
  echo "nonode"
}

frame_ok() { # $1 topic
  local t0=$SECONDS
  while [ $((SECONDS - t0)) -lt 45 ]; do
    timeout 6 ros2 topic echo --once --no-arr --field header.frame_id "$1" >/dev/null 2>&1 && { echo yes; return; }
  done
  echo no
}

echo "cam_cycle_test: $N cycles; stack will be stopped first"
stop_all
for i in $(seq 1 "$N"); do
  [ "$i" -le $(( (N + 1) / 2 )) ] && GAP=10 || GAP=2
  NOTE=""
  stop_all
  sleep "$GAP"

  CHK="$("$TOOLS/cam_order_check.sh" 2>&1 | tail -1)"; RC=$?
  case "$CHK" in SAME*) CHK=SAME ;; \*\*\**) CHK=SWAPPED ;; *) NOTE="check: $CHK"; CHK=ERROR ;; esac
  P0="$(tail -2 "$ORDER_LOG" | awk -F, '$4==0{print $5}')"
  P1="$(tail -2 "$ORDER_LOG" | awk -F, '$4==1{print $5}')"

  "$REPO/start_dual_cam.sh" >/dev/null 2>&1
  t0=$SECONDS
  while [ "$(pgrep -cx vizionsdk_camer)" -lt 2 ]; do
    [ $((SECONDS - t0)) -gt 60 ] && { NOTE="$NOTE; <2 camera nodes after 60s"; break; }
    sleep 1
  done
  F0="$(frame_ok /cam0/image_raw)"; F1="$(frame_ok /cam1/image_raw)"
  H0="$(held_video cam0)"; H1="$(held_video cam1)"
  [ "$H0" = "$P0" ] && [ "$H1" = "$P1" ] && M=yes || M=NO

  echo "$(date -Iseconds),$i,$GAP,$STOP_S,$CHK,$P0,$P1,$H0,$H1,$M,$F0,$F1,$NOTE" | tee -a "$LOG"
done
stop_all
echo "done. stack stopped. log: $LOG"
awk -F, 'NR>1{c[$5]++; m[$10]++} END{for(k in c) print "order_check " k ": " c[k]; for(k in m) print "nodes_match_check " k ": " m[k]}' "$LOG"
