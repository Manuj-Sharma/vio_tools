#!/usr/bin/env bash
# pretest_disk.sh -- run this BEFORE a recording test / flight day, never during a recording.
#
#     ~/vio_tools/pretest_disk.sh          # asks for your sudo password once (for fstrim)
#
# What it does, and why it is NOT a defragmentation:
#   * The root disk is an SSD (CONSISTENT NVME S7, ext4). Defragmenting an SSD does not help and only
#     wears it. The bags are huge sequential files (filefrag shows a handful of extents each), so
#     fragmentation is not the problem.
#   * What helped (measured 2026-10-28: the ROS-free write test went from 21 stalls up to 6 s to 2 stalls
#     of <0.4 s) was `fstrim`: it tells the SSD which blocks are free after you deleted bags, so it does
#     not have to shuffle dead data around while we write at 55-110 MB/s. Safe: it touches only free blocks.
#   * It also checks free space (an SSD that is nearly full stalls much worse; on 10-27 it was 99% full)
#     and that no recorder is running.
#
# Run it after deleting old bags, wait ~1 minute, then start the sensors / the test.
set -u
MIN_FREE_GB="${MIN_FREE_GB:-40}"
WARN_USED_PCT="${WARN_USED_PCT:-85}"

echo "== pre-test disk check ($(date +%H:%M:%S))"
if pgrep -af 'ros2 bag record' | grep -v pgrep | grep -q . || tmux has-session -t rosbag 2>/dev/null; then
  echo "A bag recorder is running. Not trimming during a recording -- stop it first."; exit 1
fi

free_gb=$(df -BG --output=avail / | tail -1 | tr -dc 0-9)
used_pct=$(df --output=pcent / | tail -1 | tr -dc 0-9)
echo "free space: ${free_gb} GB, used ${used_pct}%"
(( free_gb < MIN_FREE_GB )) && echo "  WARNING: under ${MIN_FREE_GB} GB free -- delete or move old bags first (a 5 min recording at 60 Hz is ~33 GB)."
(( used_pct > WARN_USED_PCT )) && echo "  WARNING: over ${WARN_USED_PCT}% used -- a nearly full SSD stalls badly."

echo
echo "== fstrim (needs sudo): tells the SSD which blocks are free"
if sudo fstrim -v /; then
  echo "done. Give the drive about a minute before starting a test."
else
  echo "fstrim failed or was cancelled."; exit 1
fi

echo
echo "== for reference"
echo "scheduled trim: $(systemctl is-active fstrim.timer 2>/dev/null) (weekly); next: $(systemctl list-timers fstrim.timer --no-legend 2>/dev/null | awk '{print $1, $2, $3}')"
echo "dirty page limits: ratio $(cat /proc/sys/vm/dirty_ratio)%, background $(cat /proc/sys/vm/dirty_background_ratio)%"
