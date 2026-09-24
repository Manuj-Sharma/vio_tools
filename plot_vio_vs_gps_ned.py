#!/usr/bin/env python3
"""plot_vio_vs_gps_ned.py -- overlay VIO and GPS-derived trajectories (both
already in the same local NED frame) from a vio_gps_ned_logger.py CSV, shown
on screen.

These CSVs live in ~/rosbags/vio_gps_ned_logs/ (see vio_gps_ned_logger.py) --
already anchored to the same point, so no coordinate conversion needed here,
just plot the columns directly.

NOT the same tool as vs_gps.py / plot_vs_gps.py in this same directory. Those
solve a full Umeyama fit (yaw+translation, optionally scale) against a
separate ArduPilot xlsx ground-truth log, built for overlaying many repeats of
a SITL/offline tuning run where VIO's yaw is genuinely unknown. This script is
for the real-hardware pipeline instead: the yaw alignment is ALREADY applied
upstream (the one-time frozen yaw_offset in vio_pose_to_fc_topic.py, logged to
~/rosbags/yaw_alignment_log.csv) and GPS comes from the live rig's own
/fc/gps1, not a separate log file -- no fitting here, just plot what the
pipeline already produced, for one run at a time.

Usage:
    python3 ~/vio_tools/plot_vio_vs_gps_ned.py ~/rosbags/vio_gps_ned_logs/vio_gps_ned_20260924_145538.csv
    python3 ~/vio_tools/plot_vio_vs_gps_ned.py ~/rosbags/vio_gps_ned_logs/vio_gps_ned_latest.csv
    python3 ~/vio_tools/plot_vio_vs_gps_ned.py <csv> -o out.png
"""
import argparse
import csv
import os
import sys

os.environ.setdefault("DISPLAY", ":0")

import matplotlib
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", help="vio_gps_ned_logger.py CSV file")
    ap.add_argument("-o", "--output", default=None,
                     help="also save a PNG to this path (default: don't save, just show)")
    args = ap.parse_args()

    csv_path = os.path.expanduser(args.csv)
    needed = ("t_sec", "vio_n", "vio_e", "gps_n", "gps_e", "err_horiz")
    all_rows = list(csv.DictReader(open(csv_path)))
    rows = [r for r in all_rows if all(r.get(k) not in (None, "") for k in needed)]
    dropped = len(all_rows) - len(rows)
    if dropped:
        print(f"skipped {dropped} incomplete row(s) (e.g. a trailing row cut off mid-write)")
    if len(rows) < 2:
        sys.exit(f"not enough complete rows in {csv_path}")

    t = [float(r["t_sec"]) for r in rows]
    vio_n = [float(r["vio_n"]) for r in rows]
    vio_e = [float(r["vio_e"]) for r in rows]
    gps_n = [float(r["gps_n"]) for r in rows]
    gps_e = [float(r["gps_e"]) for r in rows]
    err = [float(r["err_horiz"]) for r in rows]

    print(f"{len(rows)} rows, span={t[-1]-t[0]:.1f}s, err_horiz min={min(err):.2f} max={max(err):.2f} last={err[-1]:.2f}")

    fig, (ax_traj, ax_err) = plt.subplots(1, 2, figsize=(14, 6.5))

    ax_traj.plot(vio_e, vio_n, "-", color="#2f6fed", linewidth=2, label="VIO", zorder=3)
    ax_traj.plot(gps_e, gps_n, "-", color="#e5484d", linewidth=2, label="GPS", zorder=2)
    ax_traj.scatter([vio_e[0]], [vio_n[0]], color="#22a06b", s=80, zorder=5, label="start")
    ax_traj.set_xlabel("East (m)")
    ax_traj.set_ylabel("North (m)")
    bag_name = os.path.basename(csv_path)
    ax_traj.set_title(f"VIO vs GPS trajectory\n{bag_name}")
    ax_traj.set_aspect("equal", adjustable="box")
    ax_traj.grid(True, linewidth=0.5, alpha=0.4)
    ax_traj.legend()

    ax_err.plot(t, err, "-", color="#2f6fed", linewidth=1.5)
    ax_err.set_xlabel("Time (s)")
    ax_err.set_ylabel("Horizontal error (m)")
    ax_err.set_title(f"|VIO - GPS| over time\nmax={max(err):.1f}m")
    ax_err.grid(True, linewidth=0.5, alpha=0.4)

    plt.tight_layout()

    if args.output:
        plt.savefig(args.output, dpi=130)
        print(f"saved: {args.output}")

    plt.show()


if __name__ == "__main__":
    main()
