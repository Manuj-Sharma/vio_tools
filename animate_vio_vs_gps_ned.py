#!/usr/bin/env python3
"""animate_vio_vs_gps_ned.py -- animate VIO and GPS positions together on the
same plot, moving in sync, from a vio_gps_ned_logger.py CSV (both already in
the same local NED frame, sharing the same t_sec time base -- no re-syncing
needed). Shown on screen.

NOT the same tool as vs_gps.py / plot_vs_gps.py in this same directory -- see
plot_vio_vs_gps_ned.py's docstring for how this differs (no Umeyama fit, no
separate ground-truth log, single run at a time, alignment already applied
upstream by the real-hardware pipeline).

Usage:
    python3 ~/vio_tools/animate_vio_vs_gps_ned.py ~/rosbags/vio_gps_ned_logs/vio_gps_ned_20260924_160640.csv
    python3 ~/vio_tools/animate_vio_vs_gps_ned.py <csv> --speed 4
    python3 ~/vio_tools/animate_vio_vs_gps_ned.py <csv> -o out.gif
"""
import argparse
import csv
import os
import sys

os.environ.setdefault("DISPLAY", ":0")

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", help="vio_gps_ned_logger.py CSV file")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier (default: 1.0 = real time)")
    ap.add_argument("-o", "--output", default=None,
                     help="also save the animation to this path (default: don't save, just show)")
    args = ap.parse_args()

    csv_path = os.path.expanduser(args.csv)
    needed = ("t_sec", "vio_n", "vio_e", "gps_n", "gps_e")
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

    print(f"{len(rows)} rows, span={t[-1]-t[0]:.1f}s")

    all_e = vio_e + gps_e
    all_n = vio_n + gps_n
    margin = 0.1 * max(max(all_e) - min(all_e), max(all_n) - min(all_n), 1.0)
    xlim = (min(all_e) - margin, max(all_e) + margin)
    ylim = (min(all_n) - margin, max(all_n) + margin)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(vio_e, vio_n, "-", color="#c9d9fb", linewidth=1.2, zorder=1)  # full VIO path, faint
    ax.plot(gps_e, gps_n, "-", color="#fbd0ce", linewidth=1.2, zorder=1)  # full GPS path, faint

    vio_trail, = ax.plot([], [], "-", color="#2f6fed", linewidth=2.5, zorder=3, label="VIO")
    gps_trail, = ax.plot([], [], "-", color="#e5484d", linewidth=2.5, zorder=3, label="GPS")
    vio_point = ax.scatter([], [], color="#2f6fed", s=100, zorder=5)
    gps_point = ax.scatter([], [], color="#e5484d", s=100, zorder=5)
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=11,
                         bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    bag_name = os.path.basename(csv_path)
    ax.set_title(f"VIO vs GPS replay - {bag_name}")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.5, alpha=0.4)
    ax.legend(loc="lower right")

    def update(i):
        vio_trail.set_data(vio_e[: i + 1], vio_n[: i + 1])
        gps_trail.set_data(gps_e[: i + 1], gps_n[: i + 1])
        vio_point.set_offsets([[vio_e[i], vio_n[i]]])
        gps_point.set_offsets([[gps_e[i], gps_n[i]]])
        err = ((vio_e[i] - gps_e[i]) ** 2 + (vio_n[i] - gps_n[i]) ** 2) ** 0.5
        time_text.set_text(f"t = {t[i]-t[0]:5.1f}s\nerr = {err:5.1f}m")
        return vio_trail, gps_trail, vio_point, gps_point, time_text

    dt_ms = [(t[i + 1] - t[i]) * 1000.0 for i in range(len(t) - 1)]
    avg_interval_ms = (sum(dt_ms) / len(dt_ms)) / args.speed if dt_ms else 100.0

    ani = animation.FuncAnimation(fig, update, frames=len(rows), interval=avg_interval_ms, blit=False, repeat=True)

    if args.output:
        print(f"saving to {args.output} (this can take a while)...")
        ani.save(args.output, fps=max(1, int(1000 / avg_interval_ms)))
        print(f"saved: {args.output}")

    plt.show()


if __name__ == "__main__":
    main()
