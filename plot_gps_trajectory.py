#!/usr/bin/env python3
"""plot_gps_trajectory.py -- plot /fc/gps1 (local east/north trajectory) and
/fc/ekf_yaw (heading over time) from a recorded rosbag, shown on screen.

Reads directly from the bag's sqlite3 file -- no ROS node, no `ros2 bag play`
needed, so it's safe to run even while a live stack or another replay is using
the same bag (pure read, no conflict).

Shows the plot interactively (needs a display -- DISPLAY defaults to :0, same
as this project's other GUI tools). Does NOT save a file unless -o is given.

Usage:
    python3 ~/vio_tools/plot_gps_trajectory.py ~/rosbags/vio_20260924_134543
    python3 ~/vio_tools/plot_gps_trajectory.py ~/rosbags/vio_20260924_134543 -o /tmp/out.png
    python3 ~/vio_tools/plot_gps_trajectory.py ~/rosbags/vio_20260924_134543 \
        --gps-topic /fc/gps1 --yaw-topic /fc/ekf_yaw
"""
import argparse
import bisect
import math
import os
import sqlite3
import sys

os.environ.setdefault("DISPLAY", ":0")

import matplotlib
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def meters_per_degree(lat0_deg):
    """Flat-earth approximation, same formula used by vio_gps_ned_logger.py
    elsewhere in this project -- valid to sub-cm error at single-walk scale."""
    lat0 = math.radians(lat0_deg)
    m_per_lat = 111132.92 - 559.82 * math.cos(2 * lat0) + 1.175 * math.cos(4 * lat0)
    m_per_lon = 111412.84 * math.cos(lat0) - 93.5 * math.cos(3 * lat0)
    return m_per_lat, m_per_lon


def find_db3(bag_dir):
    if os.path.isfile(bag_dir) and bag_dir.endswith(".db3"):
        return bag_dir
    candidates = [f for f in os.listdir(bag_dir) if f.endswith(".db3")]
    if not candidates:
        sys.exit(f"no .db3 file found in {bag_dir}")
    return os.path.join(bag_dir, candidates[0])


def nearest_yaw(yaw_ts, yaw_vals, query_ts):
    """Nearest-timestamp lookup (yaw_ts must be sorted ascending)."""
    i = bisect.bisect_left(yaw_ts, query_ts)
    if i == 0:
        return yaw_vals[0]
    if i == len(yaw_ts):
        return yaw_vals[-1]
    before, after = yaw_ts[i - 1], yaw_ts[i]
    return yaw_vals[i - 1] if (query_ts - before) <= (after - query_ts) else yaw_vals[i]


def read_topic(cur, topic, msg_type_name):
    cur.execute("SELECT id FROM topics WHERE name=?", (topic,))
    row = cur.fetchone()
    if row is None:
        return None
    topic_id = row[0]
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (topic_id,))
    rows = cur.fetchall()
    msg_type = get_message(msg_type_name)
    return [(ts, deserialize_message(data, msg_type)) for ts, data in rows]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", help="bag directory (or a .db3 file directly)")
    ap.add_argument("--gps-topic", default="/fc/gps1", help="NavSatFix topic (default: /fc/gps1)")
    ap.add_argument("--yaw-topic", default="/fc/ekf_yaw", help="Float32 yaw topic, radians (default: /fc/ekf_yaw)")
    ap.add_argument("-o", "--output", default=None,
                     help="also save a PNG to this path (default: don't save, just show)")
    ap.add_argument("--num-arrows", type=int, default=25,
                     help="number of heading arrows to draw along the GPS path (default: 25)")
    args = ap.parse_args()

    bag_dir = os.path.expanduser(args.bag)
    db3_path = find_db3(bag_dir)
    bag_name = os.path.basename(os.path.normpath(bag_dir))

    conn = sqlite3.connect(db3_path)
    cur = conn.cursor()
    gps_rows = read_topic(cur, args.gps_topic, "sensor_msgs/msg/NavSatFix")
    yaw_rows = read_topic(cur, args.yaw_topic, "std_msgs/msg/Float32")
    conn.close()

    if gps_rows is None:
        sys.exit(f"topic {args.gps_topic} not found in {db3_path}")
    if yaw_rows is None:
        sys.exit(f"topic {args.yaw_topic} not found in {db3_path}")

    fig, (ax_gps, ax_heading, ax_yaw) = plt.subplots(1, 3, figsize=(19, 6.5))

    # --- GPS trajectory (local east/north, anchored to first valid fix) ---
    pts = [(ts, m.latitude, m.longitude) for ts, m in gps_rows if m.status.status >= 0]
    print(f"GPS: {len(pts)} valid fixes (of {len(gps_rows)} total messages)")
    if len(pts) < 2:
        print("not enough valid GPS fixes to plot trajectory")
    else:
        lat0, lon0 = pts[0][1], pts[0][2]
        m_per_lat, m_per_lon = meters_per_degree(lat0)
        north = [(lat - lat0) * m_per_lat for _, lat, lon in pts]
        east = [(lon - lon0) * m_per_lon for _, lat, lon in pts]
        gps_ts = [ts for ts, _, _ in pts]

        ax_gps.plot(east, north, "-", color="#2f6fed", linewidth=2)
        ax_gps.scatter([east[0]], [north[0]], color="#22a06b", s=80, zorder=5, label="start")
        ax_gps.scatter([east[-1]], [north[-1]], color="#e5484d", s=80, zorder=5, label="end")
        ax_gps.set_xlabel("East (m)")
        ax_gps.set_ylabel("North (m)")
        ax_gps.set_title(f"GPS trajectory\n{len(pts)} fixes, anchor=({lat0:.6f},{lon0:.6f})")
        ax_gps.set_aspect("equal", adjustable="box")
        ax_gps.grid(True, linewidth=0.5, alpha=0.4)
        ax_gps.legend()
        print(f"GPS bounding box: east [{min(east):.1f},{max(east):.1f}]  north [{min(north):.1f},{max(north):.1f}]")

        # --- Heading vectors along the path (separate plot) ---
        if len(yaw_rows) < 2:
            ax_heading.set_title("Heading vectors\n(no yaw data)")
        else:
            yaw_ts = [ts for ts, _ in yaw_rows]
            yaw_vals = [m.data for _, m in yaw_rows]  # radians, NED: 0=north, +=clockwise toward east

            n = min(args.num_arrows, len(pts))
            idx = sorted(set(round(i) for i in
                              [k * (len(pts) - 1) / max(1, n - 1) for k in range(n)]))

            arrow_len = 0.06 * max(max(east) - min(east), max(north) - min(north), 1.0)
            ax_heading.plot(east, north, "-", color="#9aa5b1", linewidth=1.5, zorder=1)
            for i in idx:
                yaw = nearest_yaw(yaw_ts, yaw_vals, gps_ts[i])
                dx = arrow_len * math.sin(yaw)
                dy = arrow_len * math.cos(yaw)
                ax_heading.annotate("", xy=(east[i] + dx, north[i] + dy), xytext=(east[i], north[i]),
                                     arrowprops=dict(arrowstyle="-|>", color="#e5484d", lw=1.5), zorder=5)
            ax_heading.scatter([east[0]], [north[0]], color="#22a06b", s=80, zorder=6, label="start")
            ax_heading.scatter([east[-1]], [north[-1]], color="#e5484d", s=80, zorder=6, label="end")
            ax_heading.set_xlabel("East (m)")
            ax_heading.set_ylabel("North (m)")
            ax_heading.set_title(f"Heading vectors along path\n({len(idx)} arrows, from {args.yaw_topic})")
            ax_heading.set_aspect("equal", adjustable="box")
            ax_heading.grid(True, linewidth=0.5, alpha=0.4)
            ax_heading.legend()

    # --- EKF yaw over time ---
    print(f"yaw: {len(yaw_rows)} samples")
    if len(yaw_rows) < 2:
        print("not enough yaw samples to plot")
    else:
        t0_ns = yaw_rows[0][0]
        t_yaw = [(ts - t0_ns) / 1e9 for ts, _ in yaw_rows]
        yaw_deg = [math.degrees(m.data) for _, m in yaw_rows]

        ax_yaw.plot(t_yaw, yaw_deg, "-", color="#2f6fed", linewidth=1.5)
        ax_yaw.set_xlabel("Time (s)")
        ax_yaw.set_ylabel("Yaw (deg)")
        ax_yaw.set_title(f"EKF yaw\n{len(yaw_rows)} samples")
        ax_yaw.set_ylim(-185, 185)
        ax_yaw.grid(True, linewidth=0.5, alpha=0.4)

    fig.suptitle(bag_name)
    plt.tight_layout()

    if args.output:
        plt.savefig(args.output, dpi=130)
        print(f"saved: {args.output}")

    plt.show()


if __name__ == "__main__":
    main()
