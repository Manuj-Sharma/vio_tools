#!/usr/bin/env python3
"""animate_gps_trajectory.py -- animate /fc/gps1 position + /fc/ekf_yaw heading
over time from a recorded rosbag, shown on screen.

Reads directly from the bag's sqlite3 file -- no ROS node, no `ros2 bag play`
needed, so it's safe to run even while a live stack or another replay is using
the same bag (pure read, no conflict).

Shows the animation interactively (needs a display -- DISPLAY defaults to :0,
same as this project's other GUI tools). Does NOT save a file unless -o is
given (requires ffmpeg or pillow for the writer, depending on extension).

Usage:
    python3 ~/vio_tools/animate_gps_trajectory.py ~/rosbags/vio_20260924_134543
    python3 ~/vio_tools/animate_gps_trajectory.py ~/rosbags/vio_20260924_134543 --speed 4
    python3 ~/vio_tools/animate_gps_trajectory.py ~/rosbags/vio_20260924_134543 -o out.mp4
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
import matplotlib.animation as animation

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
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier (default: 1.0 = real time)")
    ap.add_argument("-o", "--output", default=None,
                     help="also save the animation to this path (default: don't save, just show)")
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

    pts = [(ts, m.latitude, m.longitude) for ts, m in gps_rows if m.status.status >= 0]
    print(f"GPS: {len(pts)} valid fixes (of {len(gps_rows)} total messages)")
    if len(pts) < 2:
        sys.exit("not enough valid GPS fixes to animate")

    lat0, lon0 = pts[0][1], pts[0][2]
    m_per_lat, m_per_lon = meters_per_degree(lat0)
    north = [(lat - lat0) * m_per_lat for _, lat, lon in pts]
    east = [(lon - lon0) * m_per_lon for _, lat, lon in pts]
    gps_ts = [ts for ts, _, _ in pts]
    t_rel = [(ts - gps_ts[0]) / 1e9 for ts in gps_ts]

    yaw_ts = [ts for ts, _ in yaw_rows]
    yaw_vals = [m.data for _, m in yaw_rows]
    yaws_at_gps = [nearest_yaw(yaw_ts, yaw_vals, ts) for ts in gps_ts]

    margin = 0.1 * max(max(east) - min(east), max(north) - min(north), 1.0)
    xlim = (min(east) - margin, max(east) + margin)
    ylim = (min(north) - margin, max(north) + margin)
    arrow_len = 0.05 * max(max(east) - min(east), max(north) - min(north), 1.0)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(east, north, "-", color="#c8ccd2", linewidth=1.5, zorder=1)  # full path, faint
    trail_line, = ax.plot([], [], "-", color="#2f6fed", linewidth=2.5, zorder=3)
    point = ax.scatter([], [], color="#2f6fed", s=100, zorder=5)
    heading_arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                                 arrowprops=dict(arrowstyle="-|>", color="#e5484d", lw=2.5), zorder=6)
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=11,
                         bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"GPS trajectory replay - {bag_name}")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.5, alpha=0.4)

    def update(i):
        trail_line.set_data(east[: i + 1], north[: i + 1])
        point.set_offsets([[east[i], north[i]]])
        yaw = yaws_at_gps[i]
        dx = arrow_len * math.sin(yaw)
        dy = arrow_len * math.cos(yaw)
        heading_arrow.set_position((east[i], north[i]))
        heading_arrow.xy = (east[i] + dx, north[i] + dy)
        time_text.set_text(f"t = {t_rel[i]:5.1f}s\nyaw = {math.degrees(yaw):6.1f} deg")
        return trail_line, point, heading_arrow, time_text

    # Real interval between consecutive GPS samples, scaled by --speed.
    dt_ms = [(t_rel[i + 1] - t_rel[i]) * 1000.0 for i in range(len(t_rel) - 1)]
    avg_interval_ms = (sum(dt_ms) / len(dt_ms)) / args.speed if dt_ms else 100.0

    ani = animation.FuncAnimation(fig, update, frames=len(pts), interval=avg_interval_ms, blit=False, repeat=True)

    if args.output:
        print(f"saving to {args.output} (this can take a while)...")
        ani.save(args.output, fps=max(1, int(1000 / avg_interval_ms)))
        print(f"saved: {args.output}")

    plt.show()


if __name__ == "__main__":
    main()
