#!/usr/bin/env python3
"""plot_vio_trajectory.py -- plot /ov_msckf/odomimu (VIO position, converted to
the same NED frame used elsewhere in this project) as a local east/north
trajectory, with heading vectors from VIO's OWN orientation estimate, shown on
screen. Same layout as plot_gps_trajectory.py, but for VIO instead of GPS --
useful for comparing the two side by side, or for checking VIO's own yaw output
against where it's actually pointing (see the vio_pose_to_fc_topic.py yaw-
alignment investigation from 2026-09-24: VIO's own aligned yaw was found
reading ~180deg opposite to the Cube compass in one run, despite VIO's
*position* tracking direction correctly -- position and orientation go through
separate conversion code paths, see vio_frame_convert.py).

Reads directly from the bag's sqlite3 file -- no ROS node, no `ros2 bag play`
needed, so it's safe to run even while a live stack or another replay is using
the same bag (pure read, no conflict). Also works on a bag with no
metadata.yaml (e.g. after an unclean shutdown) since it never needs that file.

The yaw_offset (the one-time frozen alignment value -- see vio_frame_convert.py)
is required to convert VIO's own arbitrary-at-boot frame into true NED. By
default this looks up the closest-in-time entry in
~/rosbags/yaw_alignment_log.csv (populated by vio_pose_to_fc_topic.py since
2026-09-24); pass --yaw-offset-deg to override.

Usage:
    python3 ~/vio_tools/plot_vio_trajectory.py ~/rosbags/vio_20260924_160639
    python3 ~/vio_tools/plot_vio_trajectory.py ~/rosbags/vio_20260924_160639 --yaw-offset-deg 93.423
    python3 ~/vio_tools/plot_vio_trajectory.py <bag> -o out.png
"""
import argparse
import bisect
import csv
import math
import os
import sqlite3
import sys

os.environ.setdefault("DISPLAY", ":0")

import matplotlib
import matplotlib.pyplot as plt

# vio_frame_convert.py lives in the real-hardware repo, not here -- import it
# directly by path rather than duplicating the (carefully verified) conversion
# math a third time.
sys.path.insert(0, os.path.expanduser("~/vio_ar0234_icm20948/vio_bringup"))
from vio_frame_convert import enu_pos_to_ned, enu_quat_to_ned, apply_yaw_alignment  # noqa: E402

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

YAW_ALIGNMENT_LOG = os.path.expanduser("~/rosbags/yaw_alignment_log.csv")


def find_db3(bag_dir):
    if os.path.isfile(bag_dir) and bag_dir.endswith(".db3"):
        return bag_dir
    candidates = [f for f in os.listdir(bag_dir) if f.endswith(".db3")]
    if not candidates:
        sys.exit(f"no .db3 file found in {bag_dir}")
    return os.path.join(bag_dir, candidates[0])


def lookup_yaw_offset_deg(bag_dir):
    """Nearest entry in yaw_alignment_log.csv to this bag's own timestamp
    (encoded in its directory name, vio_YYYYMMDD_HHMMSS), by wall-clock
    distance. Approximate -- the log has no bag-name field, just a timestamp --
    good enough given entries are normally many minutes apart."""
    import datetime
    bag_name = os.path.basename(os.path.normpath(bag_dir))
    try:
        stamp = bag_name.split("vio_")[1]
        bag_time = datetime.datetime.strptime(stamp, "%Y%m%d_%H%M%S")
    except (IndexError, ValueError):
        return None
    if not os.path.exists(YAW_ALIGNMENT_LOG):
        return None
    best = None
    best_dt = None
    with open(YAW_ALIGNMENT_LOG) as f:
        for row in csv.DictReader(f):
            try:
                t = datetime.datetime.strptime(row["wall_clock_iso"], "%Y-%m-%dT%H:%M:%S")
            except ValueError:
                continue
            dt = abs((t - bag_time).total_seconds())
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best = float(row["yaw_offset_deg"])
    if best is not None:
        print(f"using yaw_offset={best:.3f} deg from {YAW_ALIGNMENT_LOG} ({best_dt:.0f}s from bag start)")
    return best


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
    ap.add_argument("--odom-topic", default="/ov_msckf/odomimu", help="Odometry topic (default: /ov_msckf/odomimu)")
    ap.add_argument("--yaw-offset-deg", type=float, default=None,
                     help="override the yaw-alignment offset in degrees (default: look up ~/rosbags/yaw_alignment_log.csv)")
    ap.add_argument("--num-arrows", type=int, default=25, help="number of heading arrows to draw (default: 25)")
    ap.add_argument("-o", "--output", default=None,
                     help="also save a PNG to this path (default: don't save, just show)")
    args = ap.parse_args()

    bag_dir = os.path.expanduser(args.bag)
    db3_path = find_db3(bag_dir)
    bag_name = os.path.basename(os.path.normpath(bag_dir))

    yaw_offset_deg = args.yaw_offset_deg
    if yaw_offset_deg is None:
        yaw_offset_deg = lookup_yaw_offset_deg(bag_dir)
    if yaw_offset_deg is None:
        sys.exit("no yaw_offset available -- pass --yaw-offset-deg explicitly")
    yaw_offset = math.radians(yaw_offset_deg)

    conn = sqlite3.connect(db3_path)
    cur = conn.cursor()
    odom_rows = read_topic(cur, args.odom_topic, "nav_msgs/msg/Odometry")
    conn.close()

    if odom_rows is None:
        sys.exit(f"topic {args.odom_topic} not found in {db3_path}")
    print(f"VIO: {len(odom_rows)} odometry samples")
    if len(odom_rows) < 2:
        sys.exit("not enough samples to plot")

    north, east, yaws = [], [], []
    for ts, m in odom_rows:
        p = m.pose.pose.position
        x_ned, y_ned, _ = enu_pos_to_ned(p.x, p.y, p.z)
        q = m.pose.pose.orientation
        _, _, yaw = enu_quat_to_ned(q.x, q.y, q.z, q.w)
        x_aligned, y_aligned, yaw_aligned = apply_yaw_alignment(x_ned, y_ned, yaw, yaw_offset)
        north.append(x_aligned)
        east.append(y_aligned)
        yaws.append(yaw_aligned)

    print(f"VIO bounding box: east [{min(east):.1f},{max(east):.1f}]  north [{min(north):.1f},{max(north):.1f}]")

    fig, (ax_traj, ax_heading) = plt.subplots(1, 2, figsize=(13, 6.5))

    ax_traj.plot(east, north, "-", color="#2f6fed", linewidth=2)
    ax_traj.scatter([east[0]], [north[0]], color="#22a06b", s=80, zorder=5, label="start")
    ax_traj.scatter([east[-1]], [north[-1]], color="#e5484d", s=80, zorder=5, label="end")
    ax_traj.set_xlabel("East (m)")
    ax_traj.set_ylabel("North (m)")
    ax_traj.set_title(f"VIO trajectory\n{len(odom_rows)} samples, yaw_offset={yaw_offset_deg:.1f} deg")
    ax_traj.set_aspect("equal", adjustable="box")
    ax_traj.grid(True, linewidth=0.5, alpha=0.4)
    ax_traj.legend()

    n = min(args.num_arrows, len(north))
    idx = sorted(set(round(i) for i in [k * (len(north) - 1) / max(1, n - 1) for k in range(n)]))
    arrow_len = 0.06 * max(max(east) - min(east), max(north) - min(north), 1.0)
    ax_heading.plot(east, north, "-", color="#9aa5b1", linewidth=1.5, zorder=1)
    for i in idx:
        dx = arrow_len * math.sin(yaws[i])
        dy = arrow_len * math.cos(yaws[i])
        ax_heading.annotate("", xy=(east[i] + dx, north[i] + dy), xytext=(east[i], north[i]),
                             arrowprops=dict(arrowstyle="-|>", color="#e5484d", lw=1.5), zorder=5)
    ax_heading.scatter([east[0]], [north[0]], color="#22a06b", s=80, zorder=6, label="start")
    ax_heading.scatter([east[-1]], [north[-1]], color="#e5484d", s=80, zorder=6, label="end")
    ax_heading.set_xlabel("East (m)")
    ax_heading.set_ylabel("North (m)")
    ax_heading.set_title(f"VIO's own heading along path\n({len(idx)} arrows, from VIO orientation)")
    ax_heading.set_aspect("equal", adjustable="box")
    ax_heading.grid(True, linewidth=0.5, alpha=0.4)
    ax_heading.legend()

    fig.suptitle(bag_name)
    plt.tight_layout()

    if args.output:
        plt.savefig(args.output, dpi=130)
        print(f"saved: {args.output}")

    plt.show()


if __name__ == "__main__":
    main()
