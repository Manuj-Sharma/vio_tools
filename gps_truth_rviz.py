#!/usr/bin/env python3
"""gps_truth_rviz.py -- publish the GPS ground-truth track into RViz alongside the VIO.

Run it next to run_openvins_bag.sh. It publishes on /ov_msckf/pathgt by default,
which is the CYAN "Path GT" display already present in display_ros2.rviz and
which nothing else publishes unless a groundtruth file is configured -- so the
truth track appears with no RViz setup at all. Green is the VIO (/ov_msckf/
pathimu); both are in the "global" frame.

WHY THIS IS NOT JUST "PLOT THE GPS"

The VIO global frame is gravity-aligned but its YAW IS ARBITRARY -- VIO cannot
observe absolute heading, so the trajectory comes out rotated by an unknown
constant (measured 168-174 deg on this dataset, and it differs run to run).
Publishing raw ENU would put the truth box at a random angle to the estimate and
look like a catastrophic failure that isn't there.

So this node solves for the alignment against the live VIO path, as vs_gps.py
does offline -- with one deliberate difference:

    YAW and TRANSLATION are applied.  SCALE IS NOT.

Yaw, not a general rotation: both frames are gravity-aligned, so heading is the
only unknown between them. See umeyama_yaw_only().

Scale is the error we are hunting (0.91-0.95 on this dataset). Fitting it away
would hide the very thing worth looking at. Aligned this way, a VIO box that
renders visibly LARGER than the truth box is the scale error, to scale, on
screen. The live scale figure is printed to the terminal instead.

CORRESPONDENCE is by timestamp, not by normalised time. vs_gps.py can use
normalised time because it works on a finished run; here the VIO path is partial
and growing, so stretching it across the whole GPS span would mis-pair the two.
Every pose in nav_msgs/Path carries a stamp on the bag clock, and seg01_fc.xlsx
carries t_bag, so they pair directly.

USAGE
    source /opt/ros/humble/setup.bash
    python3 gps_truth_rviz.py                     # seg01 defaults
    python3 gps_truth_rviz.py --yaw-deg 170       # fixed yaw, skip auto-align
"""
import argparse
import math
import sys

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
except ImportError:
    sys.exit("ROS 2 not sourced -- run: source /opt/ros/humble/setup.bash")

XLSX = "/home/vc/rosbags/seg01_fc.xlsx"

# Alignment is refused below these, because a near-stationary VIO path gives an
# ill-conditioned rotation that would swing the truth box around the screen.
MIN_POSES = 200
MIN_PATH_M = 50.0

# Realignment is a background convenience, not a per-frame need: the yaw offset
# is a constant and settles within a few degrees early on. Doing it on a timer
# off a subsample keeps this node's cost flat and negligible next to the replay.
REALIGN_PERIOD_S = 2.0
MAX_ALIGN_POSES = 800


def load_gps(path, alt_col):
    import pandas as pd
    df = pd.read_excel(path, header=4)
    df = df[pd.to_numeric(df["t_bag"], errors="coerce").notna()]
    for c in ("t_bag", "gps_e_m", "gps_n_m", alt_col):
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=["t_bag", "gps_e_m", "gps_n_m", alt_col])
    t = df["t_bag"].values
    z = df[alt_col].values
    z = z - z[t < t[0] + 1.0].mean()          # same reference convention as the filter
    return t, np.c_[df["gps_e_m"].values, df["gps_n_m"].values, z]


def umeyama_yaw_only(src, dst):
    """Yaw rotation + 3D translation mapping src onto dst. Scale NOT applied.

    The rotation is solved in 2D and lifted to a pure yaw, because both frames
    are gravity-aligned -- the VIO global frame by construction, GPS ENU by
    definition -- so the only unknown between them is heading. A full 3D fit
    would be free to TILT the truth track to absorb altitude error, which is
    physically impossible and would quietly make a vertical problem look like a
    rotation.

    Fitting in 2D also makes the reported scale the HORIZONTAL scale, directly
    comparable to vs_gps.py. A 3D fit blends the (baro-anchored, ~1% correct)
    vertical in with the horizontal and dilutes the very error being measured.

    z is matched by translation only: the mean offset between the two tracks.
    """
    S2, D2 = src[:, :2], dst[:, :2]
    mu_s, mu_d = S2.mean(0), D2.mean(0)
    S, D = S2 - mu_s, D2 - mu_d
    U, sig, Vt = np.linalg.svd(D.T @ S / len(src))
    R2 = U @ Vt
    if np.linalg.det(R2) < 0:                 # reflection guard, as in vs_gps.py
        Vt[-1] *= -1
        sig[-1] *= -1
        R2 = U @ Vt
    scale_would_be = sig.sum() / (S ** 2).sum() * len(src)
    R = np.eye(3)
    R[:2, :2] = R2
    t = np.zeros(3)
    t[:2] = mu_d - R2 @ mu_s
    t[2] = dst[:, 2].mean() - src[:, 2].mean()
    return R, t, scale_would_be


class GpsTruth(Node):
    def __init__(self, args):
        super().__init__("gps_truth_rviz")
        self.frame = args.frame
        self.fixed_yaw = None if args.yaw_deg is None else math.radians(args.yaw_deg)
        self.t_gps, self.P_gps = load_gps(args.xlsx, args.alt_col)
        self.get_logger().info(
            f"loaded {len(self.t_gps)} GPS samples, "
            f"t_bag {self.t_gps[0]:.1f}..{self.t_gps[-1]:.1f}s, "
            f"path {np.linalg.norm(np.diff(self.P_gps[:, :2], axis=0), axis=1).sum():.1f} m")

        latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Path, args.topic, latched)
        self.create_subscription(Path, "/ov_msckf/pathimu", self.on_vio, 2)
        self.create_timer(1.0, self.publish)
        self.R, self.t = np.eye(3), np.zeros(3)
        self.last_align = 0.0
        self.dirty = True
        self.scale, self.n_aligned = float("nan"), 0
        if self.fixed_yaw is not None:
            c, s = math.cos(self.fixed_yaw), math.sin(self.fixed_yaw)
            # GPS -> VIO is the inverse of the VIO -> GPS yaw that vs_gps.py reports
            self.R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
            self.get_logger().info(f"fixed yaw {args.yaw_deg:.1f} deg, auto-align off")

    def on_vio(self, msg):
        if self.fixed_yaw is not None or len(msg.poses) < MIN_POSES:
            return
        # /ov_msckf/pathimu carries the WHOLE path every message, at ~15 Hz, and
        # it grows all run -- so converting it in Python per callback is O(N) work
        # that peaks exactly when the filter is most loaded. Realign on a timer
        # instead, and off a subsample: 800 points already over-determine a yaw
        # and a translation. This runs beside a real-time replay; it must not
        # compete with it.
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_align < REALIGN_PERIOD_S:
            return
        self.last_align = now
        step = max(1, len(msg.poses) // MAX_ALIGN_POSES)
        sel = msg.poses[::step]
        V = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                      for p in sel])
        if np.linalg.norm(np.diff(V[:, :2], axis=0), axis=1).sum() < MIN_PATH_M:
            return
        tv = np.array([p.header.stamp.sec + p.header.stamp.nanosec * 1e-9
                       for p in sel])
        if tv[-1] <= tv[0]:
            return
        # pair by bag timestamp; only where GPS actually covers the VIO span
        m = (tv >= self.t_gps[0]) & (tv <= self.t_gps[-1])
        if m.sum() < 50:          # subsampled, so the full-path threshold no longer applies
            return
        G = np.c_[tuple(np.interp(tv[m], self.t_gps, self.P_gps[:, i]) for i in range(3))]
        # solve GPS -> VIO so the truth moves onto the estimate, leaving the
        # estimate untouched: what you see in RViz is the raw filter output.
        self.R, self.t, self.scale = umeyama_yaw_only(G, V[m])
        self.n_aligned = int(m.sum())
        self.dirty = True

    def publish(self):
        # The topic is latched (TRANSIENT_LOCAL), so a subscriber that joins late
        # -- RViz started after this node, or restarted -- still receives the last
        # message. Rebuilding 2000+ PoseStamped objects every second to republish
        # identical data would just steal cycles from the replay next to us.
        if not self.dirty:
            self.log_state()
            return
        self.dirty = False
        P = (self.R @ self.P_gps.T).T + self.t
        msg = Path()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for xyz in P:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, xyz)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub.publish(msg)
        self.log_state()

    def log_state(self):
        if self.n_aligned:
            yaw = math.degrees(math.atan2(self.R[1, 0], self.R[0, 0]))
            # self.scale maps GPS onto VIO, so it IS the VIO/truth size ratio and
            # a VIO that is too big gives scale > 1. vs_gps.py solves the other
            # direction and reports the reciprocal -- the factor the VIO must be
            # shrunk by. Print that one too, so the two tools can be compared
            # directly; they must agree or one of them is wrong. Likewise the yaw
            # here is GPS->VIO, i.e. the negative of the figure vs_gps.py prints.
            self.get_logger().info(
                f"aligned on {self.n_aligned} poses | yaw(GPS->VIO) {yaw:+7.1f} deg | "
                f"scale {1.0/self.scale:.3f} (vs_gps convention) | "
                f"VIO is {100*(self.scale-1):+.1f}% too big", throttle_duration_sec=5.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xlsx", default=XLSX)
    ap.add_argument("--alt-col", default="alt_baro_m",
                    help="alt_baro_m (default, agrees with the filter's baro) or gps_alt_m")
    ap.add_argument("--frame", default="global")
    ap.add_argument("--topic", default="/ov_msckf/pathgt",
                    help="default is the CYAN 'Path GT' display already in "
                         "display_ros2.rviz, which nothing else publishes -- so the "
                         "truth track appears with no RViz configuration at all")
    ap.add_argument("--yaw-deg", type=float, default=None,
                    help="fix the GPS->VIO yaw instead of solving for it live")
    args = ap.parse_args()
    rclpy.init()
    node = GpsTruth(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
