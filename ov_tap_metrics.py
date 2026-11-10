#!/usr/bin/env python3
"""Quantify how much a tap/knock disturbs OpenVINS: numbers instead of looking at RViz.

  ov_tap_metrics.py RAW_BAG OV_BAG [--label TEXT] [--csv OUT.csv] [--tap-t S] [--tap-thresh 15] [--settle 2.0]
RAW_BAG : bag with the raw 1 kHz /imu0 (the one that was replayed) -- used only to find when the taps happen.
OV_BAG  : bag with /ov_msckf/odomimu of the replay (vio_replay_with_filter.sh with REC_OUT=..., or a live recording).
Needs ROS sourced; run with python3 (NOT -I).

Assumes the rig sits still on a table and is tapped (any estimated motion is then error). If the rig is moving, read the
"before" numbers as the baseline and compare only the change at the tap.
Metrics, all in the bag time base (stamps are shared by IMU and odomimu):
  tap times      first sample where |accel - median| > thresh (m/s^2) and further clusters (>= 1 s apart); peak of each
  before tap     max speed |v|, max displacement from the first pose
  after tap      max speed within 1 s, max speed in the LAST 5 s (should return to ~0 if it recovers),
                 displacement growth: |p(end) - p(tap+settle)|  (settle seconds after the tap),  max |p - p_before_tap|
  health         any NaN/inf in the estimate, total path length, final distance from the start
"""
import sys, glob, os, sqlite3, argparse
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

ap = argparse.ArgumentParser()
ap.add_argument("raw_bag"); ap.add_argument("ov_bag")
ap.add_argument("--label", default=""); ap.add_argument("--csv")
ap.add_argument("--tap-t", type=float, nargs="*", help="tap time(s), seconds from the first IMU stamp (skip auto-detection)")
ap.add_argument("--tap-thresh", type=float, default=15.0); ap.add_argument("--settle", type=float, default=2.0)
ap.add_argument("--imu-topic", default="/imu0"); ap.add_argument("--odom-topic", default="/ov_msckf/odomimu")
a = ap.parse_args()

def rows(bag, topic):
    for db in sorted(glob.glob(os.path.join(bag, "*.db3"))):
        c = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
        r = c.execute("select id from topics where name=?", (topic,)).fetchone()
        if not r: continue
        for (b,) in c.execute("select data from messages where topic_id=? order by timestamp", (r[0],)): yield b

ti, ai = [], []
for b in rows(a.raw_bag, a.imu_topic):
    m = deserialize_message(bytes(b), Imu); ti.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    ai.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
if len(ti) < 100: sys.exit(f"no {a.imu_topic} in {a.raw_bag}")
ti = np.array(ti); ai = np.array(ai); t0 = ti[0]; ti -= t0
dev = np.linalg.norm(ai - np.median(ai, 0), axis=1)

if a.tap_t: taps = list(a.tap_t); peaks = [float(dev[np.argmin(abs(ti - x))]) for x in taps]
else:
    idx = np.where(dev > a.tap_thresh)[0]; taps, peaks = [], []
    for i in idx:
        if taps and ti[i] - taps[-1] < 1.0: peaks[-1] = max(peaks[-1], float(dev[i]))
        else: taps.append(float(ti[i])); peaks.append(float(dev[i]))
if not taps: sys.exit(f"no tap above {a.tap_thresh} m/s^2 in the raw IMU (lower --tap-thresh or pass --tap-t)")

to, po, vo = [], [], []
for b in rows(a.ov_bag, a.odom_topic):
    m = deserialize_message(bytes(b), Odometry); to.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - t0)
    p = m.pose.pose.position; v = m.twist.twist.linear; po.append([p.x, p.y, p.z]); vo.append([v.x, v.y, v.z])
if len(to) < 10: sys.exit(f"only {len(to)} {a.odom_topic} messages in {a.ov_bag}")
to = np.array(to); po = np.array(po); vo = np.array(vo)
finite = np.isfinite(po).all(1) & np.isfinite(vo).all(1)
nbad = int((~finite).sum()); first_bad = float(to[~finite][0]) if nbad else None
to, po, vo = to[finite], po[finite], vo[finite]
sp = np.linalg.norm(vo, axis=1); d0 = np.linalg.norm(po - po[0], axis=1)
T0 = taps[0]; pre = to < T0 - 0.2; post = to > T0
print(f"label: {a.label or '-'}   raw {os.path.basename(a.raw_bag.rstrip('/'))}   odom {os.path.basename(a.ov_bag.rstrip('/'))}")
print(f"taps (s, peak |a-median| m/s^2): " + ", ".join(f"{t:.1f} ({p:.0f})" for t, p in zip(taps, peaks)))
print(f"odom: {len(to)} msgs, {to[0]:.1f}..{to[-1]:.1f} s   non-finite: {nbad}" + (f" (first at {first_bad:.1f} s)" if nbad else ""))
if pre.sum() > 5: print(f"before 1st tap : max speed {sp[pre].max():.3f} m/s, max displacement {d0[pre].max():.3f} m")
else: print("before 1st tap : (no odometry before the tap)")
w1 = (to >= T0) & (to <= T0 + 1.0); last5 = to >= to[-1] - 5.0
pt = po[np.argmin(abs(to - (T0 + a.settle)))]
base = po[pre][-1] if pre.sum() > 5 else po[np.argmin(abs(to - T0))]
print(f"after 1st tap  : max speed in 1 s {sp[w1].max() if w1.any() else float('nan'):.3f} m/s | speed over the last 5 s: median {np.median(sp[last5]):.3f}, max {sp[last5].max():.3f} m/s")
print(f"                 displacement: max |p - p_before_tap| {np.linalg.norm(po[post] - base, axis=1).max():.3f} m | |p_end - p(tap+{a.settle:g}s)| {np.linalg.norm(po[-1] - pt):.3f} m")
print(f"overall        : path length {np.linalg.norm(np.diff(po, axis=0), axis=1).sum():.3f} m, final distance from start {d0[-1]:.3f} m")
if a.csv:
    new = not os.path.exists(a.csv)
    with open(a.csv, "a") as f:
        if new: f.write("label,odom_bag,ntaps,tap_peak_max,nonfinite,pre_max_speed,post_max_speed_1s,last5_speed_median,max_disp_after_tap,drift_after_settle,final_dist\n")
        f.write(f"{a.label},{os.path.basename(a.ov_bag.rstrip('/'))},{len(taps)},{max(peaks):.1f},{nbad},{sp[pre].max() if pre.sum()>5 else float('nan'):.4f},"
                f"{sp[w1].max() if w1.any() else float('nan'):.4f},{np.median(sp[last5]):.4f},{np.linalg.norm(po[post]-base,axis=1).max():.4f},{np.linalg.norm(po[-1]-pt):.4f},{d0[-1]:.4f}\n")
    print("csv:", a.csv)
