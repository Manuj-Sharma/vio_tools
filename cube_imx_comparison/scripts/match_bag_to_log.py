#!/usr/bin/env python3
"""Find which Cube log a rosbag belongs to, by correlating the SIGNED yaw rate (gyro Z) of /imu0 with each log's IMU[0].

usage: match_bag_to_log.py BAG_DIR [--logs DIR] [--start 30] [--stop 400] [--topic /imu0]
  --logs   directory holding logNNN_imu_att.npz files made by extract_cube_imu.py (default: the folder of this skill's data)
  --start/--stop  bag seconds to use. START MUST BE AFTER ARMING (Cube IMU data begins at arm; the bag usually starts ~20 s earlier).

Why post-arm and signed:
  * forcing the whole bag inside the log yields spurious offsets (a +17 s 'match' that was really a half-turn-pattern shift);
  * |gyro| magnitude cannot tell a half-period shift from the truth (the repeated mission has periodic +/- yaw pulses).
A true match gives r >= 0.99 at an offset of about 0 s (Jetson clock and Cube GPS time agree to ~0.1 s); wrong logs give r < 0.5."""
import argparse, glob, os
import numpy as np
from scipy.signal import butter, sosfiltfilt, fftconvolve
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

F = 20.0
ap = argparse.ArgumentParser()
ap.add_argument("bag"); ap.add_argument("--logs", default=os.path.expanduser("~/Manuj/cube_logs_nov3-4_2026"))
ap.add_argument("--start", type=float, default=30); ap.add_argument("--stop", type=float, default=400); ap.add_argument("--topic", default="/imu0")
a = ap.parse_args()


def ser(t, g):
    tt = np.arange(t[0], t[-1], 1 / F)
    return tt, sosfiltfilt(butter(4, 5 / (F / 2), output="sos"), np.interp(tt, t, g))


def sliding_r(x, c):                       # Pearson r of x against every length-len(x) window of c
    n = len(x); x = x - x.mean(); sx = np.sqrt((x ** 2).sum())
    num = fftconvolve(c, x[::-1], mode="valid")
    cs = np.concatenate([[0], np.cumsum(c)]); cs2 = np.concatenate([[0], np.cumsum(c ** 2)])
    mu = (cs[n:] - cs[:-n]) / n; var = (cs2[n:] - cs2[:-n]) - n * mu ** 2
    return num / (sx * np.sqrt(np.maximum(var, 1e-12)))


ts = get_typestore(Stores.ROS2_HUMBLE); T = []; G = []
with Reader(a.bag) as r:
    cons = [c for c in r.connections if c.topic == a.topic]
    if not cons:
        raise SystemExit(f"no {a.topic} in {a.bag}")
    for c, s, raw in r.messages(connections=cons, start=r.start_time + int(a.start * 1e9), stop=r.start_time + int(a.stop * 1e9)):
        m = ts.deserialize_cdr(raw, c.msgtype); T.append(s / 1e9); G.append(m.angular_velocity.z)   # receive time (header stamps are IMX-clock, not epoch)
tb, sb = ser(np.array(T), np.array(G))
print(f"{os.path.basename(a.bag.rstrip('/'))}: {tb[-1]-tb[0]:.0f} s of gyro Z used (bag {a.start:.0f}-{a.stop:.0f} s)")
res = []
for f in sorted(glob.glob(os.path.join(a.logs, "log*_imu_att.npz"))):
    Z = np.load(f); cu = Z["imu0"]
    if len(cu) < 2000:
        continue
    tref, uref = Z["ref"]; tc, sc = ser(uref + (cu[:, 0] - tref), cu[:, 3])
    if len(sc) < len(sb):
        continue
    r_ = sliding_r(sb, sc); i = int(np.argmax(r_)); res.append((r_[i], os.path.basename(f)[:6], tc[0] + i / F - tb[0]))
for r_, k, off in sorted(res, reverse=True)[:4]:
    print(f"   {k}: best signed r = {r_:+.3f}   Cube time minus bag time = {off:+.2f} s")
if res and max(res)[0] >= 0.95:
    print(f"=> MATCH: {max(res)[1]}")
else:
    print("=> no confident match (need r >= 0.95): the flight may not be on this Cube / not downloaded / not extracted yet")
