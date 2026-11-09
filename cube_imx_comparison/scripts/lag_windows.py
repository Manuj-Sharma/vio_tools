#!/usr/bin/env python3
"""IMX-vs-Cube time lag in sliding windows (is the shift constant? does it drift?).

usage: lag_windows.py --bag BAG_DIR --npz logNNN_imu_att.npz [--start 30] [--stop 300] [--win 30] [--step 15]
Uses |gyro| (8 Hz low-pass, 200 Hz grid) of IMX /imu0 RECEIVE time vs the Cube IMU[0] on the unix-epoch grid, no shift applied.
Sign convention of the printed numbers: POSITIVE = the IMX data is LATER than the Cube's for the same event
(consistent with the whole-flight lag printed by compare_cube_vs_imx.py).
Reference results (2026-11-09): 165506 mean +26 ms (std 10), 103617 +37 ms (std 12), 110545 +116 ms (std 39, drifting)."""
import argparse
import numpy as np
from scipy.signal import butter, sosfiltfilt
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

ap = argparse.ArgumentParser()
ap.add_argument("--bag", required=True); ap.add_argument("--npz", required=True)
ap.add_argument("--start", type=float, default=30); ap.add_argument("--stop", type=float, default=300)
ap.add_argument("--win", type=float, default=30); ap.add_argument("--step", type=float, default=15); ap.add_argument("--maxlag", type=float, default=0.5)
a = ap.parse_args()
fs = 200.0; sos = butter(4, 8 / (fs / 2), output="sos"); ts = get_typestore(Stores.ROS2_HUMBLE)


def mag(t, g, tg):
    return sosfiltfilt(sos, np.linalg.norm(np.column_stack([np.interp(tg, t, g[:, i]) for i in range(3)]), axis=1))


Z = np.load(a.npz); tref, uref = Z["ref"]; cu = Z["imu0"]; ct = uref + (cu[:, 0] - tref); cg = cu[:, 1:4]
T = []; G = []
with Reader(a.bag) as r:
    cons = [c for c in r.connections if c.topic == "/imu0"]
    for c, s, raw in r.messages(connections=cons, start=r.start_time + int(a.start * 1e9), stop=r.start_time + int(a.stop * 1e9)):
        m = ts.deserialize_cdr(raw, c.msgtype); T.append(s / 1e9); G.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
T = np.array(T); G = np.array(G); b0 = T[0]
tg = np.arange(max(T[0], ct[0]) + 1, min(T[-1], ct[-1]) - 1, 1 / fs); x_all = mag(T, G, tg); y_all = mag(ct, cg, tg)
W = int(a.win * fs); res = []
for i in range(0, len(tg) - W, int(a.step * fs)):
    x = x_all[i:i + W] - x_all[i:i + W].mean(); y = y_all[i:i + W] - y_all[i:i + W].mean()
    if x.std() < 0.02:
        continue                                   # not enough motion in this window
    K = int(a.maxlag * fs); lags = np.arange(-K, K + 1)
    cc = [np.dot(x[max(0, -l):W - max(0, l)], y[max(0, l):W - max(0, -l)]) / np.sqrt(np.dot(x, x) * np.dot(y, y)) for l in lags]
    j = int(np.argmax(cc))
    lag = (lags[j] + ((cc[j - 1] - cc[j + 1]) / (2 * (cc[j - 1] - 2 * cc[j] + cc[j + 1])) if 0 < j < len(cc) - 1 else 0.0)) / fs
    res.append((tg[i] - b0, -lag * 1e3, cc[j]))   # sign flipped so that POSITIVE = IMX later (see docstring)
L = np.array([q[1] for q in res])
print(f"{a.bag} vs {a.npz}: {len(res)} windows with motion; IMX-later-than-Cube lag in ms:")
print("  " + " ".join(f"{v:.0f}" for v in L))
print(f"  mean {L.mean():.0f} ms, std {L.std():.0f}, min {L.min():.0f}, max {L.max():.0f}; trend {np.polyfit([q[0] for q in res], L, 1)[0]*60:+.1f} ms/min")
