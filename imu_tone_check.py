#!/usr/bin/env python3
"""Find narrow spectral lines in a raw 1 kHz /imu0 bag (e.g. the ~48 Hz tone with 192/384 Hz harmonics).
  imu_tone_check.py BAG [--t0 S --t1 S] [--lo 5 --hi 500] [--top 8] [--png OUT.png] [--ref F1 F2 ...]
Gap-fills onto the 1 ms grid (longest segment), Welch PSD (0.25 Hz bins by default) per channel, lists the strongest narrow peaks
(prominence over the local median floor) and, with --ref, the peak level at those frequencies. Needs ROS sourced; run with python3 (NOT -I)."""
import sys, glob, os, sqlite3, argparse
import numpy as np
from scipy import signal
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
ap = argparse.ArgumentParser(); ap.add_argument("bag"); ap.add_argument("--t0", type=float); ap.add_argument("--t1", type=float)
ap.add_argument("--lo", type=float, default=5); ap.add_argument("--hi", type=float, default=500); ap.add_argument("--top", type=int, default=8)
ap.add_argument("--nperseg", type=int, default=4096); ap.add_argument("--png"); ap.add_argument("--ref", type=float, nargs="*", default=[])
a = ap.parse_args(); FS = 1000.0; N = ["gx", "gy", "gz", "ax", "ay", "az"]
t, x = [], []
for db in sorted(glob.glob(os.path.join(a.bag, "*.db3"))):
    c = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
    r = c.execute("select id from topics where name='/imu0'").fetchone()
    if not r: continue
    for (b,) in c.execute("select data from messages where topic_id=? order by timestamp", (r[0],)):
        m = deserialize_message(bytes(b), Imu); t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        x.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z, m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
t = np.array(t); x = np.array(x); k = np.concatenate([[True], np.diff(t) > 0]); t, x = t[k] - t[k][0], x[k]
dk = np.rint(np.diff(t) * FS).astype(int); bd = np.concatenate([[0], np.where(dk > 20)[0] + 1, [len(t)]])
if a.t0 is not None or a.t1 is not None:
    m = (t >= (a.t0 or 0)) & (t <= (a.t1 or 1e9)); t, x = t[m], x[m]; bd = np.array([0, len(t)])
s0, s1 = max(zip(bd[:-1], bd[1:]), key=lambda p: p[1] - p[0]); t, x = t[s0:s1], x[s0:s1]
tk = np.concatenate([[0.0], np.cumsum(np.rint(np.diff(t) * FS))]); g = np.arange(tk[0], tk[-1] + 1)
X = np.stack([np.interp(g, tk, x[:, c]) for c in range(6)], 1); print(f"{len(t)} samples, {len(g)/FS:.1f} s analysed (t={t[0]:.1f}..{t[-1]:.1f}), {100*(1-len(t)/len(g)):.1f} % filled")
f, P = signal.welch(X - X.mean(0), fs=FS, nperseg=min(a.nperseg, len(g)), axis=0); df = f[1]
sel = (f >= a.lo) & (f <= a.hi); fig, ax = plt.subplots(2, 3, figsize=(16, 8), sharex=True)
for c in range(6):
    p = P[:, c]; floor = signal.medfilt(p, 41); ratio = p / floor
    pk, _ = signal.find_peaks(np.where(sel, ratio, 0), height=4, distance=max(1, int(1.0 / df)))
    pk = pk[np.argsort(ratio[pk])[::-1][:a.top]]
    print(f"{N[c]}: floor@100Hz {np.interp(100, f, floor):.2e}  lines (Hz, x over local floor): " + ", ".join(f"{f[i]:.2f} ({ratio[i]:.0f}x)" for i in sorted(pk)) if len(pk) else f"{N[c]}: no line >4x floor")
    for fr in a.ref:
        j = np.argmin(abs(f - fr)); w = slice(max(0, j - 3), j + 4); print(f"      ref {fr:g} Hz: peak {f[w][np.argmax(p[w])]:.2f} Hz, {ratio[w].max():.1f}x floor")
    A = ax.flat[c]; A.semilogy(f, p, lw=.7); A.semilogy(f, floor, "k", lw=.6, alpha=.6); A.set_title(N[c]); A.grid(alpha=.3, which="both")
    for fr in a.ref: A.axvline(fr, color="r", lw=.4, ls=":")
fig.tight_layout(); png = a.png or a.bag.rstrip("/") + "_tones.png"; fig.savefig(png, dpi=100); print("figure:", png)
