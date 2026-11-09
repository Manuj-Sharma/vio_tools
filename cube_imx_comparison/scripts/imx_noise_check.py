#!/usr/bin/env python3
"""IMX /imu0 noise character in a bag segment: Gaussian or glitchy? how big? what OpenVINS noise density does it imply?

usage: imx_noise_check.py --bag BAG_DIR [--start 25] [--stop 280] [--topic /imu0] [--config-density 0.2]
Prints, per axis: robust sigma (MAD of x minus a 9-sample rolling median), excess kurtosis, % of samples beyond 6 sigma (and how many are isolated
single samples), the Welch PSD at 1/10/30 Hz BEFORE and AFTER replacing the >6 sigma samples with the rolling median (if the PSD does not move, the noise
is not glitches), and the noise density sqrt(PSD at 1 Hz) in m/s^2/sqrt(Hz) = the number to compare with OpenVINS accelerometer_noise_density.
Reference (2026-11-09, in flight): density 0.21-0.34 on X/Y, 0.50-0.65 on Z; bench static 5.5e-4. Excess kurtosis ~0 = Gaussian."""
import argparse
import numpy as np
from scipy.ndimage import median_filter
from scipy.signal import welch
from scipy.stats import kurtosis
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

ap = argparse.ArgumentParser()
ap.add_argument("--bag", required=True); ap.add_argument("--start", type=float, default=25); ap.add_argument("--stop", type=float, default=280)
ap.add_argument("--topic", default="/imu0"); ap.add_argument("--config-density", type=float, default=0.2)
a = ap.parse_args(); ts = get_typestore(Stores.ROS2_HUMBLE); T = []; A = []
with Reader(a.bag) as r:
    cons = [c for c in r.connections if c.topic == a.topic]
    for c, s, raw in r.messages(connections=cons, start=r.start_time + int(a.start * 1e9), stop=r.start_time + int(a.stop * 1e9)):
        m = ts.deserialize_cdr(raw, c.msgtype); T.append(s / 1e9); A.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
T = np.array(T); A = np.array(A); fs = len(T) / (T[-1] - T[0]); print(f"{a.bag} {a.topic}: {len(T)} samples, {fs:.0f} Hz, {T[-1]-T[0]:.0f} s")
med = median_filter(A, size=(9, 1), mode="nearest"); d = A - med; mad = 1.4826 * np.median(np.abs(d), axis=0); z = np.abs(d) / mad
print("robust sigma (m/s^2):", np.round(mad, 3), " plain std of residual:", np.round(d.std(0), 3), " excess kurtosis of raw:", np.round(kurtosis(A, axis=0), 2))
out = (z > 6).any(1); iso = out & ~np.roll(out, 1) & ~np.roll(out, -1)
print(f"samples beyond 6 sigma on any axis: {100*out.mean():.2f}% ({100*iso.sum()/max(out.sum(),1):.0f}% of them isolated single samples)")
Ac = A.copy(); Ac[z > 6] = med[z > 6]


def psd(X):
    tt = np.arange(T[0] + 2, T[-1] - 2, 1 / 320.0); out_ = []
    for i in range(3):
        y = np.interp(tt, T, X[:, i]); f, p = welch(y - y.mean(), fs=320.0, nperseg=2048); out_.append((f, p))
    return out_


p0 = psd(A); p1 = psd(Ac); g = lambda p, f, x: p[np.argmin(abs(f - x))]
for i, ax in enumerate("XYZ"):
    f = p0[i][0]
    print(f"  {ax}: PSD raw->cleaned  1 Hz {g(p0[i][1],f,1):.4f}->{g(p1[i][1],f,1):.4f}   10 Hz {g(p0[i][1],f,10):.4f}->{g(p1[i][1],f,10):.4f}   30 Hz {g(p0[i][1],f,30):.4f}->{g(p1[i][1],f,30):.4f}"
          f"   noise density {np.sqrt(g(p0[i][1], f, 1)):.3f} m/s^2/sqrtHz  (config {a.config_density}: x{np.sqrt(g(p0[i][1], f, 1))/a.config_density:.1f})")
