#!/usr/bin/env python3
"""Compare the IMX /imu0 stream in a rosbag with the Cube's logged IMU[0] for the SAME flight, and draw 6 figures.

usage: compare_cube_vs_imx.py --bag BAG_DIR --npz logNNN_imu_att.npz --tag TAG [--out DIR] [--start 25] [--stop 280] [--offset 0.0] [--lp 10]
  --start/--stop  bag seconds, START AFTER ARMING. --offset: Cube-minus-bag seconds from match_bag_to_log.py (normally ~0).
  --lp  low-pass (Hz) applied to BOTH signals (zero-phase 4th-order Butterworth, on a 200 Hz grid). The Cube's logged accel is
        already low-passed at INS_ACCEL_FILTER (10 Hz on this vehicle) and gyro at INS_GYRO_FILTER (25 Hz): above 10 Hz the two
        are NOT like-for-like.
What it does: interpolates both to a 200 Hz epoch grid, finds the small clock lag from |gyro| (IMX later = positive), rotates the Cube
into the IMX frame (Kabsch fit on the low-passed accel vectors), then prints slope/correlation per axis and writes
  fig1_timeseries_..._TAG.png (40 s window)   fig2_scatter_..._TAG.png (IMX vs Cube per axis, slope + r)
  fig3_spectrum_TAG.png (Welch PSD, raw IMX vs logged Cube)   fig4_error_vs_vibration_TAG.png
  fig5_whole_flight_10Hz_TAG.png   fig6_whole_flight_1Hz_TAG.png
Do NOT compare |accel| magnitudes under vibration (a norm rectifies the noise: it gave near-zero correlation) - compare per-axis vectors.
Colors: dataviz reference palette slots 1-2 (blue #2a78d6 = Cube, orange #eb6834 = IMX), light surface, legends on every multi-series chart."""
import argparse, os
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfiltfilt, correlate, welch
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

ap = argparse.ArgumentParser()
ap.add_argument("--bag", required=True); ap.add_argument("--npz", required=True); ap.add_argument("--tag", required=True)
ap.add_argument("--out", default="./figures"); ap.add_argument("--start", type=float, default=25); ap.add_argument("--stop", type=float, default=280)
ap.add_argument("--offset", type=float, default=0.0); ap.add_argument("--lp", type=float, default=10.0)
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True); OUT = a.out.rstrip("/") + "/"; TAG = a.tag
SURF = "#fcfcfb"; INK = "#0b0b0b"; INK2 = "#52514e"; GRID = "#e6e5e1"; C_CUBE = "#2a78d6"; C_IMX = "#eb6834"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF, "text.color": INK, "axes.labelcolor": INK2,
    "xtick.color": INK2, "ytick.color": INK2, "axes.edgecolor": GRID, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 10.5, "axes.titlecolor": INK, "legend.frameon": False})
AX = "XYZ"
Z = np.load(a.npz); tref, uref = Z["ref"]; cu = Z["imu0"]; cu_t = uref + (cu[:, 0] - tref); cu_g = cu[:, 1:4]; cu_a = cu[:, 4:7]
ts = get_typestore(Stores.ROS2_HUMBLE); T = []; G = []; A = []
with Reader(a.bag) as r:
    bag0 = r.start_time / 1e9 + a.offset; cons = [c for c in r.connections if c.topic == "/imu0"]
    for c, s, raw in r.messages(connections=cons, start=r.start_time + int(a.start * 1e9), stop=r.start_time + int(a.stop * 1e9)):
        m = ts.deserialize_cdr(raw, c.msgtype); T.append(s / 1e9 + a.offset)          # RECEIVE time on the Jetson clock (header stamps are IMX-clock)
        G.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]); A.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
T = np.array(T); G = np.array(G); A = np.array(A)
fs = 200.0; w0 = max(T[0], cu_t[0]) + 3; w1 = min(T[-1], cu_t[-1]) - 3; tg = np.arange(w0, w1, 1 / fs); tb = tg - bag0
lp = lambda x, fc: sosfiltfilt(butter(4, fc / (fs / 2), output="sos"), x, axis=0)
grid = lambda t, x, lag=0.0: np.column_stack([np.interp(tg, t - lag, x[:, i]) for i in range(3)])
band = lambda x, lo, hi: sosfiltfilt(butter(4, [lo / (fs / 2), hi / (fs / 2)], btype="band", output="sos"), x - x.mean(0), axis=0)
gc = np.linalg.norm(lp(grid(cu_t, cu_g), 5), axis=1)


def lagfor(Gx):
    gi = np.linalg.norm(lp(grid(T, Gx, 0), 5), axis=1); x = gi - gi.mean(); y = gc - gc.mean(); k = int(1.5 * fs)
    f = correlate(x, y, "full"); l = np.arange(-len(y) + 1, len(x)); sel = (l >= -k) & (l <= k); return l[sel][np.argmax(f[sel])] / fs


lag = lagfor(G)                                  # positive = IMX events are LATER than the Cube's
cuA = lp(grid(cu_t, cu_a), a.lp); imA = lp(grid(T, A, lag), a.lp); cuG = lp(grid(cu_t, cu_g), a.lp); imG = lp(grid(T, G, lag), a.lp)


def kabsch(P, Q):
    H = Q.T @ P; U, S, Vt = np.linalg.svd(H); return Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T


Ra = kabsch(imA, cuA); predA = cuA @ Ra.T; predG = cuG @ Ra.T
print(f"{a.bag}  vs  {a.npz}\nwindow {w1-w0:.0f} s, IMX rate {len(T)/(T[-1]-T[0]):.0f} Hz, clock lag IMX-later {lag*1e3:.0f} ms, low-pass {a.lp:g} Hz")
for nm, P, I in (("GYRO", predG, imG), ("ACCEL", predA, imA)):
    for i in range(3):
        x = P[:, i] - P[:, i].mean(); y = I[:, i] - I[:, i].mean()
        print(f"  {nm} {AX[i]}: slope {(x@y)/(x@x):.3f}  r {np.corrcoef(x, y)[0,1]:.3f}  rms diff {np.sqrt(np.mean((y-x)**2)):.4f} vs signal {np.sqrt(np.mean(x**2)):.4f}")

# ---- fig1: 40 s time series around the most active stretch
act = np.linalg.norm(imG, axis=1); i0 = np.argmax(np.convolve(act * (tb > 60), np.ones(400) / 400, "same")); c0 = tb[i0]; sel = (tb >= c0 - 20) & (tb <= c0 + 20)
fig, axs = plt.subplots(3, 2, figsize=(12, 7.4), sharex=True)
for j, (name, unit, P, I) in enumerate((("Rotation rate", "rad/s", predG, imG), ("Acceleration", "m/s²", predA, imA))):
    for i in range(3):
        ax = axs[i, j]; ax.plot(tb[sel], I[sel, i], color=C_IMX, lw=1.3, label="IMX /imu0"); ax.plot(tb[sel], P[sel, i], color=C_CUBE, lw=2.0, label="Cube IMU[0] (logged)")
        ax.set_ylabel(f"{AX[i]} ({unit})")
        if i == 0: ax.set_title(f"{name}, {a.lp:g} Hz low-pass, IMX frame", loc="left")
    axs[2, j].set_xlabel("time since bag start (s)")
h, l = axs[0, 1].get_legend_handles_labels(); fig.legend(h[::-1], l[::-1], loc="upper center", ncol=2, bbox_to_anchor=(0.5, 0.995))
fig.suptitle(f"Cube vs IMX over one 40 s stretch ({TAG})", y=0.955, x=0.01, ha="left", fontsize=12)
fig.text(0.01, 0.005, f"Cube rotated into the IMX frame (fitted); IMX clock shifted by {lag*1e3:.0f} ms. Both low-passed at {a.lp:g} Hz.", color=INK2, fontsize=8.5)
fig.tight_layout(rect=(0, 0.02, 1, 0.93)); fig.savefig(f"{OUT}fig1_timeseries_cube_vs_imx_{TAG}.png", dpi=150); plt.close(fig)
# ---- fig2: scatter
fig, axs = plt.subplots(2, 3, figsize=(12, 7.6))
for r_, (name, unit, P, I) in enumerate((("Rotation rate", "rad/s", predG, imG), ("Acceleration", "m/s²", predA, imA))):
    for i in range(3):
        ax = axs[r_, i]; x = P[::8, i]; y = I[::8, i]; ax.scatter(x, y, s=7, color=C_CUBE, alpha=0.22, linewidths=0)
        lim = np.array([min(x.min(), y.min()), max(x.max(), y.max())]); ax.plot(lim, lim, color=INK2, lw=1.2)
        xx = P[:, i] - P[:, i].mean(); yy = I[:, i] - I[:, i].mean()
        ax.text(0.04, 0.95, f"slope {(xx@yy)/(xx@xx):.2f}   r = {np.corrcoef(xx, yy)[0,1]:.2f}", transform=ax.transAxes, va="top", fontweight="bold")
        ax.set_xlabel(f"Cube IMU[0], {AX[i]} ({unit})"); ax.set_ylabel(f"IMX /imu0, {AX[i]} ({unit})"); ax.set_title(f"{name} {AX[i]}", loc="left"); ax.set_aspect("equal", "datalim")
fig.suptitle(f"IMX vs Cube, whole flight ({TAG}, {a.lp:g} Hz low-pass). On the line = agreement.", x=0.01, ha="left", fontsize=12, y=0.985)
fig.tight_layout(rect=(0, 0, 1, 0.96)); fig.savefig(f"{OUT}fig2_scatter_imx_vs_cube_{TAG}.png", dpi=150); plt.close(fig)
# ---- fig3: spectra (no filtering by us)
uni = lambda t, x, f: np.column_stack([np.interp(np.arange(t[0] + 2, t[-1] - 2, 1 / f), t, x[:, i]) for i in range(3)])
fi = 320.0; fc = 400.0; mi = (T > w0) & (T < w1); mc = (cu_t > w0) & (cu_t < w1); Ai = uni(T[mi], A[mi], fi); Ac = uni(cu_t[mc], cu_a[mc], fc)
fig, axs = plt.subplots(1, 3, figsize=(12, 4.6), sharey=True)
for i in range(3):
    f1, p1 = welch(Ac[:, i] - Ac[:, i].mean(), fs=fc, nperseg=2048); f2, p2 = welch(Ai[:, i] - Ai[:, i].mean(), fs=fi, nperseg=2048)
    g = lambda f, p, x: p[np.argmin(abs(f - x))]
    print(f"  PSD {AX[i]} (m/s^2)^2/Hz: 1 Hz Cube {g(f1,p1,1):.5f} IMX {g(f2,p2,1):.5f} | 10 Hz Cube {g(f1,p1,10):.6f} IMX {g(f2,p2,10):.5f}   IMX noise density at 1 Hz {np.sqrt(g(f2,p2,1)):.3f} m/s^2/sqrtHz")
    ax = axs[i]; ax.loglog(f1[1:], p1[1:], color=C_CUBE, lw=1.6, label="Cube IMU[0] (logged, filtered)"); ax.loglog(f2[1:], p2[1:], color=C_IMX, lw=1.6, label="IMX /imu0 (raw)")
    ax.axvline(10, color=INK2, lw=1.0); ax.text(10.8, 1e-7, "10 Hz", color=INK2, fontsize=9); ax.set_xlabel("frequency (Hz)"); ax.set_title(f"Acceleration {AX[i]}", loc="left"); ax.set_xlim(0.3, 160)
axs[0].set_ylabel("power spectral density ((m/s²)²/Hz)")
h, l = axs[2].get_legend_handles_labels(); fig.legend(h, l, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 0.99))
fig.suptitle(f"Acceleration spectrum ({TAG})", x=0.01, ha="left", fontsize=12, y=0.935); fig.tight_layout(rect=(0, 0, 1, 0.9)); fig.savefig(f"{OUT}fig3_spectrum_{TAG}.png", dpi=150); plt.close(fig)
# ---- fig4: low-frequency error vs vibration strength
vib = band(grid(T, A, lag), 10, 60); res = imA - predA; W = int(fs); n = len(tg) // W
R = np.array([res[i*W:(i+1)*W].mean(0) for i in range(n)]); V = np.linalg.norm(np.array([np.sqrt((vib[i*W:(i+1)*W]**2).mean(0)) for i in range(n)]), axis=1)
fig, axs = plt.subplots(1, 3, figsize=(12, 4.2), sharey=True)
for i in range(3):
    ax = axs[i]; ax.scatter(V, R[:, i], s=14, color=C_CUBE, alpha=0.5, linewidths=0); ax.axhline(0, color=INK2, lw=1.0)
    ax.text(0.04, 0.95, f"r = {np.corrcoef(V, R[:, i])[0,1]:+.2f}", transform=ax.transAxes, va="top", fontweight="bold"); ax.set_xlabel("IMX vibration, 10-60 Hz rms (m/s²), 1 s windows"); ax.set_title(f"Acceleration {AX[i]}", loc="left")
axs[0].set_ylabel("IMX minus Cube, 1 s mean (m/s²)")
fig.suptitle(f"Does the IMX low-frequency error follow vibration strength? ({TAG})", x=0.01, ha="left", fontsize=12, y=0.99); fig.tight_layout(rect=(0, 0, 1, 0.94)); fig.savefig(f"{OUT}fig4_error_vs_vibration_{TAG}.png", dpi=150); plt.close(fig)


# ---- fig5/6: whole flight at two low-pass settings
def whole(fcut, fname):
    cA = lp(grid(cu_t, cu_a), fcut) @ Ra.T; iA = lp(grid(T, A, lag), fcut); cG = lp(grid(cu_t, cu_g), fcut) @ Ra.T; iG = lp(grid(T, G, lag), fcut)
    fig, axs = plt.subplots(3, 2, figsize=(14, 8), sharex=True)
    for j, (name, unit, P, I) in enumerate((("Rotation rate", "rad/s", cG, iG), ("Acceleration", "m/s²", cA, iA))):
        for i in range(3):
            ax = axs[i, j]; ax.plot(tb, I[:, i], color=C_IMX, lw=1.0, label="IMX /imu0"); ax.plot(tb, P[:, i], color=C_CUBE, lw=1.6, label="Cube IMU[0] (logged)")
            x = P[:, i] - P[:, i].mean(); y = I[:, i] - I[:, i].mean()
            print(f"  WHOLE {fcut:g} Hz {name} {AX[i]}: r {np.corrcoef(x, y)[0,1]:.3f} rms diff {np.sqrt(np.mean((y-x)**2)):.4f} signal {np.sqrt(np.mean(x**2)):.4f}")
            ax.text(0.995, 0.97, f"r = {np.corrcoef(x, y)[0,1]:.2f}   rms diff {np.sqrt(np.mean((y-x)**2)):.3f} vs signal {np.sqrt(np.mean(x**2)):.3f}", transform=ax.transAxes, ha="right", va="top", fontsize=9, fontweight="bold")
            ax.set_ylabel(f"{AX[i]} ({unit})")
            if i == 0: ax.set_title(f"{name}, {fcut:g} Hz low-pass", loc="left")
        axs[2, j].set_xlabel("time since bag start (s)")
    h, l = axs[0, 1].get_legend_handles_labels(); fig.legend(h[::-1], l[::-1], loc="upper center", ncol=2, bbox_to_anchor=(0.5, 0.995))
    fig.suptitle(f"Cube vs IMX over the whole flight ({TAG})", y=0.955, x=0.01, ha="left", fontsize=12)
    fig.text(0.01, 0.005, f"Same alignment as fig1, both low-passed at {fcut:g} Hz. r = correlation, rms diff = rms of IMX minus Cube.", color=INK2, fontsize=8.5)
    fig.tight_layout(rect=(0, 0.02, 1, 0.93)); fig.savefig(OUT + fname, dpi=150); plt.close(fig)


whole(10, f"fig5_whole_flight_10Hz_{TAG}.png"); whole(1, f"fig6_whole_flight_1Hz_{TAG}.png")
print(f"figures written to {OUT} (suffix _{TAG})")
