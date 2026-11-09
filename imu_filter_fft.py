#!/usr/bin/env python3
"""FFT / PSD analysis of a 1 kHz IMX capture (CSV from imx5_poll_rate --dump) before and after a causal
Butterworth low-pass, all six channels.

  imu_filter_fft.py CSV [--cutoff 100] [--order 4] [--out-rate 300] [--png OUT.png]

Steps: (1) put samples on the IMX 1 ms tick grid and fill missed ticks linearly (same as the driver's
imu_filter.hpp), (2) causal Butterworth (sosfilt, state seeded to steady state), (3) Welch PSD before/after,
(4) resample the filtered stream to --out-rate by linear interpolation and show its PSD (aliasing check),
(5) group delay of the filter.
"""
import argparse
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("--cutoff", type=float, default=100.0)
ap.add_argument("--order", type=int, default=4)
ap.add_argument("--out-rate", type=float, default=300.0)
ap.add_argument("--png", default=None)
a = ap.parse_args()
FS = 1000.0
NAMES = ["gx", "gy", "gz", "ax", "ay", "az"]
UNITS = ["rad/s"] * 3 + ["m/s^2"] * 3

d = np.genfromtxt(a.csv, delimiter=",", names=True)
t = d["imx_t"]
raw = np.stack([d[n] for n in NAMES], axis=1)
ticks = np.rint((t - t[0]) * FS).astype(np.int64)
keep = np.concatenate([[True], np.diff(ticks) > 0])          # drop any non-increasing tick
ticks, raw = ticks[keep], raw[keep]
grid = np.arange(ticks[0], ticks[-1] + 1)
x = np.stack([np.interp(grid, ticks, raw[:, c]) for c in range(6)], axis=1)
n_missing = len(grid) - len(ticks)
print(f"{a.csv}\n  captured {len(ticks)} of {len(grid)} ticks ({100*len(ticks)/len(grid):.2f}%), "
      f"{n_missing} missed ticks linearly filled, duration {len(grid)/FS:.1f} s")

sos = signal.butter(a.order, a.cutoff, btype="low", fs=FS, output="sos")
zi = signal.sosfilt_zi(sos)
y = np.stack([signal.sosfilt(sos, x[:, c], zi=zi * x[0, c])[0] for c in range(6)], axis=1)

# ---- group delay -------------------------------------------------------------------------------------------
w = np.array([0.5, 5, 10, 20, 30, 50, 70, 100, 120])
_, h = signal.sosfreqz(sos, worN=w, fs=FS)
phase = np.unwrap(np.angle(signal.sosfreqz(sos, worN=np.linspace(0.01, 200, 40000), fs=FS)[1]))
f_dense = np.linspace(0.01, 200, 40000)
gd = -np.gradient(phase, 2 * np.pi * f_dense)                 # seconds
print(f"\nButterworth order {a.order}, fc {a.cutoff:.0f} Hz (-3 dB), fs {FS:.0f} Hz, causal")
print("  freq(Hz)  gain(dB)  group delay (ms)")
for fq, hh in zip(w, h):
    print(f"  {fq:7.1f}  {20*np.log10(abs(hh)):8.2f}  {1e3*np.interp(fq, f_dense, gd):8.3f}")
print(f"  DC group delay = {1e3*gd[0]:.3f} ms")

# ---- PSD -----------------------------------------------------------------------------------------------------
nper = 4096
fr, Px = signal.welch(x, fs=FS, nperseg=nper, axis=0, detrend="constant")
_, Py = signal.welch(y, fs=FS, nperseg=nper, axis=0, detrend="constant")

print("\nRMS (mean removed)           before        after     removed")
for c in range(6):
    rx, ry = np.std(x[:, c]), np.std(y[:, c])
    print(f"  {NAMES[c]}  [{UNITS[c]:5}]  {rx:12.5g} {ry:12.5g}  {100*(1-ry/rx):6.1f} %")

print("\nPSD ratio after/before (dB) in bands, and top raw spectral peaks (>2 Hz):")
bands = [(2, 50), (50, 100), (100, 150), (150, 250), (250, 500)]
print("  chan  " + "  ".join(f"{lo}-{hi}Hz".rjust(9) for lo, hi in bands) + "    top peaks (Hz @ PSD)")
for c in range(6):
    cols = []
    for lo, hi in bands:
        m = (fr >= lo) & (fr < hi)
        cols.append(10 * np.log10(Py[m, c].sum() / Px[m, c].sum()))
    m = fr > 2
    idx = np.argsort(Px[m, c])[::-1]
    pk, seen = [], []
    for i in idx:
        f0 = fr[m][i]
        if all(abs(f0 - s) > 3 for s in seen):
            seen.append(f0); pk.append(f"{f0:.1f}")
        if len(pk) == 3: break
    print(f"  {NAMES[c]}  " + "  ".join(f"{v:9.1f}" for v in cols) + "    " + ", ".join(pk))

# ---- 300 Hz resample + aliasing check ------------------------------------------------------------------------
t_in = grid / FS
t_out = np.arange(t_in[0], t_in[-1], 1.0 / a.out_rate)
yo = np.stack([np.interp(t_out, t_in, y[:, c]) for c in range(6)], axis=1)
fo, Po = signal.welch(yo, fs=a.out_rate, nperseg=int(nper * a.out_rate / FS), axis=0, detrend="constant")
print(f"\n{a.out_rate:.0f} Hz output: {len(t_out)} samples. Total variance vs 1 kHz filtered (interp. + alias error):")
for c in range(6):
    print(f"  {NAMES[c]}  var(300Hz)/var(filtered 1kHz) = {np.var(yo[:, c])/np.var(y[:, c]):.4f}")

# ---- figure ----------------------------------------------------------------------------------------------------
fig, ax = plt.subplots(2, 3, figsize=(16, 8), sharex=True)
for c in range(6):
    A = ax.flat[c]
    A.semilogy(fr, Px[:, c], lw=0.8, label="raw 1 kHz")
    A.semilogy(fr, Py[:, c], lw=0.8, label=f"Butterworth {a.order}th, {a.cutoff:.0f} Hz")
    A.semilogy(fo, Po[:, c], lw=0.8, ls="--", label=f"filtered, resampled {a.out_rate:.0f} Hz")
    A.axvline(a.cutoff, color="k", lw=0.5, ls=":")
    A.set_title(f"{NAMES[c]}"); A.set_ylabel(f"PSD [({UNITS[c]})^2/Hz]"); A.grid(alpha=0.3, which="both")
    if c >= 3: A.set_xlabel("Hz")
ax.flat[0].legend(fontsize=8)
fig.suptitle(f"{a.csv.split('/')[-1]}  |  DC group delay {1e3*gd[0]:.2f} ms")
fig.tight_layout()
png = a.png or a.csv.rsplit(".", 1)[0] + f"_fft_fc{a.cutoff:.0f}.png"
fig.savefig(png, dpi=110)
print(f"\nfigure: {png}")
