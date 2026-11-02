#!/usr/bin/env python3
"""imu_dip_period.py BAG [-o out.png] [--bin 0.5]

Plots the /imu0 rate (and cam0/cam1) across a recording in BIN-second bins, from both the bag's
receive timestamps and the IMU header stamps (sensor grid), and looks for periodic dips: prints
dip times and the dominant period (autocorrelation + FFT of the rate series).
Reads .db3 read-only (immutable=1).
"""
import argparse, glob, os, sqlite3, struct
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(bag, topic, hdr=False):
    rx, hs = [], []
    for f in sorted(glob.glob(os.path.join(bag, "*.db3"))):
        c = sqlite3.connect(f"file:{f}?immutable=1", uri=True)
        t = c.execute("select id from topics where name=?", (topic,)).fetchone()
        if not t:
            continue
        if hdr:
            for ts, d in c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (t[0],)):
                s, n = struct.unpack_from("<iI", d, 4)
                rx.append(ts * 1e-9); hs.append(s + n * 1e-9)
        else:
            rx += [r[0] * 1e-9 for r in c.execute("select timestamp from messages where topic_id=? order by timestamp", (t[0],))]
    return np.array(rx), np.array(hs)


def rate(t, t0, n, b):
    return np.histogram(t, bins=np.arange(n + 1) * b + t0)[0] / b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("-o", "--out", default="/tmp/imu_dip_period.png")
    ap.add_argument("--bin", type=float, default=0.5)
    a = ap.parse_args()
    irx, ihs = load(a.bag, "/imu0", True)
    c0, _ = load(a.bag, "/cam0/image_raw"); c1, _ = load(a.bag, "/cam1/image_raw")
    t0 = irx[0]; n = int((irx[-1] - t0) / a.bin)
    ri, rh = rate(irx, t0, n, a.bin), rate(ihs - ihs[0] + t0, t0, n, a.bin)
    r0, r1 = rate(c0, t0, n, a.bin), rate(c1, t0, n, a.bin)
    tt = (np.arange(n) + .5) * a.bin
    med = np.median(ri[4:-4])
    dips = [i for i in range(2, n - 2) if ri[i] < 0.85 * med and ri[i] <= ri[i-1] and ri[i] <= ri[i+1]]
    print(f"{len(irx)} imu msgs, median {med:.0f} Hz per {a.bin}s bin; dips <85% of median at t(s):")
    print("  " + ", ".join(f"{tt[i]:.1f}({ri[i]:.0f})" for i in dips))
    if len(dips) > 1: print("  spacing:", np.round(np.diff([tt[i] for i in dips]), 1))
    x = ri[4:-4] - ri[4:-4].mean(); ac = np.correlate(x, x, "full")[len(x) - 1:]; ac /= ac[0]
    lag = np.argmax(ac[int(5 / a.bin):int(60 / a.bin)]) + int(5 / a.bin)
    print(f"  autocorr peak at {lag * a.bin:.1f} s (r={ac[lag]:.2f})")
    f = np.fft.rfftfreq(len(x), a.bin); P = abs(np.fft.rfft(x)); k = np.argmax(P[2:]) + 2
    print(f"  FFT peak period {1 / f[k]:.1f} s")
    fig, ax = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    ax[0].plot(tt, ri, lw=.8, label="rx (bag receive time)"); ax[0].plot(tt, rh, lw=.8, alpha=.7, label="header stamp")
    ax[0].set_ylabel("/imu0 Hz"); ax[0].legend()
    ax[1].plot(tt, r0, lw=.8, label="cam0"); ax[1].plot(tt, r1, lw=.8, label="cam1"); ax[1].set_ylabel("camera Hz"); ax[1].legend()
    ax[2].plot(tt[4:-4], x, lw=.8); ax[2].set_ylabel("imu rate - mean"); ax[2].set_xlabel("s from first imu msg")
    for i in dips:
        for q in ax: q.axvline(tt[i], color="r", alpha=.2)
    fig.tight_layout(); fig.savefig(a.out, dpi=110); print("wrote", a.out)

main()
