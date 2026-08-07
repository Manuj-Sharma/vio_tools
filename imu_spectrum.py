#!/usr/bin/env python3
"""imu_spectrum.py BAG [-o out.png] -- frequency content of the IMU.

    python3 imu_spectrum.py ~/rosbags/field_data/seg01_ros2_baro
    python3 imu_spectrum.py BAG --split          # per flight phase
    python3 imu_spectrum.py BAG --topic /imu0

WHAT IT IS FOR

A multirotor's IMU sees motor and prop harmonics on top of the motion you want.
Two things go wrong and neither is visible in the time domain:

  ALIASING. Content above Nyquist folds back to low frequency and lands on top
  of the vehicle dynamics, where no filter can separate it. At ~238 Hz sampling
  Nyquist is ~119 Hz; a 100 Hz motor tone is fine, a 300 Hz tone appears at
  62 Hz and is indistinguishable from real motion. Energy piled up against the
  Nyquist edge is the warning sign -- it means the sensor's internal filter is
  not cutting hard enough before sampling.

  BIAS/ATTITUDE CORRUPTION. Rectified vibration shows up as an apparent bias.
  This dataset shows bg_y drifting -0.063 -> -0.149 deg/s during cruise and
  recovering on descent, which is the shape you would expect if vibration
  amplitude tracks throttle.

WHAT IT REPORTS

  * measured sample rate and dt jitter (the FFT assumes uniform sampling; if
    jitter is large the spectrum is smeared and peak frequencies are only
    approximate)
  * Welch PSD per axis, gyro and accel
  * the dominant peaks per axis with their frequencies
  * band powers: 0-5 Hz is vehicle motion, everything above is vibration
  * how much energy sits in the top 10% below Nyquist -- the aliasing flag

Welch is implemented here rather than imported: scipy on this box warns about a
numpy version mismatch, and this is fifteen lines.
"""
import argparse
import os
import sys

import numpy as np


def read_imu(bag, topic):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    t, g, a = [], [], []
    with Reader(bag) as r:
        cons = [c for c in r.connections if c.topic == topic]
        if not cons:
            sys.exit(f"no {topic} in {bag}; topics: {[c.topic for c in r.connections]}")
        for _, stamp, raw in r.messages(connections=cons):
            m = ts.deserialize_cdr(raw, cons[0].msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            g.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
            a.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    return np.array(t), np.array(g), np.array(a)


def welch(x, fs, nperseg=4096, overlap=0.5):
    """Welch PSD. Hann window, 50% overlap, one-sided, correctly normalised."""
    x = np.asarray(x, float) - np.mean(x)
    step = int(nperseg * (1 - overlap))
    segs = [x[i:i + nperseg] for i in range(0, len(x) - nperseg + 1, step)]
    if not segs:
        nperseg = len(x) // 2 * 2
        segs = [x[:nperseg]]
    w = np.hanning(nperseg)
    scale = 1.0 / (fs * (w ** 2).sum())
    acc = np.zeros(nperseg // 2 + 1)
    for s in segs:
        acc += np.abs(np.fft.rfft(s * w)) ** 2
    psd = acc / len(segs) * scale
    psd[1:-1] *= 2                       # one-sided
    return np.fft.rfftfreq(nperseg, 1 / fs), psd


def peaks(f, p, n=4, fmin=1.0):
    """n strongest local maxima above fmin Hz."""
    m = f >= fmin
    ff, pp = f[m], p[m]
    loc = [i for i in range(1, len(pp) - 1) if pp[i] > pp[i - 1] and pp[i] > pp[i + 1]]
    loc.sort(key=lambda i: -pp[i])
    return [(ff[i], pp[i]) for i in loc[:n]]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/imu0")
    ap.add_argument("-o", "--out", default="/tmp/imu_spectrum.png")
    ap.add_argument("--split", action="store_true",
                    help="also analyse first/middle/last third separately")
    args = ap.parse_args()

    t, g, a = read_imu(args.bag, args.topic)
    dt = np.diff(t)
    fs = 1.0 / np.median(dt)
    nyq = fs / 2
    print(f"  {len(t)} samples over {t[-1]-t[0]:.1f} s")
    print(f"  fs {fs:.2f} Hz (median dt {np.median(dt)*1000:.3f} ms)   Nyquist {nyq:.1f} Hz")
    print(f"  dt jitter: std {np.std(dt)*1000:.3f} ms, p99 {np.percentile(dt,99)*1000:.3f} ms, "
          f"max {dt.max()*1000:.1f} ms")
    if np.std(dt) / np.median(dt) > 0.2:
        print("  WARNING: dt jitter > 20% of the interval -- peak frequencies are approximate")
    print()

    names = ["gyro x", "gyro y", "gyro z", "accel x", "accel y", "accel z"]
    chans = [g[:, 0], g[:, 1], g[:, 2], a[:, 0], a[:, 1], a[:, 2]]
    units = ["(rad/s)^2/Hz"] * 3 + ["(m/s^2)^2/Hz"] * 3

    spectra = []
    print(f"  {'channel':9s} {'dominant peaks [Hz]':>34s}   {'0-5Hz':>9s} {'5-50':>9s} "
          f"{'50-Nyq':>9s} {'top10%':>7s}")
    for nm, x, u in zip(names, chans, units):
        f, p = welch(x, fs)
        spectra.append((nm, f, p, u))
        pk = peaks(f, p)
        band = lambda lo, hi: np.trapz(p[(f >= lo) & (f < hi)], f[(f >= lo) & (f < hi)])
        tot = np.trapz(p, f)
        b1, b2, b3 = band(0, 5), band(5, 50), band(50, nyq)
        edge = band(0.9 * nyq, nyq) / tot * 100 if tot > 0 else 0
        pks = "  ".join(f"{fq:6.1f}" for fq, _ in pk)
        print(f"  {nm:9s} {pks:>34s}   {100*b1/tot:8.1f}% {100*b2/tot:8.1f}% "
              f"{100*b3/tot:8.1f}% {edge:6.1f}%")

    print()
    print("  0-5 Hz is vehicle motion; above that is vibration.")
    print("  top10% = share of power in the highest 10% below Nyquist. More than a few")
    print("  percent suggests content is being aliased in from above Nyquist, which no")
    print("  amount of filtering downstream can undo.")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 3, figsize=(16, 7), sharex=True)
    for i, (nm, f, p, u) in enumerate(spectra):
        b = ax[i // 3][i % 3]
        b.semilogy(f, np.maximum(p, 1e-20), lw=.7)
        for fq, pv in peaks(f, p, 3):
            b.plot(fq, pv, "rv", ms=5)
            b.annotate(f"{fq:.0f}", (fq, pv), fontsize=7, xytext=(2, 4),
                       textcoords="offset points")
        b.axvline(nyq, color="k", ls="--", lw=.8)
        b.set_title(f"{nm}   [{u}]", fontsize=9)
        b.grid(alpha=.3, which="both")
        if i // 3 == 1: b.set_xlabel("Hz")
    fig.suptitle(f"{os.path.basename(args.bag)}  fs={fs:.1f} Hz  (dashed = Nyquist)", fontsize=10)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"\n  wrote {args.out}")

    if args.split:
        print("\n  === by flight phase (thirds) ===")
        n = len(t) // 3
        for lbl, sl in (("first ", slice(0, n)), ("middle", slice(n, 2 * n)),
                        ("last  ", slice(2 * n, None))):
            row = f"  {lbl}"
            for nm, x in zip(names, chans):
                f, p = welch(x[sl], fs, nperseg=min(4096, (len(x[sl]) // 2) * 2))
                tot = np.trapz(p, f)
                hi = np.trapz(p[f >= 50], f[f >= 50])
                row += f"  {nm.split()[0][0]}{nm[-1]}:{100*hi/tot:5.1f}%"
            print(row + "   (share of power above 50 Hz)")


if __name__ == "__main__":
    main()
