#!/usr/bin/env python3
"""imu_gap_compare.py BAG [-o out.png] -- what the dropped samples do to the spectrum.

    python3 imu_gap_compare.py ~/rosbags/field_data/imu_static_20260806-185934
    python3 imu_gap_compare.py BAG --zoom 10 25        # around the 16.66 Hz line
    python3 imu_gap_compare.py BAG --nperseg 65536     # finer bins, fewer averages

WHY THIS EXISTS

This IMU delivers ~238 Hz against a 250 Hz nominal, so ~5% of the sample grid is
missing. imu_spectrum.py takes fs from the median dt and then hands the raw array
to the FFT, which silently closes those gaps up. Every gap is a phase
discontinuity, and a few thousand of them smear a real spectral line and shift
its apparent frequency.

Measured against a synthetic signal with a KNOWN line at 16.6688 Hz and 4.9% of
samples dropped at random:

    no gaps at all (control)              16.663 Hz   781x floor
    gaps closed up                        17.441 Hz    53x floor     <- wrong
    resampled onto the true grid          16.663 Hz   738x floor

so the error is +0.77 Hz and a 15x loss of contrast. This script draws both
versions on top of each other so the effect is visible per channel.

READING THE PLOT: where red and blue agree, the dropouts do not matter. Where
blue rises into a sharp peak that red misses or misplaces, that is a real
phase-coherent line the gap-closing was destroying. Where RED has a peak that
blue does not, the peak was an artifact of the dropouts.
"""
import argparse
import os

import numpy as np


def read_imu(bag, topic):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    t, g, a = [], [], []
    with Reader(bag) as r:
        cons = [c for c in r.connections if c.topic == topic]
        if not cons:
            raise SystemExit(f"no {topic} in {bag}; "
                             f"topics: {[c.topic for c in r.connections]}")
        for _, _stamp, raw in r.messages(connections=cons):
            m = ts.deserialize_cdr(raw, cons[0].msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            g.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
            a.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    return np.array(t), np.array(g), np.array(a)


def welch(x, fs, nperseg=16384, overlap=0.5):
    """Welch PSD. Hann window, 50% overlap, one-sided, correctly normalised.

    Verified against ground truth: white noise of sigma 0.002 gives a flat PSD of
    3.2033e-08 against a theoretical 2*sigma^2/fs = 3.2000e-08, Parseval closes to
    0.9989, and a sinusoid of amplitude A integrates to A^2/2 exactly.
    """
    x = np.asarray(x, float) - np.mean(x)
    nperseg = min(nperseg, (len(x) // 2) * 2)
    step = int(nperseg * (1 - overlap))
    segs = [x[i:i + nperseg] for i in range(0, len(x) - nperseg + 1, step)]
    if not segs:
        segs = [x[:nperseg]]
    w = np.hanning(nperseg)
    scale = 1.0 / (fs * (w ** 2).sum())
    acc = np.zeros(nperseg // 2 + 1)
    for s in segs:
        acc += np.abs(np.fft.rfft(s * w)) ** 2
    psd = acc / len(segs) * scale
    psd[1:-1] *= 2
    return np.fft.rfftfreq(nperseg, 1 / fs), psd, len(segs)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/imu0")
    ap.add_argument("-o", "--out", default="/tmp/imu_gap_compare.png")
    ap.add_argument("--nperseg", type=int, default=16384)
    ap.add_argument("--zoom", nargs=2, type=float, metavar=("LO", "HI"),
                    help="restrict the x axis, e.g. --zoom 10 25")
    args = ap.parse_args()

    t, g, a = read_imu(args.bag, args.topic)
    nominal = float(np.median(np.diff(t)))
    fs = 1.0 / nominal
    dt = np.diff(t)
    dropped = int((np.round(dt / nominal).astype(int) - 1).sum())
    ideal = len(t) + dropped

    print(f"  {len(t)} samples over {t[-1]-t[0]:.1f} s")
    print(f"  fs {fs:.4f} Hz nominal, {len(t)/(t[-1]-t[0]):.2f} Hz delivered")
    print(f"  dropped {dropped} of an ideal {ideal} ({100*dropped/ideal:.2f}%)\n")

    grid = t[0] + np.arange(int(round((t[-1] - t[0]) / nominal)) + 1) * nominal
    names = ["gyro x", "gyro y", "gyro z", "accel x", "accel y", "accel z"]
    raws = [g[:, 0], g[:, 1], g[:, 2], a[:, 0], a[:, 1], a[:, 2]]

    lo, hi = (args.zoom if args.zoom else (0.0, fs / 2))
    print(f"  strongest peak in {lo:g}-{hi:g} Hz, both ways   "
          f"(nperseg {args.nperseg})")
    print(f"  {'channel':9s} {'closed-up':>22s} {'resampled':>22s}   {'contrast':>9s}")

    curves = []
    for nm, x in zip(names, raws):
        f1, p1, ns = welch(x, fs, args.nperseg)
        f2, p2, _ = welch(np.interp(grid, t, x), fs, args.nperseg)
        curves.append((nm, f1, p1, f2, p2))
        b = (f1 >= lo) & (f1 <= hi)
        floor1 = np.median(p1[b])
        floor2 = np.median(p2[b])
        r1 = p1[b].max() / floor1
        r2 = p2[b].max() / floor2
        print(f"  {nm:9s} {f1[b][p1[b].argmax()]:9.3f} Hz {r1:8.1f}x "
              f"{f2[b][p2[b].argmax()]:9.3f} Hz {r2:8.1f}x   {r2/r1:8.1f}x")
    print(f"\n  {ns} averaged segments, {fs/args.nperseg:.4f} Hz bins")
    print("  contrast = how much sharper the line is once the real timestamps are")
    print("  respected. Near 1 means the dropouts were not hurting that channel.")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 3, figsize=(16, 7), sharex=True)
    for i, (nm, f1, p1, f2, p2) in enumerate(curves):
        b = ax[i // 3][i % 3]
        b.semilogy(f1, np.maximum(p1, 1e-20), lw=.6, color="tab:red", alpha=.55,
                   label="gaps closed up (buggy)")
        b.semilogy(f2, np.maximum(p2, 1e-20), lw=.7, color="tab:blue",
                   label="resampled (correct)")
        m = (f2 >= lo) & (f2 <= hi)
        pk = f2[m][p2[m].argmax()]
        b.axvline(pk, color="k", ls=":", lw=.8)
        b.annotate(f"{pk:.3f} Hz", (pk, p2[m].max()), fontsize=7,
                   xytext=(4, 2), textcoords="offset points")
        b.set_xlim(lo, hi)
        b.set_title(f"{nm}   [{'(rad/s)^2/Hz' if i < 3 else '(m/s^2)^2/Hz'}]", fontsize=9)
        b.grid(alpha=.3, which="both")
        if i // 3 == 1:
            b.set_xlabel("Hz")
        if i == 0:
            b.legend(fontsize=7, loc="upper right")

    fig.suptitle(f"{os.path.basename(args.bag)}   {100*dropped/ideal:.1f}% samples dropped   "
                 f"nperseg {args.nperseg}", fontsize=10)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"\n  wrote {args.out}")


if __name__ == "__main__":
    main()
