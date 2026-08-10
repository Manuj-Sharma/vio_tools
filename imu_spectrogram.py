#!/usr/bin/env python3
"""imu_spectrogram.py BAG [-o out.png] -- STFT of the IMU: how the spectrum moves.

    python3 imu_spectrogram.py ~/rosbags/field_data/seg01_ros2_baro
    python3 imu_spectrogram.py BAG --band 0 130 --track 80 125
    python3 imu_spectrogram.py BAG --overlay run.fan.csv       # rpm -> Hz on top

WHY NOT WELCH

Welch averages every segment into one curve, which is optimal when the signal is
stationary -- a bench log with a fixed fan tone -- and wrong on a moving
platform. Throttle changes, so prop RPM changes, so the harmonics SWEEP. Welch
smears a tone that slides from 95 to 115 Hz into a broad 20 Hz hump that is
indistinguishable from broadband noise. The STFT keeps the time axis and shows
it as a diagonal streak.

Rule of thumb: if the thing you want to know has the word "when" or "during" in
it -- does vibration track throttle, does it get worse on the climb -- you need
this. If it is "what is the noise floor", use imu_spectrum.py.

RESOLUTION TRADE-OFF. Segment length sets time AND frequency resolution against
each other; you cannot have both. At fs=250:

    nperseg 4096   16.4 s per column   0.061 Hz bins
    nperseg 2048    8.2 s per column   0.122 Hz bins   (default)
    nperseg 1024    4.1 s per column   0.244 Hz bins

Strong lines like prop harmonics survive short segments. Weak ones need length,
and at that point you are back to wanting Welch.

--track LO HI follows the strongest peak inside a band across time and draws it
as a white line, which is the direct test for a sweep: flat means a fixed tone,
sloped means it is tracking something.

Dropouts are resampled out first (see imu_gap_compare.py) -- an STFT closing up
gaps misplaces lines exactly the same way Welch does.
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


def stft(x, fs, nperseg=2048, overlap=0.75):
    """Short-time Fourier transform -> (freqs, times, PSD[freq, time]).

    Same front end as the welch() in imu_gap_compare.py -- Hann window, overlapped
    segments, identical normalisation -- but the segments are kept as columns
    instead of averaged. Averaging this over axis 1 reproduces the Welch PSD.
    """
    x = np.asarray(x, float) - np.mean(x)
    hop = max(1, int(nperseg * (1 - overlap)))
    starts = range(0, len(x) - nperseg + 1, hop)
    w = np.hanning(nperseg)
    scale = 1.0 / (fs * (w ** 2).sum())
    cols = []
    for i in starts:
        p = np.abs(np.fft.rfft(x[i:i + nperseg] * w)) ** 2 * scale
        p[1:-1] *= 2
        cols.append(p)
    S = np.array(cols).T
    f = np.fft.rfftfreq(nperseg, 1 / fs)
    tt = (np.array(list(starts)) + nperseg / 2) / fs
    return f, tt, S


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/imu0")
    ap.add_argument("-o", "--out", default="/tmp/imu_spectrogram.png")
    ap.add_argument("--nperseg", type=int, default=2048)
    ap.add_argument("--overlap", type=float, default=0.75)
    ap.add_argument("--band", nargs=2, type=float, metavar=("LO", "HI"),
                    help="frequency axis limits")
    ap.add_argument("--track", nargs=2, type=float, metavar=("LO", "HI"),
                    help="follow the strongest peak in this band over time")
    ap.add_argument("--overlay", metavar="CSV",
                    help="t_epoch,rpm,... csv; plots rpm/60 as a dashed line")
    args = ap.parse_args()

    t, g, a = read_imu(args.bag, args.topic)
    nominal = float(np.median(np.diff(t)))
    fs = 1.0 / nominal
    dropped = int((np.round(np.diff(t) / nominal).astype(int) - 1).sum())
    grid = t[0] + np.arange(int(round((t[-1] - t[0]) / nominal)) + 1) * nominal

    print(f"  {len(t)} samples over {t[-1]-t[0]:.1f} s, fs {fs:.3f} Hz")
    print(f"  dropped {dropped} ({100*dropped/(len(t)+dropped):.2f}%) -- resampled out")
    print(f"  nperseg {args.nperseg}: {args.nperseg/fs:.1f} s per column, "
          f"{fs/args.nperseg:.3f} Hz bins\n")

    names = ["gyro x", "gyro y", "gyro z", "accel x", "accel y", "accel z"]
    raws = [g[:, 0], g[:, 1], g[:, 2], a[:, 0], a[:, 1], a[:, 2]]

    grids = []
    for nm, x in zip(names, raws):
        f, tt, S = stft(np.interp(grid, t, x), fs, args.nperseg, args.overlap)
        grids.append((nm, f, tt, S))
    print(f"  {len(grids[0][2])} time columns\n")

    if args.track:
        lo, hi = args.track
        b = (grids[0][1] >= lo) & (grids[0][1] <= hi)
        print(f"  peak frequency in {lo:g}-{hi:g} Hz over time")
        print(f"  {'channel':9s} {'start':>9s} {'end':>9s} {'min':>9s} {'max':>9s} "
              f"{'sweep':>9s}")
        for nm, f, tt, S in grids:
            ridge = f[b][S[b].argmax(axis=0)]
            print(f"  {nm:9s} {ridge[0]:8.2f}H {ridge[-1]:8.2f}H {ridge.min():8.2f}H "
                  f"{ridge.max():8.2f}H {ridge.max()-ridge.min():8.2f}H")
        print("\n  a flat ridge is a fixed tone; a sloped one is tracking something")
        print("  (throttle, RPM). Wide min-max with no trend means it is noise and")
        print("  the ridge is just chasing the loudest ripple.\n")

    ov = None
    if args.overlay:
        d = np.genfromtxt(args.overlay, delimiter=",", names=True)
        ov = (d[d.dtype.names[0]] - d[d.dtype.names[0]][0], d["rpm"] / 60.0)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    flo, fhi = (args.band if args.band else (0.0, fs / 2))
    fig, ax = plt.subplots(2, 3, figsize=(17, 8), sharex=True, sharey=True)
    for i, (nm, f, tt, S) in enumerate(grids):
        b = ax[i // 3][i % 3]
        m = (f >= flo) & (f <= fhi)
        db = 10 * np.log10(np.maximum(S[m], 1e-20))
        b.imshow(db, aspect="auto", origin="lower", cmap="magma",
                 extent=[tt[0], tt[-1], f[m][0], f[m][-1]],
                 vmin=np.percentile(db, 5), vmax=np.percentile(db, 99.7))
        if args.track:
            tb = (f >= args.track[0]) & (f <= args.track[1])
            b.plot(tt, f[tb][S[tb].argmax(axis=0)], color="w", lw=.8, alpha=.8)
        if ov is not None:
            b.plot(ov[0], ov[1], "c--", lw=1, alpha=.9)
        b.set_title(f"{nm}   [dB re 1 {'(rad/s)' if i < 3 else '(m/s^2)'}^2/Hz]", fontsize=9)
        if i // 3 == 1:
            b.set_xlabel("t [s]")
        if i % 3 == 0:
            b.set_ylabel("Hz")
    fig.suptitle(f"{os.path.basename(args.bag)}   STFT   nperseg {args.nperseg} "
                 f"({args.nperseg/fs:.1f} s / {fs/args.nperseg:.3f} Hz)", fontsize=10)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"  wrote {args.out}")


if __name__ == "__main__":
    main()
