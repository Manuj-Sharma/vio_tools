#!/usr/bin/env python3
"""imu_vibe_vs_alt.py BAG [-o out.png] -- does IMU vibration track the flight phase?

    python3 imu_vibe_vs_alt.py ~/rosbags/field_data/seg01_ros2_baro

THE QUESTION

The STFT of seg01 shows the broadband IMU floor swinging 20 dB across the flight
-- quiet around t~57-60 s and t~131-142 s, loud around t~90-107 s. That is a
factor of 110 in power, roughly 10x in amplitude. OpenVINS takes ONE fixed
noise_density for the whole run, so if the real noise moves by 10x the filter is
over-trusting the IMU in some phases and under-trusting it in others.

This plots the vibration level against barometric altitude and climb rate to see
what the swing is actually driven by.

WHAT IT DRAWS

  row 1  accel z spectrogram -- the raw picture the level is extracted from
  row 2  broadband level per band, dB, over time. 0-5 Hz is vehicle motion,
         5-50 and 50-125 are vibration.
  row 3  barometric altitude and climb rate on a shared time axis
  row 4  scatter of vibration against altitude and against |climb rate|,
         with Pearson r

Dropouts are resampled out first. Level is the mean PSD in each band per STFT
column, so it is a power average, not a peak -- immune to any single line.

CAVEAT: correlation here is not causation and the sample is one flight. Climb
rate and throttle are strongly related but not the same thing, and this bag has
no throttle channel, so "vibration tracks throttle" stays an inference.
"""
import argparse
import os

import numpy as np


def read_bag(bag, imu_topic="/imu0", alt_topic="/baro/rel_altitude"):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    t, g, a, at, az = [], [], [], [], []
    with Reader(bag) as r:
        want = {c.topic: c for c in r.connections if c.topic in (imu_topic, alt_topic)}
        if imu_topic not in want:
            raise SystemExit(f"no {imu_topic} in {bag}")
        for con, _stamp, raw in r.messages(connections=list(want.values())):
            m = ts.deserialize_cdr(raw, con.msgtype)
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if con.topic == imu_topic:
                t.append(s)
                g.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
                a.append([m.linear_acceleration.x, m.linear_acceleration.y,
                          m.linear_acceleration.z])
            else:
                at.append(s)
                az.append(m.point.z)
    return (np.array(t), np.array(g), np.array(a),
            np.array(at), np.array(az))


def stft(x, fs, nperseg=2048, overlap=0.75):
    x = np.asarray(x, float) - np.mean(x)
    hop = max(1, int(nperseg * (1 - overlap)))
    starts = list(range(0, len(x) - nperseg + 1, hop))
    w = np.hanning(nperseg)
    scale = 1.0 / (fs * (w ** 2).sum())
    cols = []
    for i in starts:
        p = np.abs(np.fft.rfft(x[i:i + nperseg] * w)) ** 2 * scale
        p[1:-1] *= 2
        cols.append(p)
    return (np.fft.rfftfreq(nperseg, 1 / fs),
            (np.array(starts) + nperseg / 2) / fs,
            np.array(cols).T)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("-o", "--out", default="/tmp/imu_vibe_vs_alt.png")
    ap.add_argument("--nperseg", type=int, default=2048)
    ap.add_argument("--channel", default="accel z",
                    choices=["gyro x", "gyro y", "gyro z", "accel x", "accel y", "accel z"])
    ap.add_argument("--smooth", type=float, default=3.0,
                    help="climb-rate smoothing window [s]")
    args = ap.parse_args()

    t, g, a, at, az = read_bag(args.bag)
    nominal = float(np.median(np.diff(t)))
    fs = 1.0 / nominal
    dropped = int((np.round(np.diff(t) / nominal).astype(int) - 1).sum())
    grid = t[0] + np.arange(int(round((t[-1] - t[0]) / nominal)) + 1) * nominal
    t0 = t[0]

    print(f"  {len(t)} IMU samples over {t[-1]-t[0]:.1f} s, fs {fs:.3f} Hz")
    print(f"  dropped {dropped} ({100*dropped/(len(t)+dropped):.2f}%) -- resampled out")
    print(f"  {len(at)} baro samples, altitude {az.min():.1f} to {az.max():.1f} m\n")

    chans = dict(zip(["gyro x", "gyro y", "gyro z", "accel x", "accel y", "accel z"],
                     [g[:, 0], g[:, 1], g[:, 2], a[:, 0], a[:, 1], a[:, 2]]))
    f, tt, S = stft(np.interp(grid, t, chans[args.channel]), fs, args.nperseg)

    BANDS = [(0, 5, "0-5 Hz  vehicle motion"),
             (5, 50, "5-50 Hz  vibration"),
             (50, fs / 2, f"50-{fs/2:.0f} Hz  vibration")]
    levels = {}
    for lo, hi, lbl in BANDS:
        b = (f >= lo) & (f < hi)
        levels[lbl] = 10 * np.log10(np.maximum(S[b].mean(axis=0), 1e-20))

    # altitude and climb rate on the STFT time base
    ats = at - t0
    alt_i = np.interp(tt, ats, az)
    w = max(3, int(args.smooth / np.median(np.diff(ats))) | 1)
    az_s = np.convolve(az, np.ones(w) / w, mode="same")
    climb = np.gradient(az_s, ats)
    climb_i = np.interp(tt, ats, climb)

    print(f"  {'band':26s} {'range dB':>10s} {'swing':>8s}  "
          f"{'r vs alt':>9s} {'r vs |climb|':>13s}")
    for lbl, L in levels.items():
        r_a = np.corrcoef(L, alt_i)[0, 1]
        r_c = np.corrcoef(L, np.abs(climb_i))[0, 1]
        print(f"  {lbl:26s} {L.min():5.1f}/{L.max():5.1f} {L.max()-L.min():7.1f} "
              f"{r_a:9.3f} {r_c:13.3f}")
    print("\n  r near +/-1 means the level is driven by that quantity; near 0 means"
          "\n  the swing is something else (throttle without climb, wind, turns).")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(15, 13))
    gs = fig.add_gridspec(4, 2, height_ratios=[1.3, 1, 1, 1.1], hspace=.32, wspace=.22)

    ax0 = fig.add_subplot(gs[0, :])
    db = 10 * np.log10(np.maximum(S, 1e-20))
    ax0.imshow(db, aspect="auto", origin="lower", cmap="magma",
               extent=[tt[0], tt[-1], f[0], f[-1]],
               vmin=np.percentile(db, 5), vmax=np.percentile(db, 99.7))
    ax0.set_ylabel("Hz"); ax0.set_title(f"{args.channel} spectrogram", fontsize=10)

    ax1 = fig.add_subplot(gs[1, :], sharex=ax0)
    for lbl, L in levels.items():
        ax1.plot(tt, L, lw=1.3, label=lbl)
    ax1.set_ylabel("band power [dB]"); ax1.grid(alpha=.3)
    ax1.legend(fontsize=8, ncol=3); ax1.set_title("broadband level per band", fontsize=10)

    ax2 = fig.add_subplot(gs[2, :], sharex=ax0)
    ax2.plot(ats, az, color="tab:blue", lw=1.4, label="baro altitude")
    ax2.set_ylabel("altitude [m]", color="tab:blue"); ax2.grid(alpha=.3)
    ax2b = ax2.twinx()
    ax2b.plot(ats, climb, color="tab:red", lw=1, alpha=.75, label="climb rate")
    ax2b.axhline(0, color="k", lw=.6, ls=":")
    ax2b.set_ylabel("climb rate [m/s]", color="tab:red")
    ax2.set_xlabel("t [s]"); ax2.set_title("flight profile", fontsize=10)

    lbl_hi = BANDS[2][2]
    for k, (xv, xl) in enumerate(((alt_i, "altitude [m]"),
                                  (np.abs(climb_i), "|climb rate| [m/s]"))):
        axs = fig.add_subplot(gs[3, k])
        axs.scatter(xv, levels[lbl_hi], s=14, c=tt, cmap="viridis", alpha=.85)
        axs.set_xlabel(xl); axs.set_ylabel(f"{lbl_hi} [dB]"); axs.grid(alpha=.3)
        axs.set_title(f"r = {np.corrcoef(levels[lbl_hi], xv)[0,1]:+.3f}   "
                      f"(colour = time)", fontsize=9)

    fig.suptitle(f"{os.path.basename(args.bag)}   vibration vs flight phase   "
                 f"nperseg {args.nperseg} ({args.nperseg/fs:.1f} s columns)", fontsize=11)
    fig.savefig(args.out, dpi=110, bbox_inches="tight")
    print(f"\n  wrote {args.out}")


if __name__ == "__main__":
    main()
