#!/usr/bin/env python3
"""imu_window_probe.py BAG --win T0 T1 [--win T0 T1 ...] -- what happens in a window.

    python3 imu_window_probe.py ~/rosbags/field_data/seg01_ros2_baro \
            --win 50 75 --win 128 150

Built to chase the two quiet stripes in seg01's spectrogram. Draws, on one time
axis, the things that would explain a broadband vibration drop, and shades the
windows of interest so they can be read off against everything else:

  1  50-125 Hz vibration level          the thing being explained
  2  yaw rate (gyro z) and integrated   box corners are yaw-rate spikes
     yaw
  3  |accel| - g                        thrust/load factor proxy. Vibration
                                        scales with rotor thrust, so a genuine
                                        thrust reduction should show here.
  4  tilt from the gravity vector       bank angle during a turn
  5  altitude and climb rate            flight profile for context

Then prints each window's statistics against the rest of the flight.

CAVEATS. Yaw is integrated open-loop from gyro z with no bias removal, so it
drifts -- read the SHAPE (when it turns) not the absolute heading. Tilt assumes
the accel is dominated by gravity, which fails under strong acceleration. Both
are adequate for "is there a turn here", not for attitude estimation.
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
        for con, _s, raw in r.messages(connections=list(want.values())):
            m = ts.deserialize_cdr(raw, con.msgtype)
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if con.topic == imu_topic:
                t.append(s)
                g.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
                a.append([m.linear_acceleration.x, m.linear_acceleration.y,
                          m.linear_acceleration.z])
            else:
                at.append(s); az.append(m.point.z)
    return np.array(t), np.array(g), np.array(a), np.array(at), np.array(az)


def stft(x, fs, nperseg=2048, overlap=0.75):
    x = np.asarray(x, float) - np.mean(x)
    hop = max(1, int(nperseg * (1 - overlap)))
    starts = list(range(0, len(x) - nperseg + 1, hop))
    w = np.hanning(nperseg)
    scale = 1.0 / (fs * (w ** 2).sum())
    cols = [np.abs(np.fft.rfft(x[i:i + nperseg] * w)) ** 2 * scale for i in starts]
    for c in cols:
        c[1:-1] *= 2
    return (np.fft.rfftfreq(nperseg, 1 / fs),
            (np.array(starts) + nperseg / 2) / fs, np.array(cols).T)


def smooth(x, n):
    n = max(1, int(n) | 1)
    return np.convolve(x, np.ones(n) / n, mode="same")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--win", nargs=2, type=float, action="append", metavar=("T0", "T1"),
                    required=True)
    ap.add_argument("-o", "--out", default="/tmp/imu_window_probe.png")
    ap.add_argument("--nperseg", type=int, default=2048)
    args = ap.parse_args()

    t, g, a, at, az = read_bag(args.bag)
    nom = float(np.median(np.diff(t)))
    fs = 1.0 / nom
    grid = t[0] + np.arange(int(round((t[-1] - t[0]) / nom)) + 1) * nom
    tg = grid - t[0]
    G = np.array([np.interp(grid, t, g[:, i]) for i in range(3)])
    A = np.array([np.interp(grid, t, a[:, i]) for i in range(3)])

    f, tt, S = stft(A[2], fs, args.nperseg)
    lvl = 10 * np.log10(np.maximum(S[f >= 50].mean(axis=0), 1e-20))

    yaw_rate = np.degrees(smooth(G[2], fs * 0.5))
    yaw = np.degrees(np.cumsum(G[2]) * nom)
    amag = smooth(np.linalg.norm(A, axis=0), fs * 0.5)
    tilt = np.degrees(np.arccos(np.clip(smooth(A[2], fs * 0.5) /
                                        np.maximum(amag, 1e-6), -1, 1)))
    ats = at - t[0]
    climb = np.gradient(smooth(az, 15), ats)

    print(f"  {len(t)} samples, {tg[-1]:.1f} s, fs {fs:.2f} Hz\n")
    series = [("vib 50-125 [dB]", tt, lvl), ("|yaw rate| [deg/s]", tg, np.abs(yaw_rate)),
              ("|accel|-g [m/s^2]", tg, amag - 9.80), ("tilt [deg]", tg, tilt),
              ("altitude [m]", ats, az)]
    wins = [tuple(w) for w in args.win]
    print(f"  {'series':20s} " + "".join(f"{f'{a:g}-{b:g}s':>14s}" for a, b in wins) +
          f"{'REST':>14s}")
    for nm, tx, v in series:
        row = f"  {nm:20s}"
        inw = np.zeros(len(tx), bool)
        for a0, b0 in wins:
            m = (tx >= a0) & (tx <= b0)
            inw |= m
            row += f"{v[m].mean():14.2f}"
        row += f"{v[~inw].mean():14.2f}"
        print(row)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(5, 1, figsize=(15, 13), sharex=True)
    panels = [(lvl, tt, "50-125 Hz vibration [dB]", "tab:purple"),
              (yaw_rate, tg, "yaw rate (gyro z) [deg/s]", "tab:red"),
              (amag - 9.80, tg, "|accel| - g  [m/s^2]", "tab:green"),
              (tilt, tg, "tilt from vertical [deg]", "tab:orange"),
              (az, ats, "baro altitude [m]", "tab:blue")]
    for i, (v, tx, lbl, c) in enumerate(panels):
        ax[i].plot(tx, v, lw=.9, color=c)
        ax[i].set_ylabel(lbl, fontsize=9)
        ax[i].grid(alpha=.3)
        for a0, b0 in wins:
            ax[i].axvspan(a0, b0, color="tab:cyan", alpha=.18, zorder=0)
    ax[1].plot(tg, yaw / 10, lw=.8, color="k", alpha=.5, label="integrated yaw /10 [deg]")
    ax[1].legend(fontsize=8)
    ax[1].axhline(0, color="k", lw=.5, ls=":")
    ax[-1].set_xlabel("t [s]")
    fig.suptitle(f"{os.path.basename(args.bag)}   shaded: " +
                 ", ".join(f"{a:g}-{b:g}s" for a, b in wins), fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"\n  wrote {args.out}")


if __name__ == "__main__":
    main()
