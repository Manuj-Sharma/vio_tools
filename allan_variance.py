#!/usr/bin/env python3
"""allan_variance.py BAG [-o out.png] -- IMU noise parameters from a static log.

    python3 allan_variance.py ~/rosbags/field_data/imu_static_20260806-185934

Needs the IMU STATIONARY, powered, thermally settled. Any motion or vibration is
indistinguishable from sensor noise and inflates every number here -- which is
exactly the suspicion about the values currently in kalibr_imu_chain.yaml.

WHAT IT COMPUTES

Overlapping Allan deviation of the integrated signal:

    sigma(tau)^2 = 1/(2 (N-2m) tau^2) * sum (theta[k+2m] - 2 theta[k+m] + theta[k])^2

and reads the standard parameters off the curve:

    N  angle/velocity random walk   the -1/2 slope; sigma(tau=1s) IS the noise
                                    density that goes in noise_density
    B  bias instability             the flat minimum, / 0.664
    K  rate random walk             the +1/2 slope; sigma(tau=3s)/sqrt(3) is the
                                    random_walk term

CAVEAT ON TAU RANGE: a 7-minute log resolves the -1/2 slope well and reaches the
bias-instability floor only if it is short-timescale. The +1/2 rate-random-walk
region needs tens of minutes to hours. Values reported past tau ~ T/10 are based
on few independent samples and are indicative only -- the script marks them.

Sampling is assumed uniform at the median rate. This IMU's stamps are quantised
to whole ms with ~5% of samples missing, so tau is accurate to about a percent.
"""
import argparse
import os

import numpy as np


def read_imu(bag, topic="/imu0"):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    t, g, a = [], [], []
    with Reader(bag) as r:
        cons = [c for c in r.connections if c.topic == topic]
        for _, _s, raw in r.messages(connections=cons):
            m = ts.deserialize_cdr(raw, cons[0].msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            g.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
            a.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    return np.array(t), np.array(g), np.array(a)


def allan(x, dt, n_tau=60):
    """Overlapping Allan deviation. x is the rate signal; integrates internally."""
    N = len(x)
    theta = np.cumsum(x) * dt                       # integrate rate -> angle
    maxm = int((N - 1) / 3)
    ms = np.unique(np.floor(np.logspace(0, np.log10(maxm), n_tau)).astype(int))
    taus, devs = [], []
    for m in ms:
        if 2 * m >= N:
            continue
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        tau = m * dt
        var = np.sum(d ** 2) / (2 * tau ** 2 * len(d))
        taus.append(tau)
        devs.append(np.sqrt(var))
    return np.array(taus), np.array(devs)


def params(tau, dev, T):
    """Read N, B, K off the curve."""
    out = {}
    i = np.argmin(np.abs(tau - 1.0))
    out["N"] = dev[i] * np.sqrt(tau[i])             # sigma = N/sqrt(tau) -> N = sigma*sqrt(tau)
    j = np.argmin(dev)
    out["B"] = dev[j] / 0.664
    out["B_tau"] = tau[j]
    k = np.argmin(np.abs(tau - 3.0))
    out["K"] = dev[k] * np.sqrt(3.0 / tau[k]) / np.sqrt(3.0)
    out["reliable_to"] = T / 10
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("-o", "--out", default="/tmp/allan.png")
    ap.add_argument("--topic", default="/imu0")
    args = ap.parse_args()

    t, g, a = read_imu(args.bag, args.topic)
    dt = float(np.median(np.diff(t)))
    T = t[-1] - t[0]
    print(f"  {len(t)} samples, {T:.1f} s, fs {1/dt:.2f} Hz")
    print(f"  curve reliable to tau ~ {T/10:.0f} s; beyond that too few independent samples\n")

    # what is currently configured, for comparison
    CFG = {"gyro": (1.66e-4, 1.66e-5), "accel": (1.19e-3, 1.19e-4)}
    DS = {"gyro": (4.654e-5, 4.654e-6), "accel": (3.333e-4, 3.333e-5)}

    curves = []
    for kind, arr, unit in (("gyro", g, "rad/s"), ("accel", a, "m/s^2")):
        print(f"  {kind.upper()}  [{unit}/sqrt(Hz)]")
        print(f"    {'axis':5s} {'N (noise density)':>19s} {'B (bias instab)':>17s} @tau  "
              f"{'K (rate RW)':>13s}")
        Ns = []
        for i, ax in enumerate("xyz"):
            tau, dev = allan(arr[:, i], dt)
            curves.append((f"{kind} {ax}", tau, dev))
            p = params(tau, dev, T)
            Ns.append(p["N"])
            print(f"    {ax:5s} {p['N']:19.3e} {p['B']:17.3e} {p['B_tau']:5.1f}s {p['K']:13.3e}")
        mean_N = float(np.mean(Ns))
        cfg, ds = CFG[kind][0], DS[kind][0]
        print(f"    mean N {mean_N:.3e}   config {cfg:.3e} ({mean_N/cfg:5.2f}x)   "
              f"datasheet {ds:.3e} ({mean_N/ds:5.2f}x)")
        print()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax2 = plt.subplots(1, 2, figsize=(13, 5))
    for nm, tau, dev in curves:
        k = 0 if nm.startswith("gyro") else 1
        ax2[k].loglog(tau, dev, marker=".", ms=3, lw=.9, label=nm)
    for k, (kind, unit) in enumerate((("gyro", "rad/s"), ("accel", "m/s^2"))):
        b = ax2[k]
        b.axvline(T / 10, color="k", ls=":", lw=.9)
        b.annotate("reliable →|", (T / 10, b.get_ylim()[1]), fontsize=7, ha="right", va="top")
        for lbl, v, c in (("config", CFG[kind][0], "tab:red"),
                          ("datasheet", DS[kind][0], "tab:green")):
            b.loglog(tau, v / np.sqrt(tau), ls="--", lw=1, color=c, alpha=.7,
                     label=f"{lbl} N={v:.2e}")
        b.set_title(f"{kind} Allan deviation  [{unit}]")
        b.set_xlabel("tau [s]"); b.set_ylabel(f"sigma [{unit}]")
        b.grid(alpha=.3, which="both"); b.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(args.out, dpi=110)
    print(f"  wrote {args.out}")


if __name__ == "__main__":
    main()
