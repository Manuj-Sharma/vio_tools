#!/usr/bin/env python3
"""imu_plot.py BAG [-o out.png] -- raw gyro and accel against time.

    python3 imu_plot.py ~/rosbags/field_data/imu_static_20260806-185934
    python3 imu_plot.py BAG --t0 100 --t1 101      # one second, to see the shape
    python3 imu_plot.py BAG --gaps                 # mark dropped samples

The time domain, plainly. Six panels sharing an x axis, gyro on top and accel
below, plus mean/std/peak-to-peak per channel on stdout.

ZOOM IN. At full span a 400 s trace is ~100k points drawn over 1500 pixels, so
anything above about 1 Hz is a solid band and you learn nothing from it. Use
--t0/--t1 to pull out a second or two; a 16 Hz oscillation is obvious there and
invisible at full span.

--gaps marks sample dropouts with red ticks along the top. This IMU delivers
~238 Hz against a 250 Hz nominal, so roughly 5% of the grid is missing and the
gaps are not evenly spread.
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


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/imu0")
    ap.add_argument("-o", "--out", default="/tmp/imu_plot.png")
    ap.add_argument("--t0", type=float, help="window start [s from bag start]")
    ap.add_argument("--t1", type=float, help="window end [s from bag start]")
    ap.add_argument("--gaps", action="store_true", help="mark dropped samples")
    ap.add_argument("--detrend", action="store_true",
                    help="subtract each channel's mean (drops gravity off accel z)")
    args = ap.parse_args()

    t, g, a = read_imu(args.bag, args.topic)
    t = t - t[0]
    nominal = float(np.median(np.diff(t)))

    lo = args.t0 if args.t0 is not None else t[0]
    hi = args.t1 if args.t1 is not None else t[-1]
    m = (t >= lo) & (t <= hi)
    if m.sum() < 2:
        raise SystemExit(f"window {lo}-{hi}s selects {m.sum()} samples of {len(t)}")
    tw, gw, aw = t[m], g[m], a[m]

    print(f"  {len(t)} samples over {t[-1]:.1f} s at ~{1/nominal:.1f} Hz nominal")
    print(f"  showing {lo:.2f}-{hi:.2f} s ({m.sum()} samples)\n")
    print(f"  {'channel':9s} {'mean':>12s} {'std':>12s} {'min':>12s} {'max':>12s} {'p-p':>12s}")

    names = ["gyro x", "gyro y", "gyro z", "accel x", "accel y", "accel z"]
    chans = [gw[:, 0], gw[:, 1], gw[:, 2], aw[:, 0], aw[:, 1], aw[:, 2]]
    for nm, x in zip(names, chans):
        print(f"  {nm:9s} {x.mean():12.5f} {x.std():12.5f} {x.min():12.5f} "
              f"{x.max():12.5f} {x.ptp():12.5f}")
    print("\n  gyro rad/s, accel m/s^2")

    if args.detrend:
        chans = [x - x.mean() for x in chans]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 3, figsize=(16, 7), sharex=True)
    for i, (nm, x) in enumerate(zip(names, chans)):
        b = ax[i // 3][i % 3]
        b.plot(tw, x, lw=.5)
        b.set_title(f"{nm}   [{'rad/s' if i < 3 else 'm/s^2'}]", fontsize=9)
        b.grid(alpha=.3)
        if i // 3 == 1:
            b.set_xlabel("t [s]")
        if args.gaps:
            gi = np.flatnonzero(np.diff(tw) > 1.5 * nominal)
            top = b.get_ylim()[1]
            b.plot(tw[gi], np.full(len(gi), top), "r|", ms=6, alpha=.6)

    span = f"{lo:.2f}-{hi:.2f}s" if (args.t0 is not None or args.t1 is not None) else "full span"
    fig.suptitle(f"{os.path.basename(args.bag)}   {span}"
                 f"{'   (mean removed)' if args.detrend else ''}", fontsize=10)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"\n  wrote {args.out}")


if __name__ == "__main__":
    main()
