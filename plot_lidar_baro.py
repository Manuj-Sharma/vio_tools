#!/usr/bin/env python3
"""plot_lidar_baro.py -- compare the AGL lidar against the barometer (and GPS).

    python3 plot_lidar_baro.py                       # defaults below
    python3 plot_lidar_baro.py --bag PATH -o out.png

WHY THIS MATTERS

A downward rangefinder measures height above the ACTUAL terrain, with no weather
drift and no reference-offset bookkeeping -- strictly better than a barometer as
a depth constraint for the nadir-camera degeneracy, IF it reaches the ground.

On this flight it mostly does not. The sensor reports max_range = 95 m and the
mission cruises at ~107 m, so 66% of samples come back `inf`. The returns cluster
in the climb and the descent; the entire box -- which is where the depth/scale
error accumulates -- has no lidar at all. That is the first thing this plot shows,
and it is the thing that decides whether the lidar is usable for the constraint.

WHAT IS PLOTTED

  left    altitude vs time: valid lidar returns, barometer, GPS. Grey bands mark
          where the lidar is saturated (no return).
  middle  lidar minus barometer where both are usable -- the residual is terrain
          relief plus mounting offset plus sensor error, and it bounds how well a
          flat-ground assumption holds.
  right   fraction of valid returns binned by altitude -- the sensor's usable
          envelope, measured rather than taken from the datasheet.
"""
import argparse
import os
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore")
BAG = "/home/vc/flight_20260730-141353_ros2_lidar"
XLSX = "/home/vc/rosbags/seg01_fc.xlsx"


def read_bag(bag):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    lid, bar, meta = [], [], None
    with Reader(bag) as r:
        for topic, sink in (("/lidar", lid), ("/baro/alt", bar)):
            cons = [c for c in r.connections if c.topic == topic]
            if not cons:
                continue
            for _, t, raw in r.messages(connections=cons):
                m = ts.deserialize_cdr(raw, cons[0].msgtype)
                if topic == "/lidar":
                    sink.append((t / 1e9, m.range))
                    if meta is None:
                        meta = (m.min_range, m.max_range)
                else:
                    sink.append((t / 1e9, m.point.z))
    return np.array(lid), np.array(bar), meta


def read_gps(path):
    import pandas as pd
    df = pd.read_excel(path, header=4)
    df = df[pd.to_numeric(df["t_bag"], errors="coerce").notna()]
    for c in ("t_bag", "gps_alt_m"):
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=["t_bag", "gps_alt_m"])
    t = df["t_bag"].values
    z = df["gps_alt_m"].values
    return t, z - z[t < t[0] + 1.0].mean()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", default=BAG)
    ap.add_argument("--xlsx", default=XLSX)
    ap.add_argument("-o", "--out", default="/tmp/lidar_vs_baro.png")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    L, B, meta = read_bag(args.bag)
    if not len(L):
        sys.exit(f"no /lidar in {args.bag}")
    t, r = L[:, 0], L[:, 1]
    tb, zb = B[:, 0], B[:, 1]
    zb = zb - zb[tb < tb[0] + 1.0].mean()
    t0 = t[0]
    valid = np.isfinite(r)
    bi = np.interp(t, tb, zb)
    maxr = meta[1] if meta else np.nan

    print(f"  lidar: {len(r)} samples, {100*valid.mean():.1f}% valid, max_range {maxr:.1f} m")
    both = valid & (bi > 2)
    d = r[both] - bi[both]
    print(f"  lidar-baro where both usable ({both.sum()} samples): "
          f"mean {d.mean():+.2f} m, rms {np.sqrt((d**2).mean()):.2f} m, "
          f"corr {np.corrcoef(r[both], bi[both])[0,1]:.4f}")

    fig, ax = plt.subplots(1, 3, figsize=(17, 5.2))

    # ---- left: altitude vs time
    # shade the saturated stretches so the coverage gap is unmissable
    inval = ~valid
    edges = np.diff(inval.astype(int))
    starts = list(np.where(edges == 1)[0] + 1) + ([0] if inval[0] else [])
    ends = list(np.where(edges == -1)[0] + 1) + ([len(inval) - 1] if inval[-1] else [])
    for s, e in zip(sorted(starts), sorted(ends)):
        if e - s > 20:
            ax[0].axvspan(t[s] - t0, t[e] - t0, color="0.85", zorder=0)
    ax[0].plot(tb - t0, zb, "-", color="tab:blue", lw=1.6, label="barometer")
    try:
        tg, zg = read_gps(args.xlsx)
        ax[0].plot(tg - t0, zg, "-", color="tab:green", lw=1.2, alpha=.8, label="GPS")
    except Exception as e:
        print(f"  (no GPS overlay: {e})")
    ax[0].plot(t[valid] - t0, r[valid], ".", color="tab:red", ms=2.5, label="lidar (valid)")
    if np.isfinite(maxr):
        ax[0].axhline(maxr, color="tab:red", ls="--", lw=1,
                      label=f"lidar max_range {maxr:.0f} m")
    ax[0].set_title("altitude — grey = lidar saturated")
    ax[0].set_xlabel("t [s]"); ax[0].set_ylabel("height [m]")
    ax[0].grid(alpha=.3); ax[0].legend(fontsize=8)

    # ---- middle: residual
    ax[1].axhline(0, color="k", lw=.8)
    ax[1].plot(t[both] - t0, d, ".", ms=2.5, color="tab:purple")
    ax[1].set_title(f"lidar − barometer   (mean {d.mean():+.2f}, rms {np.sqrt((d**2).mean()):.2f} m)")
    ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("difference [m]"); ax[1].grid(alpha=.3)

    # ---- right: validity envelope
    bins = np.arange(0, 120, 10)
    frac, cnt = [], []
    for lo, hi in zip(bins[:-1], bins[1:]):
        m = (bi >= lo) & (bi < hi)
        frac.append(100 * valid[m].mean() if m.sum() else np.nan)
        cnt.append(m.sum())
    ax[2].bar(bins[:-1] + 5, frac, width=9, color="tab:red", alpha=.75)
    if np.isfinite(maxr):
        ax[2].axvline(maxr, color="k", ls="--", lw=1, label=f"max_range {maxr:.0f} m")
        ax[2].legend(fontsize=8)
    ax[2].set_title("valid returns vs altitude")
    ax[2].set_xlabel("barometric altitude [m]"); ax[2].set_ylabel("% valid")
    ax[2].set_ylim(0, 105); ax[2].grid(alpha=.3, axis="y")

    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"  wrote {args.out}")


if __name__ == "__main__":
    main()
