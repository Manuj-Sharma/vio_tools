#!/usr/bin/env python3
"""analyse_feature_depth.py CLOUD_BAG [VIO_LOG] -- where does the filter put the ground?

The field is flat, so every triangulated feature belongs at z ~ 0, the launch
plane. Any systematic offset is the filter placing the ground in the wrong place,
and because a nadir camera constrains only the ratio t/d, a depth offset of a
given fraction implies a trajectory scale error of the same fraction.

Measured against GPS on this bag, the trajectory comes out ~10% too big (Umeyama
scale 0.907). If depth inflation is the cause, the features should sit ~10% of
the altitude BELOW ground -- about -11 m at 107 m. If instead they cluster near
zero, the depth is fine and the scale error is entering somewhere else, which
would point at the accelerometer rather than the geometry.

The spread matters as much as the mean: on genuinely flat ground a tight cluster
means triangulation is working, and a wide one means it is returning noise.
"""
import sys

import numpy as np


def read_clouds(path):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    out = {}
    with Reader(path) as r:
        for con, t, raw in r.messages():
            m = ts.deserialize_cdr(raw, con.msgtype)
            n = m.width * m.height
            if n == 0:
                continue
            step = m.point_step
            buf = np.frombuffer(m.data, dtype=np.uint8)
            xyz = np.zeros((n, 3), np.float32)
            offs = {f.name: f.offset for f in m.fields}
            for i, k in enumerate("xyz"):
                o = offs.get(k)
                if o is None:
                    break
                xyz[:, i] = buf.reshape(n, step)[:, o:o + 4].copy().view(np.float32).ravel()
            out.setdefault(con.topic, []).append((t * 1e-9, xyz))
    return out


def hist(z, lo, hi, rows=14, width=54):
    edges = np.linspace(lo, hi, rows + 1)
    cnt, _ = np.histogram(z, bins=edges)
    mx = cnt.max() if cnt.max() else 1
    print(f"      {'z range (m)':>18s}  {'count':>7s}")
    for i in range(rows - 1, -1, -1):
        bar = "#" * int(width * cnt[i] / mx)
        print(f"    {edges[i]:+7.1f}..{edges[i+1]:+7.1f}  {cnt[i]:7d}  {bar}")


def main():
    bagpath = sys.argv[1]
    violog = sys.argv[2] if len(sys.argv) > 2 else None
    clouds = read_clouds(bagpath)
    if not clouds:
        print("  no point clouds recorded -- did the recorder attach in time?")
        return

    alt = None
    if violog:
        import re
        P = [[float(x) for x in m.groups()]
             for m in re.finditer(r"p_IinG = ([-\d.]+),([-\d.]+),([-\d.]+)",
                                  open(violog, errors="ignore").read())]
        if P:
            alt = np.array(P)[:, 2].max()

    for topic, msgs in clouds.items():
        allz = np.concatenate([x[:, 2] for _, x in msgs])
        allxyz = np.concatenate([x for _, x in msgs])
        n = len(allz)
        print(f"=== {topic} : {len(msgs)} clouds, {n} points ===")
        if n < 10:
            print("  too few points\n")
            continue
        # trim the extreme 1% each side -- a handful of wild triangulations
        # would otherwise dominate mean and std and hide the bulk behaviour
        lo, hi = np.percentile(allz, [1, 99])
        k = (allz >= lo) & (allz <= hi)
        z = allz[k]
        print(f"  feature z  : mean {z.mean():+8.2f}   median {np.median(z):+8.2f}   "
              f"std {z.std():7.2f} m")
        print(f"               p5 {np.percentile(z,5):+8.2f}   p95 {np.percentile(z,95):+8.2f}   "
              f"(1-99% trimmed, {n-k.sum()} outliers dropped)")
        print(f"  horizontal : x {allxyz[:,0].min():+8.1f}..{allxyz[:,0].max():+8.1f}   "
              f"y {allxyz[:,1].min():+8.1f}..{allxyz[:,1].max():+8.1f}")
        if alt:
            implied = alt - np.median(z)
            print()
            print(f"  VIO peak altitude          : {alt:8.2f} m")
            print(f"  median feature z           : {np.median(z):+8.2f} m   (flat ground -> expect ~0)")
            print(f"  implied depth to ground    : {implied:8.2f} m")
            print(f"  vs barometric 107.75 m     : {100*(implied/107.75-1):+7.1f}%")
            print(f"  trajectory error vs GPS    :   +10.2%   (Umeyama scale 0.907)")
            print()
            if abs(np.median(z)) < 0.05 * alt:
                print("  -> features are near ground. Depth is NOT the scale error;")
                print("     look at the accelerometer / metric anchor instead.")
            else:
                print("  -> features are off the ground plane by "
                      f"{100*abs(np.median(z))/alt:.0f}% of altitude. If that matches the")
                print("     trajectory error, the depth/translation degeneracy is the cause.")
        print()
        hist(z, np.percentile(z, 2), np.percentile(z, 98))
        print()


if __name__ == "__main__":
    main()
