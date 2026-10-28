#!/usr/bin/env python3
"""bag_hz_windows.py BAG [--win 20]

How does the rate vary ACROSS a recording? One row per WIN-second window (default 20 s), measured
on the bag's receive timestamps:

    imu Hz / miss% / worst gap   IMU rate, % of 4 ms grid slots missing (from the header stamps,
                                 so a late burst does not hide a lost sample), worst receive gap
    cam0, cam1  Hz / worst gap   camera rate and worst receive gap

Reads the .db3 files read-only (immutable=1). Only the /imu0 header is read from the blobs, so it
is fast even on a 20 GB bag. The last window is shorter; its Hz uses its real length.
"""
import argparse, glob, os, sqlite3, struct, statistics as st, sys


def load(bag, topic, with_header=False):
    rx, hs = [], []
    for f in sorted(glob.glob(os.path.join(bag, "*.db3"))):
        c = sqlite3.connect(f"file:{f}?immutable=1", uri=True)
        t = c.execute("select id from topics where name=?", (topic,)).fetchone()
        if not t:
            continue
        if with_header:
            for ts, h in c.execute("select timestamp, substr(data,5,8) from messages where topic_id=? order by timestamp", (t[0],)):
                s, n = struct.unpack("<iI", h)
                rx.append(ts); hs.append(s * 10**9 + n)
        else:
            rx += [r[0] for r in c.execute("select timestamp from messages where topic_id=? order by timestamp", (t[0],))]
    if with_header:
        order = sorted(range(len(rx)), key=rx.__getitem__)
        return [rx[i] for i in order], [hs[i] for i in order]
    return sorted(rx), None


def window_stats(rx, hs, lo, hi, period_ms):
    idx = [i for i, t in enumerate(rx) if lo <= t < hi]
    n = len(idx)
    if n < 2:
        return n, None, None, None
    r = [rx[i] for i in idx]
    # a gap belongs to the window where it ENDS, so one straddling a window boundary is not lost
    worst = max((rx[i] - rx[i - 1]) / 1e6 for i in idx if i > 0)
    miss = None
    if hs is not None:
        lost = sum(max(0, round(((hs[i] - hs[i - 1]) / 1e6) / period_ms) - 1) for i in idx
                   if i > 0 and (hs[i] - hs[i - 1]) / 1e6 > 1.5 * period_ms)
        miss = 100.0 * lost / (n + lost)
    return n, worst, miss, r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--win", type=float, default=20.0)
    a = ap.parse_args()

    imu, imu_h = load(a.bag, "/imu0", with_header=True)
    c0, _ = load(a.bag, "/cam0/image_raw")
    c1, _ = load(a.bag, "/cam1/image_raw")
    if len(imu) < 100:
        print("no /imu0 in this bag"); return 1
    t0 = imu[0]
    end = max(imu[-1], c0[-1] if c0 else 0, c1[-1] if c1 else 0)
    W = int(a.win * 1e9)
    rows = []
    k = 0
    while t0 + k * W < end:
        lo, hi = t0 + k * W, min(t0 + (k + 1) * W, end + 1)
        span = (hi - lo) / 1e9
        if span < 5:          # a tail shorter than 5 s says little
            break
        ni, wi, mi, _ = window_stats(imu, imu_h, lo, hi, 4.0)
        n0, w0, _, _ = window_stats(c0, None, lo, hi, 16.7) if c0 else (0, None, None, None)
        n1, w1, _, _ = window_stats(c1, None, lo, hi, 16.7) if c1 else (0, None, None, None)
        rows.append((k * a.win, span, ni / span, mi, wi, n0 / span, w0, n1 / span, w1))
        k += 1

    f = lambda x, p="{:.0f}": "  -  " if x is None else p.format(x)
    print(f"bag {os.path.basename(a.bag.rstrip('/'))}   {len(imu)} IMU msgs, windows of {a.win:g} s (first IMU message = 0)")
    print(f"{'win':>9} {'span':>5} | {'imu Hz':>7} {'miss%':>6} {'gap ms':>7} | {'cam0 Hz':>7} {'gap ms':>7} | {'cam1 Hz':>7} {'gap ms':>7}")
    for s, span, ih, mi, wi, h0, w0, h1, w1 in rows:
        print(f"{s:4.0f}-{s + span:<4.0f} {span:5.1f} | {ih:7.1f} {f(mi, '{:.1f}'):>6} {f(wi):>7} | {h0:7.1f} {f(w0):>7} | {h1:7.1f} {f(w1):>7}")
    if rows:
        col = lambda i: [r[i] for r in rows if r[i] is not None]
        print(f"{'':>15}   imu Hz  min {min(col(2)):.1f}  median {st.median(col(2)):.1f}  max {max(col(2)):.1f}"
              f"   | cam0 min {min(col(5)):.1f}  cam1 min {min(col(7)):.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
