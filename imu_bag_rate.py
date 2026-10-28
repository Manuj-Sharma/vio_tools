#!/usr/bin/env python3
"""imu_bag_rate.py BAG [--node-log FILE [--lines A:B]] [--label NAME] [--period-ms 4] [--trim 5]

How many /imu0 samples does a bag really hold, and where are the holes?

Reads the .db3 files read-only (immutable=1, safe on a bag that is still being written) and
prints one line: message count, average Hz over the trimmed window, per-second min/median,
the % of 4 ms grid slots that are missing (from the header stamps, which sit on the IMX's own
sample grid), and the worst gap.

--node-log: imx5_spi_node's own "/imu0 N Hz | rxPkt=.. rxErr=.." lines (every 5 s). That is the
rate the DRIVER published, before DDS and before any recorder. If the driver says ~250 Hz and the
bag says 160 Hz, the samples were lost after publishing; if the driver itself says 160 Hz, the
loss is in the SPI polling. --lines A:B limits it to log lines A..B (1-based, inclusive) so one
long sensors.log can be sliced per recording window.
"""
import argparse, glob, os, re, sqlite3, statistics as st, struct, sys


def load(bag):
    rx, hs = [], []
    for f in sorted(glob.glob(os.path.join(bag, "*.db3"))):
        c = sqlite3.connect(f"file:{f}?immutable=1", uri=True)
        t = c.execute("select id from topics where name='/imu0'").fetchone()
        if not t:
            continue
        for ts, data in c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (t[0],)):
            s, n = struct.unpack_from("<iI", data, 4)
            rx.append(ts)
            hs.append(s * 10**9 + n)
    return rx, hs


def cam_rates(bag, t0, t1):
    """{'cam0': Hz, 'cam1': Hz} over [t0, t1] (bag receive time, ns); only cameras present in the bag."""
    cnt = {}
    for f in sorted(glob.glob(os.path.join(bag, "*.db3"))):
        c = sqlite3.connect(f"file:{f}?immutable=1", uri=True)
        for k in ("cam0", "cam1"):
            t = c.execute("select id from topics where name=?", (f"/{k}/image_raw",)).fetchone()
            if t:
                n = c.execute("select count(*) from messages where topic_id=? and timestamp>=? and timestamp<=?",
                              (t[0], int(t0), int(t1))).fetchone()[0]
                cnt[k] = cnt.get(k, 0) + n
    span = (t1 - t0) / 1e9
    return {k: n / span for k, n in cnt.items()}


def cam_delivery(bag, t0, t1, topic="/cam0/image_raw"):
    """How evenly do camera frames ARRIVE at the recorder? Frames can be complete (header stamps
    regular) yet delivered in bursts. Returns (receive gaps > 42 ms, max receive gap ms,
    delivery latency p95 ms, max ms above its minimum, % of arrivals < 2 ms after the previous)
    or None if the topic is absent."""
    rx, hs = [], []
    for f in sorted(glob.glob(os.path.join(bag, "*.db3"))):
        c = sqlite3.connect(f"file:{f}?immutable=1", uri=True)
        t = c.execute("select id from topics where name=?", (topic,)).fetchone()
        if not t:
            continue
        for ts, h in c.execute("select timestamp, substr(data,5,8) from messages "
                               "where topic_id=? and timestamp>=? and timestamp<=? order by timestamp",
                               (t[0], int(t0), int(t1))):
            s, n = struct.unpack("<iI", h)
            rx.append(ts)
            hs.append(s * 10**9 + n)
    if len(rx) < 50:
        return None
    gaps = [(b - a) / 1e6 for a, b in zip(rx, rx[1:])]
    lat = [(r - h) / 1e6 for r, h in zip(rx, hs)]
    m = min(lat)
    lat = sorted(x - m for x in lat)
    return (sum(1 for g in gaps if g > 42), max(gaps), lat[int(0.95 * len(lat))], lat[-1],
            100.0 * sum(1 for g in gaps if g < 2) / len(gaps))


def driver_rates(path, lines):
    pat = re.compile(r"/imu0 ([\d.]+) Hz \| rxPkt=(\d+) rxErr=(\d+)")
    out = []
    with open(path, errors="replace") as fh:
        for i, line in enumerate(fh, 1):
            if lines and not (lines[0] <= i <= lines[1]):
                continue
            m = pat.search(line)
            if m:
                out.append((float(m.group(1)), int(m.group(3))))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--node-log")
    ap.add_argument("--lines")
    ap.add_argument("--label", default=None)
    ap.add_argument("--period-ms", type=float, default=4.0)
    ap.add_argument("--trim", type=float, default=5.0, help="seconds dropped from each end")
    ap.add_argument("--min-cam-hz", type=float, default=0.0,
                    help="flag the line 'CAMERA LOW' if a camera is below this (or absent); 0 = just report")
    a = ap.parse_args()

    label = a.label or os.path.basename(a.bag.rstrip("/"))
    rx, hs = load(a.bag)
    if len(rx) < 100:
        print(f"{label}: only {len(rx)} /imu0 messages")
        return 1
    t0, t1 = rx[0] + a.trim * 1e9, rx[-1] - a.trim * 1e9
    idx = [i for i, t in enumerate(rx) if t0 <= t <= t1]
    if len(idx) < 100:
        print(f"{label}: window too short after trimming ({len(idx)} msgs)")
        return 1
    r = [rx[i] for i in idx]
    h = [hs[i] for i in idx]
    span = (r[-1] - r[0]) / 1e9
    avg = len(r) / span
    sec = {}
    for t in r:
        k = int((t - r[0]) / 1e9)
        sec[k] = sec.get(k, 0) + 1
    per = [sec.get(k, 0) for k in range(int(span))]
    dh = [(b - x) / 1e6 for x, b in zip(h, h[1:])]
    missed = sum(max(0, round(d / a.period_ms) - 1) for d in dh if d > 1.5 * a.period_ms)
    expected = len(r) + missed
    line = (f"{label:<22} {len(r):7d} msgs {span:6.1f}s  avg {avg:6.1f} Hz  per-s min {min(per):3d} med {st.median(per):5.0f}"
            f"  missed {100 * missed / expected:5.1f}%  worst gap {max(dh):7.1f} ms")
    cams = cam_rates(a.bag, t0, t1)
    if cams:
        line += "  | " + "  ".join(f"{n} {hz:5.1f} Hz" for n, hz in cams.items())
        if a.min_cam_hz and any(hz < a.min_cam_hz for hz in cams.values()):
            line += f"  <-- CAMERA LOW (< {a.min_cam_hz:g} Hz)"
    elif a.min_cam_hz:
        line += "  <-- CAMERA LOW (no camera messages in this bag)"
    dv = cam_delivery(a.bag, t0, t1) if cams else None
    if dv:
        line += (f"  | cam0 delivery: gaps>42ms {dv[0]}, max gap {dv[1]:.0f} ms, latency p95 {dv[2]:.0f} ms"
                 f" max {dv[3]:.0f} ms, back-to-back {dv[4]:.1f}%")
    if a.node_log and os.path.exists(a.node_log):
        lines = tuple(int(x) for x in a.lines.split(":")) if a.lines else None
        d = driver_rates(a.node_log, lines)
        if d:
            line += f"  | driver {st.mean(x for x, _ in d):6.1f} Hz (min {min(x for x, _ in d):.0f}, {len(d)} reports, rxErr+{d[-1][1] - d[0][1]})"
        else:
            line += "  | driver: no stats lines in that range"
    elif a.node_log:
        line += "  | driver log not found"
    print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
