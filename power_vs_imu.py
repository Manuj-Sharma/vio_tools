#!/usr/bin/env python3
"""power_vs_imu.py POWER_CSV BAG_DIR [--topic /imu0]

Aligns the 1 Hz power_logger.py CSV with the IMU arrival rate in a rosbag2 (sqlite3) bag, both on
the wall clock (bag receive time vs time.time() in the logger). For each power sample at t the IMU
rate is the message count in [t-0.5, t+0.5). Prints the IMU rate split by CPU-min-clock state,
by load/Tj terciles, and the Pearson correlations. Read-only.
"""
import glob, sqlite3, statistics as st, sys, csv, bisect

topic = sys.argv[sys.argv.index("--topic") + 1] if "--topic" in sys.argv else "/imu0"
pcsv, bag = sys.argv[1], sys.argv[2]

ts = []
for f in sorted(glob.glob(bag + "/*.db3")):
    c = sqlite3.connect(f"file:{f}?immutable=1", uri=True)
    t = c.execute("select id from topics where name=?", (topic,)).fetchone()
    if t:
        ts += [r[0] / 1e9 for r in c.execute("select timestamp from messages where topic_id=? order by timestamp", (t[0],))]
ts.sort()
rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(pcsv))]
lo, hi = ts[0] + 3, ts[-1] - 1            # skip the start-up gap
rows = [r for r in rows if lo <= r["t"] <= hi]
for r in rows:
    r["hz"] = bisect.bisect_left(ts, r["t"] + 0.5) - bisect.bisect_left(ts, r["t"] - 0.5)


def corr(a, b):
    ma, mb = st.mean(a), st.mean(b)
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    den = (sum((x - ma) ** 2 for x in a) * sum((y - mb) ** 2 for y in b)) ** 0.5
    return num / den if den else float("nan")


def summ(name, sel):
    xs = [r["hz"] for r in sel]
    if xs:
        print(f"  {name:34s} n={len(xs):4d}  IMU Hz mean {st.mean(xs):6.1f}  median {st.median(xs):6.1f}  min {min(xs):4.0f}")


print(f"{bag}\n{len(rows)} aligned seconds (power CSV {pcsv})")
low = [r for r in rows if r["cpu_min"] < 1000]
summ("all seconds", rows)
summ("cpu_min < 1000 MHz (core at 730)", low)
summ("cpu_min >= 1000 MHz", [r for r in rows if r["cpu_min"] >= 1000])
# does the dip PRECEDE/FOLLOW a rate drop? shift the clock state by -2..+2 s
byi = {i: r for i, r in enumerate(rows)}
print("  IMU Hz vs cpu_min<1000 shifted by k s (k>0: dip happens k s BEFORE the rate sample):")
for k in (-2, -1, 0, 1, 2):
    a = [byi[i]["hz"] for i in byi if (i - k) in byi and byi[i - k]["cpu_min"] < 1000]
    b = [byi[i]["hz"] for i in byi if (i - k) in byi and byi[i - k]["cpu_min"] >= 1000]
    if a and b:
        print(f"    k={k:+d}: dip n={len(a):3d} mean {st.mean(a):6.1f} | no-dip n={len(b):3d} mean {st.mean(b):6.1f}")
print("  Pearson r with per-second IMU Hz:")
for k in ("cpu_min", "load1", "tj", "vdd_w", "cpugpu_a", "soc_a"):
    print(f"    {k:9s} {corr([r['hz'] for r in rows], [r[k] for r in rows]):+.2f}")
print(f"  {len(low)} of {len(rows)} seconds had a core at <1000 MHz ({100*len(low)/len(rows):.0f}%)")
