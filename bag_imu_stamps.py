#!/usr/bin/env python3
"""Header-stamp spacing of /imu0 in a rosbag2 sqlite3 bag, read straight from the CDR bytes (no ROS needed).
   bag_imu_stamps.py BAG_DIR_or_db3 [--topic /imu0]
Prints count, rate, duplicate/backwards stamps and a histogram of dt in whole ms (1 ms = no loss at 1 kHz)."""
import sys, struct, sqlite3, glob, os, collections
import numpy as np
a = [x for x in sys.argv[1:] if not x.startswith("--")]
topic = sys.argv[sys.argv.index("--topic") + 1] if "--topic" in sys.argv else "/imu0"
if topic in a: a.remove(topic)
p = a[0]
dbs = sorted(glob.glob(os.path.join(p, "*.db3"))) if os.path.isdir(p) else [p]
t = []
for db in dbs:
    c = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
    tid = c.execute("select id from topics where name=?", (topic,)).fetchone()
    if not tid: continue
    for (blob,) in c.execute("select data from messages where topic_id=? order by timestamp", (tid[0],)):
        sec, nsec = struct.unpack_from("<iI", blob, 4)
        t.append(sec + nsec * 1e-9)
t = np.array(t); d = np.diff(t)
print(f"{topic}: {len(t)} msgs over {t[-1]-t[0]:.2f} s = {len(t)/(t[-1]-t[0]):.1f} Hz; duplicates {int((d==0).sum())}, backwards {int((d<0).sum())}")
k = np.rint(d * 1000).astype(int)
h = collections.Counter(np.clip(k, -1, 20))
print("dt histogram (ms):", "  ".join(f"{m}ms:{n}" for m, n in sorted(h.items())), "  (20 = >=20 ms)")
big = np.where(d > 0.02)[0]
for i in big: print(f"  gap {d[i]*1e3:.0f} ms at t+{t[i]-t[0]:.2f} s")
ticks = int(round((t[-1] - t[0]) * 1000)); gapticks = int(sum(round(x * 1000) - 1 for x in d[d > 0.02]))
print(f"1 kHz ticks in span {ticks}; missed {ticks-len(t)+1} ({100*(ticks-len(t)+1)/ticks:.1f}%); of which in gaps >20 ms: {gapticks}; elsewhere: {ticks-len(t)+1-gapticks} ({100*(ticks-len(t)+1-gapticks)/(ticks-gapticks):.1f}% of non-gap ticks)")
