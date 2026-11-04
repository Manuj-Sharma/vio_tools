#!/usr/bin/env python3
"""One-off anomaly scan of a rosbag2 sqlite3 bag. Read-only (immutable=1)."""
import sys, sqlite3, collections
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
import glob
db = sorted(glob.glob(bag + "/*.db3"))[0]
con = sqlite3.connect(f"file:{db}?immutable=1", uri=True)
topics = {r[0]: (r[1], r[2]) for r in con.execute("select id,name,type from topics")}
tid = {n: i for i, (n) in [(k, v[0]) for k, v in topics.items()]}


def rows(name):
    i = [k for k, v in topics.items() if v[0] == name]
    if not i:
        return []
    return con.execute("select timestamp,data from messages where topic_id=? order by id", (i[0],)).fetchall()


def stats(name, t, hz_exp=None):
    t = np.asarray(t, dtype=np.float64) / 1e9
    if len(t) < 3:
        print(f"{name}: only {len(t)} msgs"); return
    d = np.diff(t)
    med = np.median(d)
    print(f"{name}: n={len(t)} span={t[-1]-t[0]:.2f}s rate={(len(t)-1)/(t[-1]-t[0]):.2f}Hz "
          f"dt med={med*1e3:.2f}ms min={d.min()*1e3:.2f} max={d.max()*1e3:.2f} std={d.std()*1e3:.2f}ms")
    print(f"   non-monotonic={int((d<0).sum())} zero-dt={int((d==0).sum())} "
          f"gaps>2.5*med={int((d>2.5*med).sum())} gaps>5*med={int((d>5*med).sum())}")
    big = np.argsort(d)[::-1][:5]
    print("   biggest gaps (ms @ t+s): " + ", ".join(f"{d[i]*1e3:.0f}@{t[i]-t[0]:.1f}" for i in sorted(big)))
    return t


T0 = con.execute("select min(timestamp) from messages").fetchone()[0]

# ---- IMU
print("=== /imu0")
r = rows("/imu0")
Imsg = [deserialize_message(d, get_message(topics[[k for k,v in topics.items() if v[0]=='/imu0'][0]][1])) for _, d in r]
rec = np.array([x[0] for x in r]) / 1e9
hs = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in Imsg])
stats("/imu0 (bag recv time)", [x[0] for x in r])
stats("/imu0 (header.stamp)", hs * 1e9)
lat = rec - hs
print(f"   recv-minus-header latency: med={np.median(lat)*1e3:.2f}ms min={lat.min()*1e3:.2f} max={lat.max()*1e3:.2f}")
dup = int((np.diff(hs) == 0).sum())
print(f"   duplicate header stamps: {dup}")
a = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in Imsg])
g = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in Imsg])
print(f"   accel mean={a.mean(0).round(3)} std={a.std(0).round(4)} |a| mean={np.linalg.norm(a,axis=1).mean():.3f} min={a.min(0).round(2)} max={a.max(0).round(2)}")
print(f"   gyro  mean={g.mean(0).round(4)} std={g.std(0).round(4)} min={g.min(0).round(3)} max={g.max(0).round(3)}")
nan = int(np.isnan(a).any(axis=1).sum() + np.isnan(g).any(axis=1).sum())
print(f"   NaN rows={nan}; exact repeated (a,g) rows={int(((np.diff(a,axis=0)==0).all(1) & (np.diff(g,axis=0)==0).all(1)).sum())}")
# spikes
for nm, x in (("accel", a), ("gyro", g)):
    s = np.abs(np.diff(x, axis=0)).max(1)
    thr = np.median(s) + 10 * 1.4826 * np.median(np.abs(s - np.median(s)))
    print(f"   {nm} sample-to-sample jumps > med+10MAD ({thr:.3f}): {int((s>thr).sum())}, max jump {s.max():.3f}")

# ---- camera
print("=== /cam0/image_30hz")
cname = "/cam0/image_30hz"
cid = [k for k, v in topics.items() if v[0] == cname][0]
cr = rows(cname)
cm = get_message(topics[cid][1])
cmsgs = [deserialize_message(d, cm) for _, d in cr]
crec = np.array([x[0] for x in cr]) / 1e9
chs = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in cmsgs])
stats("cam (bag recv time)", [x[0] for x in cr])
stats("cam (header.stamp)", chs * 1e9)
clat = crec - chs
print(f"   recv-minus-header latency: med={np.median(clat)*1e3:.2f}ms min={clat.min()*1e3:.2f} max={clat.max()*1e3:.2f} std={clat.std()*1e3:.2f}")
print(f"   duplicate header stamps: {int((np.diff(chs)==0).sum())}")
m0 = cmsgs[0]
print(f"   encoding={m0.encoding} {m0.width}x{m0.height} step={m0.step} frame_id={m0.header.frame_id!r}")
sizes = {len(m.data) for m in cmsgs}
print(f"   distinct data sizes: {sizes} (expect step*height={m0.step*m0.height})")
# content
mean = []; std = []; sat = []; ident = 0; prev = None; diffs = []
for m in cmsgs:
    arr = np.frombuffer(bytes(m.data), dtype=np.uint8)
    mean.append(arr.mean()); std.append(arr.std()); sat.append((arr >= 250).mean())
    if prev is not None:
        dd = np.abs(arr.astype(np.int16) - prev).mean()
        diffs.append(dd)
        if dd == 0: ident += 1
    prev = arr.astype(np.int16)
mean = np.array(mean); std = np.array(std); diffs = np.array(diffs)
print(f"   brightness mean: med={np.median(mean):.1f} min={mean.min():.1f} max={mean.max():.1f}; contrast std: med={np.median(std):.1f} min={std.min():.1f}")
print(f"   saturated(>=250) fraction max={max(sat):.3f}; identical consecutive frames={ident}")
print(f"   mean abs frame-to-frame diff: med={np.median(diffs):.2f} max={diffs.max():.2f} (frames with diff<0.05: {int((diffs<0.05).sum())})")
jump = np.where(np.abs(np.diff(mean)) > 15)[0]
print(f"   brightness jumps >15 levels between frames: {len(jump)} at idx {jump[:8].tolist()}")
low = np.where(std < 0.5 * np.median(std))[0]
print(f"   low-contrast frames (<50% median): {len(low)} at idx {low[:8].tolist()}")
# sample throttle pattern: gaps between frames relative to source ~51Hz
d = np.diff(chs)
print(f"   header dt histogram (ms): " + ", ".join(f"{k}:{v}" for k, v in sorted(collections.Counter(np.round(d*1e3/5)*5).items())))

# ---- cam / imu alignment
print("=== cam vs imu")
print(f"   first cam hdr - first imu hdr = {(chs[0]-hs[0])*1e3:.1f}ms; last = {(chs[-1]-hs[-1])*1e3:.1f}ms")
print(f"   header-clock vs recv-clock: first cam recv = +{crec[0]-T0/1e9:.2f}s, first imu recv = +{rec[0]-T0/1e9:.2f}s into bag")
# imu samples between consecutive camera frames
cnt = np.searchsorted(hs, chs[1:]) - np.searchsorted(hs, chs[:-1])
print(f"   imu samples between consecutive frames: med={np.median(cnt):.0f} min={cnt.min()} max={cnt.max()}")

# ---- every other topic
print("=== other topics (count, rate, max gap)")
for k, (n, ty) in sorted(topics.items(), key=lambda kv: kv[1][0]):
    if n in ("/imu0", cname):
        continue
    t = [x[0] for x in con.execute("select timestamp from messages where topic_id=? order by id", (k,))]
    if len(t) < 2:
        print(f"{n}: n={len(t)}"); continue
    tt = np.array(t) / 1e9; d = np.diff(tt)
    print(f"{n}: n={len(t)} rate={(len(t)-1)/(tt[-1]-tt[0]):.2f}Hz maxgap={d.max()*1e3:.0f}ms first=+{tt[0]-T0/1e9:.2f}s last=+{tt[-1]-T0/1e9:.2f}s")

# ---- FC sanity
print("=== FC sanity")
def load(n):
    i = [k for k, v in topics.items() if v[0] == n]
    if not i: return []
    mt = get_message(topics[i[0]][1])
    return [deserialize_message(d, mt) for (d,) in con.execute("select data from messages where topic_id=? order by id", (i[0],))]
for n in ("/fc/statustext",):
    for m in load(n):
        print(f"   {n}: {m.data if hasattr(m,'data') else m}")
gp = load("/fc/gps1")
if gp:
    lat = np.array([m.latitude for m in gp]); lon = np.array([m.longitude for m in gp]); alt = np.array([m.altitude for m in gp])
    print(f"   gps1 lat/lon wander: {(lat.max()-lat.min())*111e3:.2f} m N-S, {(lon.max()-lon.min())*111e3*np.cos(np.radians(lat.mean())):.2f} m E-W, alt range {alt.max()-alt.min():.2f} m")
lp = load("/fc/local_position_ned")
if lp:
    print("   local_position_ned msg type fields:", [f for f in lp[0].get_fields_and_field_types()][:6])
    p = lp[0]
    try:
        xs = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in lp])
        print(f"   local_position_ned range: {xs.max(0)-xs.min(0)}")
    except Exception as e:
        print("   (skip local pos detail:", e, ")")
att = load("/fc/attitude")
if att:
    q = np.array([[m.x, m.y, m.z, m.w] for m in att])
    print(f"   attitude quaternion norm min/max: {np.linalg.norm(q,axis=1).min():.4f}/{np.linalg.norm(q,axis=1).max():.4f}")
for n in ("/fc/armed", "/fc/ekf_status", "/fc/gps_active_src", "/fc/system_state"):
    v = load(n)
    if v:
        vals = collections.Counter(str(getattr(m, "data", m)) for m in v)
        print(f"   {n}: {dict(list(vals.items())[:6])}")
