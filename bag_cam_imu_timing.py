#!/usr/bin/env python3
"""IMU-vs-camera stamp timing in a rosbag2 sqlite3 bag, straight from the CDR header (no ROS needed).
  bag_cam_imu_timing.py BAG [--imu /imu0] [--cam /cam0/image_raw]

lat(t) = recorder receive time - header stamp. IMU and camera headers are CLOCK_MONOTONIC, the recorder time is wall clock, so the
absolute value includes a constant epoch offset; what matters is (a) the DIFFERENCE cam - imu, and (b) how each lat drifts or jumps
over the bag. Per 1 s window it takes the MINIMUM lat (the least-delayed message = the stamp's true offset + minimal transport delay).
  D = min_lat(cam) - min_lat(imu)   is the camera/IMU stamp relationship as the recorder sees it (what Kalibr's timeshift absorbs).
"""
import sys, glob, os, sqlite3, struct
import numpy as np

args = [a for a in sys.argv[1:] if not a.startswith("--")]
bag = args[0]
imu_t = sys.argv[sys.argv.index("--imu") + 1] if "--imu" in sys.argv else "/imu0"
cam_t = sys.argv[sys.argv.index("--cam") + 1] if "--cam" in sys.argv else "/cam0/image_raw"
for t in (imu_t, cam_t):
    if t in args: args.remove(t)

def load(topic):
    rec, stamp = [], []
    for db in sorted(glob.glob(os.path.join(bag, "*.db3"))):
        c = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
        r = c.execute("select id from topics where name=?", (topic,)).fetchone()
        if not r: continue
        for ts, blob in c.execute("select timestamp, data from messages where topic_id=? order by timestamp", (r[0],)):
            sec, nsec = struct.unpack_from("<iI", blob, 4)
            rec.append(ts * 1e-9); stamp.append(sec + nsec * 1e-9)
    return np.array(rec), np.array(stamp)

ri, si = load(imu_t); rc, sc = load(cam_t)
print(f"{bag}\n  {imu_t}: {len(ri)} msgs  {len(ri)/(ri[-1]-ri[0]):.1f} Hz | {cam_t}: {len(rc)} msgs  {len(rc)/(rc[-1]-rc[0]):.1f} Hz")
t0 = max(ri[0], rc[0]); t1 = min(ri[-1], rc[-1])
li, lc = ri - si, rc - sc

def windows(rec, lat, w=1.0):
    edges = np.arange(t0, t1, w); mn, md, tt = [], [], []
    for a in edges:
        m = (rec >= a) & (rec < a + w)
        if m.sum() > 3: mn.append(lat[m].min()); md.append(np.median(lat[m])); tt.append(a - t0 + w / 2)
    return np.array(tt), np.array(mn), np.array(md)

ti, mni, mdi = windows(ri, li); tc, mnc, mdc = windows(rc, lc)
n = min(len(ti), len(tc)); D = mnc[:n] - mni[:n]
print("\n  per-second MIN of (receive - header stamp), relative to the first second (ms):")
print("   t(s)    imu_min   imu_median   cam_min   cam_median   D=cam_min-imu_min")
for k in range(0, n, max(1, n // 12)):
    print(f"  {ti[k]:5.1f}  {1e3*(mni[k]-mni[0]):8.2f}   {1e3*(mdi[k]-mni[0]):9.2f}   {1e3*(mnc[k]-mnc[0]):7.2f}   {1e3*(mdc[k]-mnc[0]):9.2f}   {1e3*(D[k]-D[0]):10.2f}")
pi = np.polyfit(ti, mni, 1)[0]; pc = np.polyfit(tc, mnc, 1)[0]; pd = np.polyfit(ti[:n], D, 1)[0]
print(f"\n  drift of the min-latency line (us per second):  imu {1e6*pi:+.1f}   cam {1e6*pc:+.1f}   D {1e6*pd:+.1f}")
print(f"  over this bag ({ti[-1]-ti[0]:.0f} s): D changes by {1e3*pd*(ti[-1]-ti[0]):+.2f} ms;  D std about its trend {1e3*np.std(D-np.polyval(np.polyfit(ti[:n],D,1),ti[:n])):.2f} ms")
print(f"  IMU latency spread inside a window (median-min) typical {1e3*np.median(mdi-mni):.2f} ms, 95th pct {1e3*np.percentile(mdi-mni,95):.2f} ms")
print(f"  cam latency spread inside a window (median-min) typical {1e3*np.median(mdc-mnc):.2f} ms, 95th pct {1e3*np.percentile(mdc-mnc,95):.2f} ms")
print(f"  absolute D (cam_min - imu_min, includes nothing else because both use the same wall-vs-monotonic epoch): median {1e3*np.median(D):.2f} ms")
