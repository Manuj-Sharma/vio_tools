#!/usr/bin/env python3
"""Validate + analyse imu_filter_node output.
  imu_filter_check.py RAW_BAG FILTERED_BAG [--order 4] [--cutoff 100] [--out-rate 300] [--max-fill 20] [--png OUT.png]
RAW_BAG: bag with the 1 kHz /imu0 (original recording).  FILTERED_BAG: bag with /imu0_filtered (replay_filter_bag.sh output).
Needs ROS sourced (rclpy deserialisation); run with `python3` (NOT -I).
 1. C++ node output vs an independent scipy reference (same gap rule: gaps > max-fill ticks restart the filter).
 2. Delay check: transfer-function phase at the strongest tone vs the analytic phase of the Butterworth.
 3. PSD of raw vs filtered per channel -> PNG.
"""
import sys, glob, os, sqlite3, argparse
import numpy as np
from scipy import signal
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

ap = argparse.ArgumentParser()
ap.add_argument("raw_bag"); ap.add_argument("filt_bag")
ap.add_argument("--order", type=int, default=4); ap.add_argument("--cutoff", type=float, default=100.0)
ap.add_argument("--out-rate", type=float, default=300.0); ap.add_argument("--max-fill", type=int, default=20)
ap.add_argument("--png", default=None)
a = ap.parse_args()
FS = 1000.0; NAMES = ["gx", "gy", "gz", "ax", "ay", "az"]

def load(bag, topic):
    t, x = [], []
    for db in sorted(glob.glob(os.path.join(bag, "*.db3"))):
        c = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
        r = c.execute("select id from topics where name=?", (topic,)).fetchone()
        if not r: continue
        for (blob,) in c.execute("select data from messages where topic_id=? order by timestamp", (r[0],)):
            m = deserialize_message(bytes(blob), Imu)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            x.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                      m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    return np.array(t), np.array(x)

tr, xr = load(a.raw_bag, "/imu0"); tf, xf = load(a.filt_bag, "/imu0_filtered")
print(f"raw {len(tr)} msgs ({len(tr)/(tr[-1]-tr[0]):.1f} Hz over {tr[-1]-tr[0]:.1f} s) | filtered {len(tf)} msgs ({len(tf)/(tf[-1]-tf[0]):.1f} Hz)")
print(f"filtered stamp spacing: median {np.median(np.diff(tf))*1e3:.4f} ms, max {np.max(np.diff(tf))*1e3:.1f} ms, backwards {int((np.diff(tf)<=0).sum())}")

sos = signal.butter(a.order, a.cutoff, btype="low", fs=FS, output="sos")
f_d = np.linspace(0.01, 200, 40000); h_d = signal.sosfreqz(sos, worN=f_d, fs=FS)[1]
gd0 = -np.gradient(np.unwrap(np.angle(h_d)), 2 * np.pi * f_d)[0]

# ---- 1. scipy reference, segmented like the node (gap > max_fill ticks => restart) ----------------------------
keep = np.concatenate([[True], np.diff(tr) > 0]); tr, xr = tr[keep], xr[keep]
dt_ticks = np.rint(np.diff(tr) * FS).astype(int)
seg_starts = np.concatenate([[0], np.where(dt_ticks > a.max_fill)[0] + 1, [len(tr)]])
ref_t, ref_y = [], []
for s0, s1 in zip(seg_starts[:-1], seg_starts[1:]):
    ts, xs = tr[s0:s1], xr[s0:s1]
    tk = np.concatenate([[0.0], np.cumsum(np.rint(np.diff(ts) * FS))]); grid = np.arange(tk[0], tk[-1] + 1)   # per-pair rounding, like the node (cumulative rounding drifts with the ~40 ppm clock offset)
    xg = np.stack([np.interp(grid, tk, xs[:, c]) for c in range(6)], 1)
    yg = np.stack([signal.sosfilt(sos, xg[:, c], zi=signal.sosfilt_zi(sos) * xg[0, c])[0] for c in range(6)], 1)
    ref_t.append(np.interp(grid, tk, ts)); ref_y.append(yg)   # tick k -> its ACTUAL stamp (IMX/host clocks differ ~40 ppm; a nominal 1 ms grid would drift ~1 ms in 26 s)
print(f"segments (filter restarts at gaps > {a.max_fill} ms): {len(seg_starts)-1}")
t_in = tf + gd0     # node output stamp = t_filtered_signal - DC group delay  ->  undo
err = np.full_like(xf, np.nan)
for tg, yg in zip(ref_t, ref_y):
    m = (t_in >= tg[0]) & (t_in <= tg[-1])
    for c in range(6): err[m, c] = xf[m, c] - np.interp(t_in[m], tg, yg[:, c])
ok = ~np.isnan(err[:, 0])
print(f"\n1) C++ node vs scipy reference ({ok.sum()} of {len(tf)} outputs inside a reference segment)")
print("   chan   max|err|        rms err      signal rms(AC)   rel rms err")
for c in range(6):
    e = err[ok, c]; sr = np.std(xf[ok, c])
    print(f"   {NAMES[c]}   {np.max(abs(e)):10.3e}   {np.sqrt(np.mean(e**2)):10.3e}   {sr:10.3e}   {np.sqrt(np.mean(e**2))/sr:9.2e}")

print("   per segment (all channels, rel. to AC rms):  segment  start(s)  len(s)  outputs  median|err|/rms  max|err|/rms  at t+(s)")
sig = np.std(xf, axis=0)
for si, (tg_, yg_) in enumerate(zip(ref_t, ref_y)):
    mm = (t_in >= tg_[0]) & (t_in <= tg_[-1])
    if mm.sum() == 0: continue
    r = np.abs(err[mm]) / sig; worst = np.unravel_index(np.argmax(r), r.shape)
    print(f"      #{si}   {tg_[0]-tr[0]:7.2f}  {tg_[-1]-tg_[0]:6.2f}  {mm.sum():6d}   {np.median(r):10.2e}   {r.max():10.2e}   {t_in[mm][worst[0]]-tr[0]:7.3f} ({NAMES[worst[1]]})")

# ---- 2. phase/delay at the strongest tone ---------------------------------------------------------------------
print("\n2) delay check at the strongest tone (cross-spectrum raw->filtered, node stamps un-compensated)")
tg, yg = max(zip(ref_t, ref_y), key=lambda p: len(p[0])); lo, hi = tg[0], tg[-1]
m = (t_in >= lo) & (t_in <= hi)
tt = t_in[m]; fs_o = a.out_rate
print("   chan   tone(Hz)  coherence  measured delay (ms)  analytic delay (ms)")
for c in range(6):
    xin = np.interp(tt, tr, xr[:, c]); yo = xf[m, c]
    fr, Pxy = signal.csd(xin, yo, fs=fs_o, nperseg=2048); _, Pxx = signal.welch(xin, fs=fs_o, nperseg=2048)
    _, coh = signal.coherence(xin, yo, fs=fs_o, nperseg=2048)
    sel = (fr > 20) & (fr < 90); i = np.where(sel)[0][np.argmax(Pxx[sel])]
    meas = -np.angle(Pxy[i] / Pxx[i]) / (2 * np.pi * fr[i]) * -1   # y lags x -> negative phase; delay = -phi/w
    meas = -np.angle(Pxy[i]) / (2 * np.pi * fr[i])
    ana = -np.angle(np.interp(fr[i], f_d, h_d.real) + 1j * np.interp(fr[i], f_d, h_d.imag)) / (2 * np.pi * fr[i])
    print(f"   {NAMES[c]}   {fr[i]:7.2f}   {coh[i]:8.3f}   {meas*1e3:15.3f}      {ana*1e3:12.3f}")

# ---- 3. PSD figure ------------------------------------------------------------------------------------------------
fig, ax = plt.subplots(2, 3, figsize=(16, 8), sharex=True)
tk = np.rint((tr - tr[0]) * FS).astype(int); grid = np.arange(tk[0], tk[-1] + 1)
for c in range(6):
    xg = np.interp(grid, tk, xr[:, c]); fr, P = signal.welch(xg, fs=FS, nperseg=4096)
    fo, Po = signal.welch(xf[:, c], fs=fs_o, nperseg=int(4096 * fs_o / FS))
    A = ax.flat[c]; A.semilogy(fr, P, lw=.8, label="raw 1 kHz (gap-filled)"); A.semilogy(fo, Po, lw=.8, label=f"imu_filter_node out ({fs_o:.0f} Hz)")
    A.axvline(a.cutoff, color="k", ls=":", lw=.5); A.set_title(NAMES[c]); A.grid(alpha=.3, which="both")
ax.flat[0].legend(fontsize=8); fig.suptitle(f"imu_filter_node: Butterworth order {a.order}, fc {a.cutoff:.0f} Hz, out {fs_o:.0f} Hz, DC delay {gd0*1e3:.2f} ms")
fig.tight_layout(); png = a.png or a.filt_bag.rstrip("/") + "_psd.png"; fig.savefig(png, dpi=110); print(f"\nfigure: {png}")
