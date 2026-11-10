#!/usr/bin/env python3
"""Offline sweep of causal Butterworth low-pass settings over a raw 1 kHz /imu0 bag recorded while lifting the rig.

  imu_filter_sweep.py BAG [--orders 1 2 3 4] [--cutoffs 20 30 40 50 75 100 125 150] [--out-rate 300]
                          [--quiet T0 T1] [--event T0 T1] [--lifts THRESH] [--png OUT.png] [--csv OUT.csv]
BAG: ros2 bag with /imu0 (1 kHz stream). Needs ROS sourced (rclpy deserialisation); run with `python3` (NOT -I).
Times T0/T1 are seconds from the first sample. Without --quiet/--event the lift is auto-detected from |accel|.

For every (order, cutoff) it reports (all causal, same gap-fill as imu_filter_node):
  gd_ms      DC group delay of the filter (what the node does NOT compensate in the data, only in the stamp)
  rise_ms    10-90% rise time of the analytic step response  (how fast a real step shows up)
  over_%     analytic step overshoot (ringing)
  q_<ch>     rms of the filtered signal in the quiet window  (noise left after filtering; compare raw row)
  alias_%    share of the filtered quiet-window power above out_rate/2  = what folds into band when resampling
  pk_ratio   peak of |accel - quiet mean| in the event, filtered/raw       (1.0 = transient fully kept)
  ev_err_%   rms(filtered shifted by gd - raw) in the event / rms(raw event deviation): distortion of the transient
             (raw includes its own noise, so a small floor of ev_err is the removed noise, not damage)
"""
import sys, glob, os, sqlite3, argparse
import numpy as np
from scipy import signal
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

ap = argparse.ArgumentParser()
ap.add_argument("bag"); ap.add_argument("--topic", default="/imu0")
ap.add_argument("--orders", type=int, nargs="+", default=[1, 2, 3, 4])
ap.add_argument("--cutoffs", type=float, nargs="+", default=[20, 30, 40, 50, 75, 100, 125, 150])
ap.add_argument("--out-rate", type=float, default=300.0)
ap.add_argument("--max-fill", type=int, default=20)
ap.add_argument("--quiet", type=float, nargs=2); ap.add_argument("--event", type=float, nargs=2)
ap.add_argument("--lifts", type=float, metavar="THRESH", help="event = only the moments where |accel - quiet| > THRESH m/s^2 (+-0.25 s), e.g. 3; for bags with many lifts")
ap.add_argument("--png"); ap.add_argument("--csv")
a = ap.parse_args()
FS = 1000.0; NAMES = ["gx", "gy", "gz", "ax", "ay", "az"]

# ---- load ------------------------------------------------------------------------------------------------------
t, x = [], []
for db in sorted(glob.glob(os.path.join(a.bag, "*.db3"))):
    c = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)     # live-db read needs immutable=1
    r = c.execute("select id from topics where name=?", (a.topic,)).fetchone()
    if not r: continue
    for (blob,) in c.execute("select data from messages where topic_id=? order by timestamp", (r[0],)):
        m = deserialize_message(bytes(blob), Imu)
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        x.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                  m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
t = np.array(t); x = np.array(x)
if len(t) < 2000: sys.exit(f"only {len(t)} samples of {a.topic} in {a.bag}")
keep = np.concatenate([[True], np.diff(t) > 0]); t, x = t[keep], x[keep]
t0 = t[0]; t = t - t0
print(f"{len(t)} samples, {t[-1]:.1f} s, mean rate {len(t)/t[-1]:.1f} Hz")

# ---- gap-fill onto the 1 ms tick grid (longest segment only: a gap > max_fill restarts the node's filter) -----------
dtk = np.rint(np.diff(t) * FS).astype(int)
bounds = np.concatenate([[0], np.where(dtk > a.max_fill)[0] + 1, [len(t)]])
s0, s1 = max(zip(bounds[:-1], bounds[1:]), key=lambda p: p[1] - p[0])
if len(bounds) > 2: print(f"{len(bounds)-1} segments (gaps > {a.max_fill} ms); using the longest: {t[s0]:.2f}..{t[s1-1]:.2f} s")
ts, xs = t[s0:s1], x[s0:s1]
tk = np.concatenate([[0.0], np.cumsum(np.rint(np.diff(ts) * FS))]); grid = np.arange(tk[0], tk[-1] + 1)
X = np.stack([np.interp(grid, tk, xs[:, c]) for c in range(6)], 1)
miss = 1 - len(ts) / len(grid); tg = ts[0] + grid / FS
print(f"grid {len(grid)} ticks, {100*miss:.2f} % filled")

# ---- quiet window + event window ---------------------------------------------------------------------------------------
am = np.linalg.norm(X[:, 3:], axis=1)
sm = np.convolve(am, np.ones(25) / 25, mode="same")
if a.quiet: q0, q1 = a.quiet
else:
    q1 = min(3.0, tg[-1] - tg[0]); q0 = 0.2
qi = (tg - tg[0] >= q0) & (tg - tg[0] <= q1)
base, sd = np.median(sm[qi]), np.std(sm[qi])
if a.event: e0, e1 = a.event
else:
    hit = np.where((np.abs(sm - base) > max(8 * sd, 0.3)) & (tg - tg[0] > q1))[0]
    if len(hit) == 0: sys.exit("no event found (|accel| never leaves the quiet level by >0.3 m/s^2): pass --event T0 T1")
    e0, e1 = tg[hit[0]] - tg[0] - 0.3, tg[hit[-1]] - tg[0] + 0.3
ei = (tg - tg[0] >= e0) & (tg - tg[0] <= e1)
if a.lifts:
    big = (np.abs(am - base) > a.lifts) & (tg - tg[0] > q1)
    ei = np.convolve(big.astype(float), np.ones(501), mode="same") > 0
    print(f"--lifts {a.lifts}: {int(ei.sum())} ticks ({ei.sum()/FS:.1f} s) in {int((np.diff(ei.astype(int)) == 1).sum())} lift windows")
print(f"quiet window {q0:.2f}..{q1:.2f} s ({qi.sum()} ticks), event window {e0:.2f}..{e1:.2f} s ({ei.sum()} ticks)")
qmean = X[qi].mean(0)
dev_raw = X[ei] - qmean                    # event deviation from the pre-lift mean (gravity removed, per channel)
pk_raw = np.max(np.linalg.norm(dev_raw[:, 3:], axis=1)); rms_raw = np.sqrt(np.mean(dev_raw ** 2, axis=0))
print(f"raw: quiet rms  " + "  ".join(f"{n} {np.std(X[qi][:, c]):.4f}" for c, n in enumerate(NAMES)))
print(f"raw: event peak |a - a_quiet| {pk_raw:.2f} m/s^2; event rms  " + "  ".join(f"{n} {rms_raw[c]:.3f}" for c, n in enumerate(NAMES)))

# ---- sweep -------------------------------------------------------------------------------------------------------------
rows = []; filt = {}
f_d = np.linspace(0.01, 300, 30000)
hdr = f"{'ord':>3} {'fc':>5} {'gd_ms':>6} {'rise_ms':>7} {'over_%':>6} {'q_gyro':>9} {'q_acc':>9} {'alias_%':>7} {'pk_ratio':>8} {'ev_err_%':>8}"
print("\n" + hdr)
for o in a.orders:
    for fc in a.cutoffs:
        if fc >= FS / 2: continue
        sos = signal.butter(o, fc, btype="low", fs=FS, output="sos")
        h = signal.sosfreqz(sos, worN=f_d, fs=FS)[1]
        gd = -np.gradient(np.unwrap(np.angle(h)), 2 * np.pi * f_d)[0]
        n = int(0.5 * FS); st = signal.sosfilt(sos, np.ones(n))
        i10, i90 = np.argmax(st >= .1), np.argmax(st >= .9)
        rise = (i90 - i10) / FS * 1e3; over = (st.max() - 1) * 100
        Y = np.stack([signal.sosfilt(sos, X[:, c], zi=signal.sosfilt_zi(sos) * X[0, c])[0] for c in range(6)], 1)
        filt[(o, fc)] = Y
        qg = np.sqrt(np.mean(np.var(Y[qi][:, :3], axis=0))); qa = np.sqrt(np.mean(np.var(Y[qi][:, 3:], axis=0)))
        fr, P = signal.welch(Y[qi][:, 3:] - Y[qi][:, 3:].mean(0), fs=FS, nperseg=min(512, int(qi.sum())), axis=0)
        P = P.sum(1); alias = 100 * P[fr > a.out_rate / 2].sum() / P.sum()
        sh = int(round(gd * FS)); idx = np.where(ei)[0]
        Yev = Y[np.clip(idx + sh, 0, len(Y) - 1)] - qmean       # undo the group delay so only shape distortion remains
        pk = np.max(np.linalg.norm((Y[np.clip(idx + sh, 0, len(Y) - 1)] - qmean)[:, 3:], axis=1)) / pk_raw
        err = 100 * np.sqrt(np.mean((Yev - dev_raw) ** 2)) / np.sqrt(np.mean(dev_raw ** 2))
        rows.append((o, fc, gd * 1e3, rise, over, qg, qa, alias, pk, err))
        print(f"{o:>3} {fc:>5.0f} {gd*1e3:>6.2f} {rise:>7.2f} {over:>6.2f} {qg:>9.5f} {qa:>9.5f} {alias:>7.3f} {pk:>8.3f} {err:>8.2f}")
print(f"{'raw':>3} {'-':>5} {'0':>6} {'-':>7} {'-':>6} {np.sqrt(np.mean(np.var(X[qi][:, :3], axis=0))):>9.5f} {np.sqrt(np.mean(np.var(X[qi][:, 3:], axis=0))):>9.5f}")
print("units: q_gyro rad/s, q_acc m/s^2 (rms over channels in the quiet window)")

if a.csv:
    np.savetxt(a.csv, np.array(rows), delimiter=",", header="order,cutoff_hz,gd_ms,rise_ms,overshoot_pct,q_gyro,q_acc,alias_pct,pk_ratio,ev_err_pct", comments="")
    print("csv:", a.csv)

# ---- figure: event window, raw vs a few settings, + PSD of the quiet window ----------------------------------------------
pick = [k for k in [(2, 50), (2, 100), (4, 50), (4, 100)] if k in filt] or list(filt)[:4]
fig, ax = plt.subplots(2, 2, figsize=(15, 8))
ei_p = ei if not a.lifts else (tg - tg[0] > q1)
tt = tg[ei_p] - tg[0]; ch = 5   # az: lift is mostly vertical
ax[0, 0].plot(tt, X[ei_p][:, ch], "k", lw=.5, label="raw"); ax[0, 1].plot(tt, X[ei_p][:, 0], "k", lw=.5, label="raw")
for k in pick:
    ax[0, 0].plot(tt, filt[k][ei_p][:, ch], lw=1, label=f"order {k[0]}, {k[1]:.0f} Hz"); ax[0, 1].plot(tt, filt[k][ei_p][:, 0], lw=1, label=f"order {k[0]}, {k[1]:.0f} Hz")
ax[0, 0].set_title("az, event window (causal output, delay NOT removed)"); ax[0, 1].set_title("gx, event window")
ax[0, 0].legend(fontsize=8)
for c, A in ((5, ax[1, 0]), (3, ax[1, 1])):
    fr, P = signal.welch(X[qi][:, c] - X[qi][:, c].mean(), fs=FS, nperseg=min(512, int(qi.sum()))); A.semilogy(fr, P, "k", lw=.8, label="raw")
    for k in pick:
        fr, P = signal.welch(filt[k][qi][:, c] - filt[k][qi][:, c].mean(), fs=FS, nperseg=min(512, int(qi.sum()))); A.semilogy(fr, P, lw=.8, label=f"order {k[0]}, {k[1]:.0f} Hz")
    A.axvline(a.out_rate / 2, color="r", ls=":", lw=.8); A.set_title(f"{NAMES[c]} quiet-window PSD (red = {a.out_rate/2:.0f} Hz, output Nyquist)"); A.set_xlabel("Hz"); A.grid(alpha=.3, which="both")
for A in ax[0]: A.set_xlabel("s"); A.grid(alpha=.3)
fig.tight_layout(); png = a.png or a.bag.rstrip("/") + "_sweep.png"; fig.savefig(png, dpi=110); print("figure:", png)
