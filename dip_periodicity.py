#!/usr/bin/env python3
"""Periodicity of the /imu0 rate dips in a rate_watch.py CSV.

  dip_periodicity.py CSV [--from HH:MM:SS] [--to HH:MM:SS] [--thresh 960] [--frame-us 102]
Finds dips (consecutive 5 s windows below --thresh Hz), prints start / length / depth, the intervals between starts and a linear
fit of how the interval changes, a Lomb-Scargle periodogram peak of the loss series (handles gaps), and a prediction for the next dip.
--frame-us is the SPI transfer time of one read (128 B: 342 us at 3 MHz, 102 us at 10 MHz); the "beat" check compares
dip length / period with frame-us / 1000 us (a read that overlaps the IMX writing its packet is lost, so the share of the beat
cycle spent in the bad phase should be about frame time / poll period).
"""
import csv, argparse
import numpy as np
from scipy.signal import lombscargle
ap = argparse.ArgumentParser(); ap.add_argument("csv"); ap.add_argument("--from", dest="t0", default="00:00:00"); ap.add_argument("--to", dest="t1", default="23:59:59")
ap.add_argument("--thresh", type=float, default=960.0); ap.add_argument("--frame-us", type=float, default=102.0); a = ap.parse_args()
sec = lambda t: sum(int(x) * m for x, m in zip(t.split(":"), (3600, 60, 1)))
hms = lambda s: f"{int(s)//3600:02d}:{(int(s)%3600)//60:02d}:{int(s)%60:02d}"
rows = [(sec(x["time"]), float(x["imu_hz"])) for x in csv.DictReader(open(a.csv)) if a.t0 <= x["time"] <= a.t1]
# drop watcher start-up windows (first window after each restart: a gap > 8 s before it)
clean = [r for i, r in enumerate(rows) if not (i == 0 or rows[i][0] - rows[i-1][0] > 8)]
t = np.array([r[0] for r in clean], float); h = np.array([r[1] for r in clean]); loss = 1000.0 - h
print(f"{len(clean)} windows {hms(t[0])}..{hms(t[-1])}, IMU mean {h.mean():.1f} Hz ({100*loss.mean()/1000:.2f}% missed)")
eps = []; cur = None
for ti, hi in zip(t, h):
    if hi < a.thresh:
        if cur and ti - cur[1] <= 10: cur[1] = ti; cur[2].append(hi)
        else:
            if cur: eps.append(cur)
            cur = [ti, ti, [hi]]
    elif cur: eps.append(cur); cur = None
if cur: eps.append(cur)
eps = [e for e in eps if len(e[2]) >= 2]      # a real dip is >= 2 windows, single-window sags are noise
st = np.array([e[0] - 5 for e in eps]); ln = np.array([(e[1] - e[0] + 5) for e in eps]); mn = np.array([min(e[2]) for e in eps])
print(f"\n{len(eps)} dips (>=2 windows below {a.thresh:g} Hz):   start      length  min Hz   interval since previous")
for i in range(len(eps)): print(f"                              {hms(st[i])}   {ln[i]:5.0f}s  {mn[i]:6.1f}   {('%.0f s' % (st[i]-st[i-1])) if i else '-'}")
if len(eps) >= 3:
    iv = np.diff(st); k = np.arange(len(iv)); sl, ic = np.polyfit(k, iv, 1)
    print(f"\nintervals: {', '.join('%.0f' % x for x in iv)} s | mean {iv.mean():.0f} s | linear fit: {ic:.0f} s + {sl:.1f} s per cycle")
    nxt = st[-1] + ic + sl * len(iv); print(f"next dip predicted at {hms(nxt)} (+-{max(15, iv.std()):.0f} s)")
    P = iv.mean(); L = ln.mean(); print(f"beat check: mean dip length {L:.0f} s / mean period {P:.0f} s = {L/P:.3f}  vs frame time / 1 ms = {a.frame_us/1000:.3f}")
    print(f"         implied host/IMX clock difference = 1 ms / period = {1000.0/P:.1f} ppm")
if len(t) > 40:
    periods = np.arange(60, 400, 1.0); w = 2 * np.pi / periods
    pg = lombscargle(t - t[0], loss - loss.mean(), w, normalize=True); top = np.argsort(pg)[::-1][:1]
    print(f"\nLomb-Scargle peak of the loss series: period {periods[top[0]]:.0f} s (power {pg[top[0]]:.2f}; 1.0 = pure sinusoid)")
