#!/usr/bin/env python3
"""score_baro.py LOG [LOG ...] -- score runs on barometer residual.

Peak altitude is one sample. The residual is a continuous measure of agreement
across the whole flight, so it is the better scorecard.

Two residuals are reported, and the pair matters:

  res_filter = (z_baro - h_ref) - (z_vio + bias)   what the chi2 gate tests
  res_raw    = (z_baro - h_ref) -  z_vio           bias excluded

res_filter looks healthy whenever the bias absorbs the error -- which is exactly
the failure mode where a bias with too much freedom quietly swallows real VIO
altitude error. res_raw cannot be fooled that way. When the two disagree, the
bias is hiding something; when they match, it is not.

Split by phase because the physics differs: on climb the features are close and
parallax is plentiful, at cruise the ground is 100 m away and depth is weakly
observed.
"""
import re
import sys

import numpy as np


def parse(path):
    txt = open(path, errors="ignore").read()
    ZB, ZV, B, R = [], [], [], []
    for m in re.finditer(r"\[BARO\]: t=([\d.]+) z_baro=\s*([-+\d.]+) z_vio=\s*([-+\d.]+) "
                         r"bias=\s*([-+\d.]+) res=\s*([-+\d.]+)", txt):
        _, zb, zv, b, r = (float(x) for x in m.groups())
        ZB.append(zb); ZV.append(zv); B.append(b); R.append(r)
    rej = len(re.findall(r"\[BARO\]: rejected", txt))
    ms, klt = [], 0
    for line in txt.splitlines():
        if "seconds for tracking" in line:
            klt += 1
        mm = re.search(r"MSCKF update \((\d+) feats\)", line)
        if mm:
            ms.append(int(mm.group(1)))
    cfg = {k: (re.search(k + r":\s*([\d.]+)", txt).group(1)
               if re.search(k + r":\s*([\d.]+)", txt) else "?")
           for k in ("baro_sigma", "baro_bias_prior", "baro_bias_rw")}
    return dict(ZB=np.array(ZB), ZV=np.array(ZV), B=np.array(B), R=np.array(R),
                rej=rej, ms=ms, klt=klt, cfg=cfg)


def rms(x):
    return float(np.sqrt((x ** 2).mean())) if len(x) else float("nan")


print(f"{'log':22s} {'sig/prior/rw':>18s} {'MSCKF':>6s} {'rej%':>5s} "
      f"{'raw RMS':>8s} {'raw mean':>9s} {'climb':>7s} {'cruise':>7s} {'desc':>7s} {'bias range':>15s}")
print("-" * 118)
for path in sys.argv[1:]:
    d = parse(path)
    if not len(d["R"]):
        print(f"{path.split('/')[-1][:22]:22s}  no baro lines")
        continue
    ZB, ZV, B, R = d["ZB"], d["ZV"], d["B"], d["R"]
    raw = ZB - ZV
    pk = int(np.argmax(ZB))
    idx = np.arange(len(ZB))
    climb = (idx < pk) & (ZB < 90)
    cruise = ZB >= 90
    desc = (idx > pk) & (ZB < 90)
    nz = sum(1 for x in d["ms"] if x > 0)
    pct = 100 * nz / len(d["ms"]) if d["ms"] else float("nan")
    rp = 100 * d["rej"] / (len(R) + d["rej"])
    c = d["cfg"]
    tag = f"{c['baro_sigma']}/{c['baro_bias_prior']}/{c['baro_bias_rw']}"
    print(f"{path.split('/')[-1][-22:]:22s} {tag:>18s} {pct:5.1f}% {rp:5.1f} "
          f"{rms(raw):8.2f} {raw.mean():+9.2f} {rms(raw[climb]):7.2f} {rms(raw[cruise]):7.2f} "
          f"{rms(raw[desc]):7.2f} {B.min():+6.2f}..{B.max():+6.2f}")
