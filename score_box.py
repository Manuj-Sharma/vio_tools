#!/usr/bin/env python3
"""score_box.py LOG [LOG ...] -- score runs against the KNOWN flight box.

The flight is a 267 x 131 m box at ~107 m. That is the only horizontal ground
truth we have (there is no GPS in this dataset), and it turns out to be a much
sharper discriminator than MSCKF percentage:

    16:31 rep2   273.1 x 131.2   +2.3%  +0.1%   <- essentially exact
    15:00        361.0 x 152.5  +35.2% +16.4%
    16:21        359.2 x 150.7  +34.5% +15.0%
    17:33        363.4 x 154.2  +36.1% +17.7%
    16:26 rep1    96.3 x   8.2  -63.9% -93.7%   <- collapsed, MSCKF 18.8%

Note the failure is BIMODAL, not scattered: runs land either near +2% or near
+35%. That is a discrete failure mode, not noise, and it is what makes the
number worth tracking.

Dimensions come from PCA on the cruise-phase horizontal track, so the aircraft's
heading does not matter -- only the shape it flew.

Watch the aspect ratio too. True is 2.04. A pure depth/scale error inflates both
axes equally and leaves aspect alone; the inflated runs show 2.36-2.38, so
something is ALSO distorting the shape (heading drift is the likely candidate).
"""
import re
import sys

import numpy as np

TRUE_L, TRUE_W, TRUE_Z = 267.0, 131.0, 107.75


def main(paths):
    print(f"  truth: {TRUE_L:.0f} x {TRUE_W:.0f} m   aspect {TRUE_L/TRUE_W:.2f}   peak z {TRUE_Z:.2f} m")
    print()
    print(f"  {'log':24s} {'MSCKF':>6s} {'box (PCA)':>16s} {'errL':>7s} {'errW':>7s} "
          f"{'aspect':>7s} {'peak z':>7s} {'errZ':>7s} {'baro':>5s}")
    print("  " + "-" * 100)
    for p in paths:
        try:
            txt = open(p, errors="ignore").read()
        except OSError:
            print(f"  {p[-24:]:24s}  unreadable")
            continue
        P = [[float(x) for x in m.groups()]
             for m in re.finditer(r"p_IinG = ([-\d.]+),([-\d.]+),([-\d.]+)", txt)]
        if len(P) < 200:
            print(f"  {p.split('/')[-1][-24:]:24s}  too short ({len(P)} poses) -- stopped early?")
            continue
        P = np.array(P)
        z = P[:, 2]
        # cruise only: the box is flown at altitude, climb/descent add nothing
        hi = z > 0.8 * z.max()
        if hi.sum() < 50:
            hi = np.ones(len(z), bool)
        c = P[hi, :2] - P[hi, :2].mean(0)
        _, _, vt = np.linalg.svd(c, full_matrices=False)
        pr = c @ vt.T
        L = pr[:, 0].max() - pr[:, 0].min()
        W = pr[:, 1].max() - pr[:, 1].min()
        ms = [int(x) for x in re.findall(r"MSCKF update \((\d+) feats\)", txt)]
        pct = 100 * sum(1 for x in ms if x > 0) / len(ms) if ms else float("nan")
        ub = re.search(r"use_baro: (\d)", txt)
        print(f"  {p.split('/')[-1][-24:]:24s} {pct:5.1f}% {L:7.1f} x {W:6.1f} "
              f"{100*(L/TRUE_L-1):+6.1f}% {100*(W/TRUE_W-1):+6.1f}% {L/W:7.2f} "
              f"{z.max():7.1f} {100*(z.max()/TRUE_Z-1):+6.1f}% {(ub.group(1) if ub else '?'):>5s}")
    print()
    print("  errL/errW near 0 is the goal. +35%/+16% is the known bad mode.")
    print("  aspect 2.04 = correct shape; 2.36+ means the box is also distorted.")


if __name__ == "__main__":
    main(sys.argv[1:] or ["/tmp/vio_logs/openvins_bag_latest.log"])
