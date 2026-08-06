#!/usr/bin/env python3
"""plot_vs_gps.py LOG [LOG ...] -- plot VIO runs against the GPS track.

    python3 plot_vs_gps.py /tmp/vio_logs/openvins_bag_2026*.log
    python3 plot_vs_gps.py -o /tmp/dbest.png log1.log log2.log log3.log

Built for REPEATS. Pass every rep of a config and they overlay on one figure, so
the run-to-run spread is the thing you see rather than something you reconstruct
from a column of scalars. A single scalar per run hides the shape of the failure:
run 201555 read 36.8% MSCKF / 167 m at its midpoint and 29.5% / 376 m by the end,
and only the second number ever reached a table.

WHAT IS PLOTTED

  left    top-down XY. GPS truth in black, each run in colour.
  middle  altitude vs time, with the barometric truth.
  right   horizontal error vs time -- where a run departs, not just by how much.

ALIGNMENT is yaw + translation only, NEVER scale. VIO cannot observe heading, so
the trajectory comes out rotated by an unknown constant that must be removed --
but scale is the error being measured, and fitting it away would draw every run
sitting neatly on the truth. A run that plots visibly larger than the black track
is oversized by exactly that much. The scale that WOULD have been fitted is
printed in the legend (vs_gps.py convention: <1 means the VIO travelled further).

Both frames are gravity-aligned, so the rotation is a pure yaw by construction;
solving a full 3D rotation would let the fit tilt a run to absorb altitude error.

CORRESPONDENCE is by pose index over the run's own span, matching vs_gps.py. That
carries vs_gps.py's known ~2.7% bias (the filter starts ~3.4 s late and its
updates are not uniform), so these plots are directly comparable to vs_gps.py
numbers -- and equally biased. Fixing it properly needs a bag timestamp on the
p_IinG log line.
"""
import argparse
import os
import re
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore")
XLSX = "/home/vc/rosbags/seg01_fc.xlsx"
FULL_POSES = 2985          # poses in a complete seg01 replay; used to detect
                           # runs that were stopped early (see the note below)


def load_gps(path):
    import pandas as pd
    df = pd.read_excel(path, header=4)
    df = df[pd.to_numeric(df["t_rel"], errors="coerce").notna()]
    for c in ("t_rel", "gps_e_m", "gps_n_m", "alt_baro_m"):
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=["t_rel", "gps_e_m", "gps_n_m"])
    t = df["t_rel"].values
    alt = df["alt_baro_m"].values
    alt = alt - alt[t < t[0] + 1.0].mean()
    return t, df["gps_e_m"].values, df["gps_n_m"].values, alt


def load_vio(path):
    P = [[float(x) for x in m.groups()]
         for m in re.finditer(r"p_IinG = ([-\d.]+),([-\d.]+),([-\d.]+)",
                              open(path, errors="ignore").read())]
    return np.array(P)


def yaw_align(src, dst):
    """Yaw + translation mapping src onto dst. Returns (R, t, scale_not_applied)."""
    S2, D2 = src[:, :2], dst[:, :2]
    mu_s, mu_d = S2.mean(0), D2.mean(0)
    S, D = S2 - mu_s, D2 - mu_d
    U, sig, Vt = np.linalg.svd(D.T @ S / len(src))
    R2 = U @ Vt
    if np.linalg.det(R2) < 0:
        Vt[-1] *= -1
        sig[-1] *= -1
        R2 = U @ Vt
    scale = sig.sum() / (S ** 2).sum() * len(src)
    R = np.eye(3)
    R[:2, :2] = R2
    t = np.zeros(3)
    t[:2] = mu_d - R2 @ mu_s
    t[2] = dst[:, 2].mean() - src[:, 2].mean()
    return R, t, scale


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("-o", "--out", default="/tmp/vs_gps.png")
    ap.add_argument("--xlsx", default=XLSX)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tg, ge, gn, galt = load_gps(args.xlsx)
    G = np.c_[ge, gn]

    fig, ax = plt.subplots(1, 3, figsize=(17, 5.4))
    ax[0].plot(ge, gn, "k-", lw=2.4, label="GPS truth", zorder=1)
    ax[0].plot(ge[0], gn[0], "ko", ms=7, zorder=3)
    ax[1].plot(tg, galt, "k-", lw=2.2, label="baro truth")

    colors = plt.cm.tab10(np.linspace(0, 1, 10))
    scales = []
    for i, p in enumerate(args.logs):
        V = load_vio(p)
        if len(V) < 200:
            print(f"  skip {os.path.basename(p)}: only {len(V)} poses")
            continue
        # A run that was stopped early covers only the START of the flight.
        # Stretching its poses across the WHOLE GPS span compares its first third
        # against the entire track and returns a meaningless scale (run 190500:
        # 1000 poses -> "scale 0.172"). Estimate coverage from the pose count
        # against a complete run and map over that sub-span instead.
        frac = min(1.0, len(V) / FULL_POSES)
        t_end = tg[0] + frac * (tg[-1] - tg[0])
        if frac < 0.95:
            print(f"  NOTE {os.path.basename(p)}: {len(V)} poses = {100*frac:.0f}% of a "
                  f"full run -- scored against the first {100*frac:.0f}% of the flight only")
        tv = np.linspace(tg[0], t_end, len(V))
        gi = np.c_[np.interp(tv, tg, ge), np.interp(tv, tg, gn),
                   np.interp(tv, tg, galt)]
        R, t, s = yaw_align(V, gi)
        A = (R @ V.T).T + t                      # yaw+translation only, NOT scaled
        err = np.linalg.norm(A[:, :2] - gi[:, :2], axis=1)
        scales.append(s)
        c = colors[i % 10]
        name = os.path.basename(p).replace("openvins_bag_", "").replace(".log", "")
        lab = f"{name}  s={s:.3f} rms={err.mean():.1f}m"
        ax[0].plot(A[:, 0], A[:, 1], "-", color=c, lw=1.0, alpha=0.85, label=lab)
        ax[1].plot(tv, A[:, 2], "-", color=c, lw=1.0, alpha=0.85)
        ax[2].plot(tv, err, "-", color=c, lw=1.0, alpha=0.85, label=name)
        print(f"  {name}  scale {s:.3f}  rms {err.mean():6.1f} m  max {err.max():6.1f} m  "
              f"peak z {V[:,2].max():6.1f} m")

    ax[0].set_title("top-down  (yaw-aligned, NOT scaled)")
    ax[0].set_xlabel("east [m]"); ax[0].set_ylabel("north [m]")
    ax[0].axis("equal"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=7, loc="best")
    ax[1].set_title("altitude"); ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("z [m]")
    ax[1].grid(alpha=.3); ax[1].legend(fontsize=7)
    ax[2].set_title("horizontal error vs truth")
    ax[2].set_xlabel("t [s]"); ax[2].set_ylabel("error [m]")
    ax[2].grid(alpha=.3); ax[2].legend(fontsize=7)

    if len(scales) > 1:
        sp = f"scale spread {min(scales):.3f}-{max(scales):.3f} over {len(scales)} runs"
        fig.suptitle(sp, fontsize=10)
        print(f"\n  {sp}")
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"\n  wrote {args.out}")


if __name__ == "__main__":
    main()
