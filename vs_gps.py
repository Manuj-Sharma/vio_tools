#!/usr/bin/env python3
"""vs_gps.py LOG [LOG ...] -- align a VIO run to the GPS track and report scale.

Ground truth is ~/rosbags/seg01_fc.xlsx, an ArduPilot log already aligned to the
bag clock (header records "bag t_rel 0 == log t 210.769s", CTUN.Alt vs bag baro
correlation 0.9983). Columns gps_e_m / gps_n_m are local ENU metres about
lat0=13.40955940 lon0=77.75808009.

WHY UMEYAMA RATHER THAN COMPARING EXTENTS

The VIO's global frame is gravity-aligned but its yaw is arbitrary -- VIO cannot
observe absolute heading, so the trajectory comes out rotated by an unknown
constant. Comparing raw extents conflates that rotation with real error. Umeyama
solves for the rotation, translation and (optionally) scale that best align the
two tracks, so what is left over is genuine error.

The scale factor s is the number that matters here: a nadir camera over flat
ground cannot separate depth from translation (pixel flow only constrains t/d),
so a scale error is the expected failure and s measures it directly.

  s > 1  : VIO travelled further than truth
  rms    : residual AFTER alignment -- drift and shape error, not scale

VIO poses carry no state timestamp in the log, so they are mapped onto the GPS
clock by normalised time across the flight. Both cover the same interval, so
this is adequate for scale; it would not be for tight temporal analysis.
"""
import re
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore")

XLSX = "/home/vc/rosbags/seg01_fc.xlsx"


def load_gps():
    import pandas as pd
    df = pd.read_excel(XLSX, header=4)
    df = df[pd.to_numeric(df["t_rel"], errors="coerce").notna()]
    for c in ("t_rel", "gps_e_m", "gps_n_m", "alt_baro_m"):
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=["t_rel", "gps_e_m", "gps_n_m"])
    return (df["t_rel"].values, df["gps_e_m"].values,
            df["gps_n_m"].values, df["alt_baro_m"].values)


def load_vio(path):
    P = [[float(x) for x in m.groups()]
         for m in re.finditer(r"p_IinG = ([-\d.]+),([-\d.]+),([-\d.]+)",
                              open(path, errors="ignore").read())]
    return np.array(P)


def umeyama(src, dst, with_scale=True):
    """Least-squares similarity transform mapping src onto dst (Nx2 or Nx3)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    C = D.T @ S / len(src)
    U, sig, Vt = np.linalg.svd(C)
    R = U @ Vt
    if np.linalg.det(R) < 0:                 # reflection guard
        Vt[-1] *= -1; sig[-1] *= -1
        R = U @ Vt
    s = sig.sum() / (S ** 2).sum() * len(src) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def main(paths):
    tg, ge, gn, galt = load_gps()
    print(f"  GPS: {len(tg)} samples, {tg[0]:.1f}..{tg[-1]:.1f} s, "
          f"path {np.hypot(np.diff(ge), np.diff(gn)).sum():.1f} m")
    print()
    print(f"  {'log':24s} {'scale':>7s} {'yaw':>8s} {'rms':>7s} {'max':>7s} "
          f"{'vio path':>9s} {'gps path':>9s}")
    print("  " + "-" * 82)
    G = np.c_[ge, gn]
    gps_path = np.hypot(np.diff(ge), np.diff(gn)).sum()
    for p in paths:
        V = load_vio(p)
        if len(V) < 200:
            print(f"  {p.split('/')[-1][-24:]:24s}  too short ({len(V)})")
            continue
        # map VIO samples onto the GPS clock by normalised time
        tv = np.linspace(tg[0], tg[-1], len(V))
        gi = np.c_[np.interp(tv, tg, ge), np.interp(tv, tg, gn)]
        s, R, t = umeyama(V[:, :2], gi, with_scale=True)
        aligned = (s * (R @ V[:, :2].T).T) + t
        err = np.linalg.norm(aligned - gi, axis=1)
        yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        vio_path = np.hypot(np.diff(V[:, 0]), np.diff(V[:, 1])).sum()
        print(f"  {p.split('/')[-1][-24:]:24s} {s:7.3f} {yaw:7.1f}° "
              f"{err.mean():7.1f} {err.max():7.1f} {vio_path:9.1f} {gps_path:9.1f}")
    print()
    print("  scale 1.000 = VIO distance matches GPS. >1 means the VIO travelled further.")
    print("  yaw is the frame offset Umeyama solved for -- expected and harmless,")
    print("  since VIO cannot observe absolute heading.")
    print("  rms/max are residuals AFTER alignment: drift and shape error only.")


if __name__ == "__main__":
    main(sys.argv[1:] or ["/tmp/vio_logs/openvins_bag_latest.log"])
