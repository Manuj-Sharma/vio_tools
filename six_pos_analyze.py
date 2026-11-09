#!/usr/bin/env python3
"""six_pos_analyze.py -- six-position static accelerometer/gyro check from imx5_read_pimu --raw-csv files.

  six_pos_analyze.py --label FILE            one pose: print LABEL=+Z STILL=1 FRAC=0.998 ... (used by six_pos_capture.sh)
  six_pos_analyze.py DIR [--g 9.78]          all pose_*.csv in DIR: need all six faces (+X -X +Y -Y +Z -Z up)

KNOWN LIMITATION (2026-11-09): the face-based bias/scale numbers assume each captured pose is axis-aligned (|a_axis| / |a| >= 0.95). For tilted
poses they are WRONG (a 15 degree tilt gave a bogus -0.19 m/s^2 bias). With tilted/arbitrary static poses use the magnitude fit instead:
minimise | |diag(s)(a - b)| - g | over all still poses (scipy least_squares; mean(s)=1, g free) -- it gave bias -0.008/+0.036/-0.017 m/s^2 and scale
within 0.05% on the 10 poses of six_pos_20261109_184814. Not yet built into this script.
CSV columns (imx5_read_pimu --raw-csv): time,gx,gy,gz,ax,ay,az  (rad/s, m/s^2).
Method: mean of the six pose means = accel bias vector (gravity cancels); per axis S_i = (a[+i]_i - a[-i]_i)/2 is the
sensitivity; relative scale mismatch between axes is independent of local g; cross-axis coupling = off-axis components of the
normalised response (includes any tilt of the surface and any non-orthogonality: cannot be separated with a table alone).
Gyro: per-axis mean over poses = gyro bias; spread across poses = g-sensitivity/placement/drift; mean std = noise."""
import sys, os, glob, argparse
import numpy as np

AX = "XYZ"
G_REF = 9.80665


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    t = np.atleast_1d(d["time"])
    g = np.column_stack([np.atleast_1d(d[c]) for c in ("gx", "gy", "gz")])
    a = np.column_stack([np.atleast_1d(d[c]) for c in ("ax", "ay", "az")])
    return t, g, a


def pose(path, cut=1.0):
    t, g, a = load(path)
    m = (t > t[0] + cut) & (t < t[-1] - cut)
    t, g, a = t[m], g[m], a[m]
    if len(t) < 200:
        raise ValueError(f"{path}: only {len(t)} samples after trimming")
    ma = a.mean(0)
    k = int(np.argmax(np.abs(ma)))
    h = len(t) // 2
    drift = float(np.linalg.norm(a[:h].mean(0) - a[h:].mean(0)))
    return dict(path=path, k=k, label=("+" if ma[k] > 0 else "-") + AX[k], ma=ma, sa=a.std(0), mg=g.mean(0), sg=g.std(0),
                amag=float(np.linalg.norm(ma)), frac=float(abs(ma[k]) / np.linalg.norm(ma)), n=len(t),
                dur=float(t[-1] - t[0]), drift=drift, gstd_max=float(g.std(0).max()))


def still_ok(p):
    return p["gstd_max"] <= 0.02 and p["drift"] <= 0.05 and p["frac"] >= 0.95


def gyro_section(plist, note=""):
    gb = np.mean([p["mg"] for p in plist], axis=0)
    gs = np.std([p["mg"] for p in plist], axis=0) if len(plist) > 1 else np.full(3, np.nan)
    gn = np.mean([p["sg"] for p in plist], axis=0)
    print("\n== GYRO" + note)
    print(f"bias (rad/s):      X {gb[0]:+.5f}  Y {gb[1]:+.5f}  Z {gb[2]:+.5f}    (deg/s: {np.degrees(gb[0]):+.3f} {np.degrees(gb[1]):+.3f} {np.degrees(gb[2]):+.3f})")
    print(f"spread of bias across poses (rad/s): X {gs[0]:.5f}  Y {gs[1]:.5f}  Z {gs[2]:.5f}   (large = g-sensitivity / warm-up drift / vibration)")
    print(f"noise std (rad/s): X {gn[0]:.5f}  Y {gn[1]:.5f}  Z {gn[2]:.5f}   (BIT reported sigma 0.0035 at rest)")


def partial(poses, faces, miss, gloc):
    print("\nMISSING faces: " + " ".join(miss) + "  -> PARTIAL report (only what the captured faces can show)")
    print("\n== ACCELEROMETER (partial)")
    for lab, p in sorted(poses.items()):
        k = AX.index(lab[1]); oth = [j for j in range(3) if j != k]
        print(f"face {lab} up ({p['n']} pose(s)): mean a = X {p['ma'][0]:+.4f}  Y {p['ma'][1]:+.4f}  Z {p['ma'][2]:+.4f}   |a| = {p['amag']:.4f}")
        print(f"   offset on the two horizontal axes ({AX[oth[0]]}, {AX[oth[1]]}): {p['ma'][oth[0]]:+.4f}, {p['ma'][oth[1]]:+.4f} m/s^2"
              f"  (= bias + gravity coupling; surface tilt cancels only if the poses are spread in heading)")
        if p["spread"] is not None:
            print(f"   spread of those across poses: {p['spread'][oth[0]]:.4f}, {p['spread'][oth[1]]:.4f} m/s^2"
                  f"  (~{np.degrees(max(p['spread'][oth])/p['amag']):.2f} deg of tilt/placement change; small = trustworthy)")
        else:
            print("   only one pose of this face: tilt of the surface is NOT averaged out; treat the offset as +/- (tilt * 9.8) = ~0.17 m/s^2 per degree")
        if "+" + AX[k] in poses and "-" + AX[k] in poses:
            pass
    for i in range(3):
        if "+" + AX[i] in poses and "-" + AX[i] in poses:
            b = (poses["+" + AX[i]]["ma"][i] + poses["-" + AX[i]]["ma"][i]) / 2
            sv = (poses["+" + AX[i]]["ma"][i] - poses["-" + AX[i]]["ma"][i]) / 2
            print(f"axis {AX[i]} (both faces captured): bias {b:+.4f} m/s^2 ({b/9.80665*1e3:+.1f} mg), sensitivity {sv:.4f}"
                  + (f", scale vs g={gloc:.4f}: {100*(sv/gloc-1):+.3f}%" if gloc > 0 else ""))
        else:
            have = [f for f in ("+" + AX[i], "-" + AX[i]) if f in poses]
            print(f"axis {AX[i]}: bias and scale CANNOT be separated without both faces ({' '.join(have) or 'none'} captured); a single face gives bias + scale*g combined"
                  + (f" -> vertical reading minus local g = {abs(poses[have[0]]['ma'][i]) - gloc:+.4f} m/s^2" if have and gloc > 0 else (" (pass --g LOCAL_G to see reading minus g)" if have else "")))
    gyro_section([q for lst in faces.values() for q in lst], " (all still poses)")
    print("\n== NOTE: with fewer than six faces the scale/bias split and axis-to-axis scale mismatch are not observable. Rough guide: horizontal"
          "\n   offset < 0.1 m/s^2, gyro bias < 0.01 rad/s with spread < 0.005 rad/s.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target")
    ap.add_argument("--label", action="store_true")
    ap.add_argument("--g", type=float, default=0.0, help="local gravity m/s^2 (default: derived from the data)")
    a = ap.parse_args()
    if a.label:
        p = pose(a.target)
        print(f"LABEL={p['label']} STILL={int(still_ok(p))} FRAC={p['frac']:.4f} GSTD={p['gstd_max']:.5f} DRIFT={p['drift']:.4f} "
              f"AMAG={p['amag']:.4f} N={p['n']}")
        return 0
    files = sorted(glob.glob(os.path.join(a.target, "pose_*.csv")))
    faces = {}
    for f in files:
        p = pose(f)
        flag = "" if still_ok(p) else "   <-- NOT STILL / ambiguous, EXCLUDED"
        print(f"{os.path.basename(f)}: {p['label']}  |a|={p['amag']:.4f}  frac={p['frac']:.4f}  gyro std max={p['gstd_max']:.5f}  "
              f"half-drift={p['drift']:.4f}  n={p['n']} ({p['dur']:.0f} s){flag}")
        if still_ok(p):
            faces.setdefault(p["label"], []).append(p)     # several poses of one face (e.g. different headings) are averaged
    if not faces:
        print("no usable (still) poses")
        return 1
    poses = {}
    for lab, lst in faces.items():
        ma = np.mean([q["ma"] for q in lst], axis=0)
        poses[lab] = dict(label=lab, ma=ma, sa=np.mean([q["sa"] for q in lst], axis=0), mg=np.mean([q["mg"] for q in lst], axis=0),
                          sg=np.mean([q["sg"] for q in lst], axis=0), amag=float(np.linalg.norm(ma)), n=len(lst),
                          spread=(np.std([q["ma"] for q in lst], axis=0) if len(lst) > 1 else None))
    miss = [s_ + x for x in AX for s_ in "+-" if s_ + x not in poses]
    if miss:
        return partial(poses, faces, miss, a.g)
    ordered = [poses[s_ + x] for x in AX for s_ in "+-"]
    B = np.mean([p["ma"] for p in ordered], axis=0)
    S = np.array([(poses["+" + AX[i]]["ma"][i] - poses["-" + AX[i]]["ma"][i]) / 2 for i in range(3)])
    g_loc = a.g if a.g > 0 else float(S.mean())
    print("\n== ACCELEROMETER")
    print(f"bias (m/s^2):      X {B[0]:+.4f}  Y {B[1]:+.4f}  Z {B[2]:+.4f}    (mg: {B[0]/9.80665*1e3:+.1f} {B[1]/9.80665*1e3:+.1f} {B[2]/9.80665*1e3:+.1f})")
    print(f"sensitivity S_i:   X {S[0]:.4f}  Y {S[1]:.4f}  Z {S[2]:.4f}  m/s^2 per g-axis  (mean {S.mean():.4f})")
    print(f"relative scale mismatch between axes (indep. of local g): " + "  ".join(f"{AX[i]} {100*(S[i]/S.mean()-1):+.3f}%" for i in range(3)))
    src = "given by --g" if a.g > 0 else "DERIVED from the data (mean S), so absolute scale below is 0 by construction"
    print(f"absolute scale vs g = {g_loc:.4f} ({src}): " + "  ".join(f"{AX[i]} {100*(S[i]/g_loc-1):+.3f}%" for i in range(3)))
    print(f"scale vs standard g {G_REF}: " + "  ".join(f"{AX[i]} {100*(S[i]/G_REF-1):+.3f}%" for i in range(3)) + "   (local g differs from 9.80665 by up to ~0.3%)")
    mags = np.array([np.linalg.norm(p["ma"] - B) for p in ordered])
    print(f"|a - bias| per pose: " + " ".join(f"{p['label']}={m:.4f}" for p, m in zip(ordered, mags)) + f"   spread {100*(mags.max()-mags.min())/mags.mean():.3f}%")
    print("cross-axis coupling (deg), response of the other axes when one axis is up (includes surface tilt + non-orthogonality):")
    for i in range(3):
        d = (poses["+" + AX[i]]["ma"] - poses["-" + AX[i]]["ma"]) / 2 / S[i]
        print("   gravity along " + AX[i] + ": " + "  ".join(f"{AX[j]} {np.degrees(d[j]):+.2f}" for j in range(3) if j != i))
    print("accel noise std per pose (m/s^2): mean X/Y/Z = " + " ".join(f"{np.mean([p['sa'][i] for p in ordered]):.4f}" for i in range(3)))
    gyro_section(ordered)
    print("\n== ROUGH GUIDE (typical MEMS, not a datasheet spec): accel bias < 0.1 m/s^2, scale mismatch < 0.5%, cross-axis < 1.5 deg,"
          "\n   |a| spread < 0.3%, gyro bias < 0.01 rad/s with spread < 0.005 rad/s. Everything above is for DID_IMU_RAW as delivered.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
