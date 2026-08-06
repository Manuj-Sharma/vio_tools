#!/usr/bin/env python3
"""bag_stats.py -- exhaustive statistics for a camera+IMU rosbag (ROS1 or ROS2).

Prints everything we have found ourselves needing when a VIO run misbehaves:
timing hygiene, IMU noise spectra, image quality, motion content, and a measured
camera-IMU time offset. Sections are independent -- a failure in one does not
stop the rest.

  bag_stats.py BAG [--cam TOPIC] [--imu TOPIC] [--max-img N] [--flow N]
               [--chain PATH] [--json OUT.json]

BAG may be a ROS2 directory (sqlite3 or mcap) or a ROS1 .bag file.

Notation used below
  dt          inter-message interval, ms
  MAD         median absolute deviation, a robust spread measure
  PSD         power spectral density (Welch), units^2/Hz
  noise dens. sqrt(PSD) in the flat band = continuous-time sigma, units/sqrt(Hz)
  ARW/VRW     angle/velocity random walk, the same quantity in deg/sqrt(hr)
              and m/s/sqrt(hr) as IMU datasheets quote it
"""
import argparse
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

# --------------------------------------------------------------------------- io
def read_bag(path, cam_topic, imu_topic, max_img):
    """Return (cam_hdr, cam_recv, imgs, imu_hdr, imu_recv, gyro, accel, meta)."""
    from rosbags.typesys import Stores, get_typestore
    if os.path.isdir(path):
        from rosbags.rosbag2 import Reader
        ts = get_typestore(Stores.ROS2_HUMBLE)
        deser = lambda raw, t: ts.deserialize_cdr(raw, t)
    else:
        from rosbags.rosbag1 import Reader
        ts = get_typestore(Stores.ROS1_NOETIC)
        deser = lambda raw, t: ts.deserialize_ros1(raw, t)

    ch, cr, imgs, ih, ir, g, a = [], [], [], [], [], [], []
    meta = {"topics": {}, "img": set()}
    n_img = 0
    with Reader(path) as r:
        for c in r.connections:
            meta["topics"][c.topic] = (c.msgtype, getattr(c, "msgcount", None))
        for c, bagt, raw in r.messages():
            if c.topic not in (cam_topic, imu_topic):
                continue
            m = deser(raw, c.msgtype)
            hdr = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if c.topic == cam_topic:
                ch.append(hdr); cr.append(bagt * 1e-9)
                meta["img"].add((m.width, m.height, m.encoding))
                n_img += 1
                if len(imgs) < max_img:
                    arr = np.frombuffer(m.data, np.uint8)
                    if m.encoding == "mono8":
                        imgs.append(arr.reshape(m.height, m.width))
            else:
                ih.append(hdr); ir.append(bagt * 1e-9)
                g.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
                a.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    meta["n_img_total"] = n_img
    o = np.argsort(ih)
    return (np.array(ch), np.array(cr), imgs,
            np.array(ih)[o], np.array(ir)[o],
            np.array(g)[o], np.array(a)[o], meta)


def p(label, val, unit="", w=34):
    print(f"  {label:<{w}} {val}{(' ' + unit) if unit else ''}")


def sec(title):
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


def stat_block(x, name, unit, out=None):
    """Full descriptive stats for a 1-D array."""
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        print(f"  {name}: EMPTY"); return
    q = np.percentile(x, [0.1, 1, 5, 25, 50, 75, 95, 99, 99.9])
    mad = np.median(np.abs(x - np.median(x)))
    print(f"  {name} [{unit}]")
    print(f"     n {x.size}   mean {x.mean():.6g}   std {x.std(ddof=1):.6g}   "
          f"MAD {mad:.6g}   (robust sigma {1.4826*mad:.6g})")
    print(f"     min {x.min():.6g}   max {x.max():.6g}   range {x.max()-x.min():.6g}")
    print(f"     pct  0.1% {q[0]:.6g} | 1% {q[1]:.6g} | 5% {q[2]:.6g} | 25% {q[3]:.6g} | "
          f"50% {q[4]:.6g}")
    print(f"          75% {q[5]:.6g} | 95% {q[6]:.6g} | 99% {q[7]:.6g} | 99.9% {q[8]:.6g}")
    # third and fourth moments say whether the distribution is well behaved
    m2 = ((x - x.mean()) ** 2).mean()
    if m2 > 0:
        skew = ((x - x.mean()) ** 3).mean() / m2 ** 1.5
        kurt = ((x - x.mean()) ** 4).mean() / m2 ** 2 - 3.0
        print(f"     skewness {skew:+.3f}   excess kurtosis {kurt:+.3f}   "
              f"({'gaussian-like' if abs(skew)<0.5 and abs(kurt)<1 else 'non-gaussian'})")
    if out is not None:
        out[name] = dict(n=int(x.size), mean=float(x.mean()), std=float(x.std(ddof=1)),
                         median=float(q[4]), min=float(x.min()), max=float(x.max()))


def timing_block(t, name, out=None):
    t = np.sort(t)
    dt = np.diff(t) * 1e3
    dur = t[-1] - t[0]
    rate = (len(t) - 1) / dur if dur > 0 else 0
    med = np.median(dt)
    print(f"\n  --- {name} ---")
    p("messages", len(t))
    p("duration", f"{dur:.3f}", "s")
    p("mean rate", f"{rate:.3f}", "Hz")
    p("nominal period (median dt)", f"{med:.4f}", "ms")
    stat_block(dt, f"{name} inter-message dt", "ms", out)
    p("monotonic (all dt>0)", bool(np.all(dt > 0)))
    p("duplicate stamps (dt==0)", int((dt == 0).sum()))
    p("negative dt", int((dt < 0).sum()))
    for k in (2, 5, 10):
        n = int((dt > k * med).sum())
        p(f"gaps > {k}x nominal", f"{n}  ({100*n/len(dt):.2f}%)")
    # jitter as a fraction of the nominal interval
    p("jitter std / nominal", f"{dt.std()/med:.4f}")
    # is the stream a perfect arithmetic sequence? (synthetic timestamps)
    n = np.arange(len(t))
    A = np.polyfit(n, t, 1)
    resid = (t - np.polyval(A, n)) * 1e6
    p("fit to perfect grid, residual", f"std {resid.std():.1f} us  max {np.abs(resid).max():.1f} us")
    if resid.std() < 500:
        print("       ^ NOTE nearly a perfect arithmetic sequence -- either a free-running")
        print("         sensor with a hardware timestamp, or synthesised from a counter")
    epoch = "UNIX/REALTIME" if t[0] > 1e9 else "MONOTONIC/uptime"
    p("epoch", f"{epoch}  (t0 = {t[0]:.3f})")
    return dict(rate=rate, med_dt=med, dur=dur, epoch=epoch)


def psd_density(x, fs, band=(20, None), nperseg=4096):
    """sqrt of median PSD in a flat band = continuous-time noise density."""
    try:
        from scipy.signal import welch
    except ImportError:
        return None, None, None
    f, P = welch(x - np.mean(x), fs=fs, nperseg=min(nperseg, len(x)))
    hi = band[1] if band[1] else 0.95 * fs / 2
    m = (f > band[0]) & (f < hi)
    if m.sum() < 5:
        return None, f, P
    return math.sqrt(np.median(P[m])), f, P


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--cam", default="/cam0/image_raw")
    ap.add_argument("--imu", default="/imu0")
    ap.add_argument("--max-img", type=int, default=2500, help="images held in RAM")
    ap.add_argument("--flow", type=int, default=1500, help="frames used for flow/offset")
    ap.add_argument("--chain", default="/home/vc/sensor_ws/src/sensor_bringup/config/"
                                       "ov_9281_aerial_staticinit/kalibr_imucam_chain.yaml")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()
    J = {}

    print(f"bag_stats  {args.bag}")
    ch, cr, imgs, ih, ir, gyr, acc, meta = read_bag(args.bag, args.cam, args.imu, args.max_img)

    sec("1. CONTENTS")
    for t, (ty, n) in meta["topics"].items():
        p(t, f"{ty}   {n if n is not None else ''}")
    p("images total", meta["n_img_total"])
    p("image format", meta["img"] if meta["img"] else "n/a")
    p("images decoded for analysis", len(imgs))
    if len(ch) == 0 or len(ih) == 0:
        print("\n  *** missing camera or IMU topic -- cannot continue ***"); return

    sec("2. TIMING (header stamps)")
    J["cam_timing"] = timing_block(ch, "camera")
    J["imu_timing"] = timing_block(ih, "imu")

    sec("3. TRANSPORT LATENCY (bag receive time - header stamp)")
    print("  A large CONSTANT offset only means the two clocks differ in epoch.")
    print("  What matters is the VARIATION, which is publish->record delay.\n")
    for nm, h, rr in (("camera", ch, cr), ("imu", ih, ir)):
        lat = (rr - h) * 1e3
        rel = lat - np.median(lat)
        drift = np.polyfit(h - h[0], rel, 1)[0]
        p(f"{nm} delay spread", f"p50 0.00  p99 {np.percentile(rel,99):.2f}  max {rel.max():.2f} ms")
        p(f"{nm} clock drift vs bag clock", f"{drift*1e3:+.1f} ppm")

    sec("4. CROSS-SENSOR TIMING")
    e1 = "UNIX" if ch[0] > 1e9 else "MONO"; e2 = "UNIX" if ih[0] > 1e9 else "MONO"
    p("epochs", f"camera {e1}  imu {e2}  ->  {'AGREE' if e1==e2 else '*** DISAGREE ***'}")
    lo, hi = max(ch[0], ih[0]), min(ch[-1], ih[-1])
    p("overlapping span", f"{hi-lo:.3f} s")
    p("imu fully spans camera", bool(ih[0] <= ch[0] and ih[-1] >= ch[-1]))
    p("images outside imu span", int(((ch < ih[0]) | (ch > ih[-1])).sum()))
    idx = np.clip(np.searchsorted(ih, ch), 1, len(ih) - 1)
    gap_b = (ch - ih[idx - 1]) * 1e3
    gap_a = (ih[idx] - ch) * 1e3
    worst = np.maximum(gap_b, gap_a)
    stat_block(worst, "worst-side IMU gap per image", "ms", J)
    p("images with >20 ms one-sided gap", int((worst > 20).sum()))
    p("imu samples per image (mean)", f"{len(ih)/len(ch):.2f}")

    sec("5. IMU STATISTICS")
    an = np.linalg.norm(acc, axis=1); wn = np.linalg.norm(gyr, axis=1)
    for i, ax in enumerate("xyz"):
        stat_block(acc[:, i], f"accel {ax}", "m/s^2")
    stat_block(an, "|accel|", "m/s^2", J)
    print(f"     gravity check: median |accel| {np.median(an):.4f} vs 9.7801 local "
          f"-> {100*(np.median(an)-9.7801)/9.7801:+.2f}%")
    for i, ax in enumerate("xyz"):
        stat_block(gyr[:, i], f"gyro {ax}", "rad/s")
    stat_block(wn, "|gyro|", "rad/s", J)
    print(f"     |gyro| in deg/s: median {math.degrees(np.median(wn)):.2f}  "
          f"p99 {math.degrees(np.percentile(wn,99)):.2f}  max {math.degrees(wn.max()):.2f}")
    p("non-finite samples", f"accel {int((~np.isfinite(acc)).sum())}  gyro {int((~np.isfinite(gyr)).sum())}")

    sec("6. IMU NOISE SPECTRUM")
    print("  Resampled to a uniform grid, Welch PSD, density = sqrt(median PSD) in a")
    print("  band above manoeuvring and below Nyquist. This is the sigma a filter")
    print("  should be told, and it INCLUDES platform vibration -- which is why a")
    print("  bench calibration under-states it in flight.\n")
    fs = 1.0 / np.median(np.diff(ih))
    grid = np.arange(ih[0], ih[-1], 1 / fs)
    band = (min(20, 0.2 * fs / 2), None)
    dens = {}
    for X, lab, unit, cvt in ((acc, "accel", "m/s^2/sqrt(Hz)", None),
                              (gyr, "gyro", "rad/s/sqrt(Hz)", None)):
        print(f"  {lab}  (band {band[0]:.0f}-{0.95*fs/2:.0f} Hz, fs={fs:.1f} Hz)")
        for i, ax in enumerate("xyz"):
            xi = np.interp(grid, ih, X[:, i])
            d, f, P = psd_density(xi, fs, band)
            if d is None:
                print(f"     {ax}: scipy unavailable"); continue
            dens[f"{lab}_{ax}"] = d
            extra = ""
            if lab == "gyro":
                extra = f"   = {math.degrees(d)*60:.4f} deg/sqrt(hr) ARW"
            else:
                extra = f"   = {d*60:.4f} m/s/sqrt(hr) VRW"
            print(f"     {ax}: {d:.5g} {unit}{extra}")
        if dens:
            v = [dens[f'{lab}_{a}'] for a in 'xyz' if f'{lab}_{a}' in dens]
            if v:
                print(f"     -> worst axis {max(v):.5g}, geometric mean "
                      f"{np.exp(np.mean(np.log(v))):.5g} {unit}")
    J["noise_density"] = dens
    # dominant vibration lines
    try:
        from scipy.signal import welch
        xi = np.interp(grid, ih, an)
        f, P = welch(xi - xi.mean(), fs=fs, nperseg=min(4096, len(xi)))
        k = np.argsort(P)[::-1][:5]
        print("\n  dominant |accel| spectral peaks (vibration lines):")
        for i in sorted(k, key=lambda j: -P[j]):
            if f[i] > 1:
                print(f"     {f[i]:7.2f} Hz   PSD {P[i]:.4g}   amplitude {math.sqrt(2*P[i]*(f[1]-f[0])):.4g} m/s^2")
    except ImportError:
        pass

    sec("7. STATIC-PERIOD ANALYSIS (bias / short-tau noise)")
    win = int(0.5 * fs)
    if len(an) > 4 * win:
        k = np.ones(win) / win
        va = np.convolve((an - np.convolve(an, k, "same")) ** 2, k, "same")
        quiet = va < np.percentile(va, 5)
        if quiet.sum() > win:
            print(f"  quietest 5% of the record ({quiet.sum()} samples, {quiet.sum()/fs:.1f} s)")
            p("  |accel| there", f"{np.median(an[quiet]):.5f} m/s^2  (gravity estimate)")
            for i, ax in enumerate("xyz"):
                p(f"  gyro {ax} bias estimate", f"{np.median(gyr[quiet,i]):+.6f} rad/s "
                                                f"({math.degrees(np.median(gyr[quiet,i]))*3600:+.1f} deg/hr)")
            p("  accel noise there (std)", f"{an[quiet].std():.5f} m/s^2")
            p("  gyro noise there (std)", f"{np.linalg.norm(gyr[quiet],axis=1).std():.6f} rad/s")
    else:
        print("  record too short")

    sec("8. IMAGE STATISTICS")
    if cv2 is None or not imgs:
        print("  no decoded mono8 images (or cv2 missing)")
    else:
        step = max(1, len(imgs) // 120)
        sel = list(range(0, len(imgs), step))
        mean_, std_, sat, blk, corn, sharp, ent = [], [], [], [], [], [], []
        fast = cv2.FastFeatureDetector_create(20)
        hashes = []
        for i in sel:
            im = imgs[i]
            mean_.append(im.mean()); std_.append(im.std())
            sat.append((im >= 254).mean() * 100); blk.append((im <= 1).mean() * 100)
            corn.append(len(fast.detect(im, None)))
            sharp.append(cv2.Laplacian(im, cv2.CV_64F).var())
            h = np.bincount(im.ravel(), minlength=256).astype(float); h /= h.sum()
            ent.append(-(h[h > 0] * np.log2(h[h > 0])).sum())
            hashes.append(hash(im[::37, ::37].tobytes()))
        stat_block(mean_, "frame mean intensity", "0-255", J)
        stat_block(std_, "frame intensity std (contrast)", "0-255")
        stat_block(sat, "saturated-white pixels", "%")
        stat_block(blk, "crushed-black pixels", "%")
        stat_block(ent, "histogram entropy", "bits (max 8)")
        stat_block(corn, "FAST corners (thresh 20)", "per frame", J)
        stat_block(sharp, "Laplacian variance (sharpness)", "")
        dup = sum(1 for i in range(1, len(hashes)) if hashes[i] == hashes[i - 1])
        p("byte-identical consecutive frames", f"{dup}  (frozen pipeline if >0)")

    sec("9. MOTION AND BLUR")
    fx = fy = None
    try:
        import yaml
        cal = yaml.safe_load(open(args.chain).read().replace("%YAML:1.0", ""))["cam0"]
        fx, fy = cal["intrinsics"][0], cal["intrinsics"][1]
        R_ci = np.array(cal["T_cam_imu"])[:3, :3]
        ts_cal = cal["timeshift_cam_imu"]
        p("calibration", f"fx {fx:.2f}  fy {fy:.2f}  timeshift {ts_cal*1000:+.3f} ms")
    except Exception as e:
        R_ci = np.eye(3); ts_cal = None
        print(f"  (no calibration chain: {e})")
    if fx:
        for t_exp in (1e-3, 4e-3, 1e-2):
            b = fx * wn * t_exp
            print(f"  rotational blur @ {t_exp*1e3:4.1f} ms exposure: "
                  f"median {np.median(b):5.2f}  p95 {np.percentile(b,95):6.2f}  max {b.max():7.2f} px")
    # angular acceleration -- what makes a lever arm observable
    dw = np.linalg.norm(np.diff(gyr, axis=0), axis=1) / np.maximum(np.diff(ih), 1e-6)
    stat_block(dw, "angular acceleration |dw/dt|", "rad/s^2")

    sec("10. CAMERA-IMU TIME OFFSET (measured)")
    if cv2 is None or len(imgs) < 50:
        print("  not enough decoded images")
    else:
        nfl = min(args.flow, len(imgs), len(ch))
        wz, wt = [], []
        lk = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        prev = None
        for i in range(nfl):
            cur = cv2.resize(imgs[i], (imgs[i].shape[1] // 2, imgs[i].shape[0] // 2))
            if prev is not None:
                p0 = cv2.goodFeaturesToTrack(prev, 300, 0.01, 8)
                if p0 is not None and len(p0) > 30:
                    p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, cur, p0, None, **lk)
                    if st is not None and st.sum() > 20:
                        M, _ = cv2.estimateAffinePartial2D(p0[st == 1], p1[st == 1],
                                                           method=cv2.RANSAC,
                                                           ransacReprojThreshold=2.0)
                        if M is not None:
                            dt = ch[i] - ch[i - 1]
                            if 0 < dt < 0.2:
                                wz.append(math.atan2(M[1, 0], M[0, 0]) / dt)
                                wt.append(0.5 * (ch[i] + ch[i - 1]))
            prev = cur
        wz = np.array(wz); wt = np.array(wt)
        if len(wz) < 50:
            print("  too few usable flow frames")
        else:
            gz = (R_ci @ gyr.T).T[:, 2]
            grid = np.arange(max(wt[0], ih[0]), min(wt[-1], ih[-1]), 0.002)
            A = np.interp(grid, wt, wz); B = np.interp(grid, ih, gz)
            A = (A - A.mean()) / A.std(); B = (B - B.mean()) / B.std()
            ml = int(0.150 / 0.002)
            full = np.correlate(A, B, "full") / len(A)
            mid = len(A) - 1
            segd = full[mid - ml: mid + ml + 1]
            k = int(np.argmax(np.abs(segd)))
            lag = (k - ml) * 2.0
            if 0 < k < len(segd) - 1:
                y0, y1, y2 = segd[k - 1], segd[k], segd[k + 1]
                den = y0 - 2 * y1 + y2
                if den:
                    lag += 1.0 * (y0 - y2) / den
            p("usable flow frames", f"{len(wz)} / {nfl}")
            p("image rotation rate std", f"{wz.std():.5f} rad/s")
            p("gyro-z (camera frame) std", f"{gz.std():.5f} rad/s")
            p("peak |correlation|", f"{abs(segd[k]):.4f}")
            p("peak sign", "+" if segd[k] > 0 else "-  (anti-correlated)")
            p("raw cross-correlation lag", f"{lag:+.2f} ms")
            print()
            print("  *** SIGN CAVEAT -- READ BEFORE USING ***")
            print("  The MAGNITUDE below is well determined by the correlation peak.")
            print("  Its SIGN in Kalibr's convention (t_imu = t_cam + shift) has NOT")
            print("  been reliably derived here; an earlier attempt got it backwards and")
            print("  a test of the wrong sign made VIO 20x worse. Treat |offset| as the")
            print("  result and settle the sign by sweeping timeshift_cam_imu in the")
            print("  estimator, which is cheap and unambiguous.")
            p("  |offset| magnitude", f"{abs(lag):.2f} ms")
            if ts_cal is not None:
                p("  calibrated timeshift", f"{ts_cal*1000:+.3f} ms")
                p("  |calibrated|", f"{abs(ts_cal*1000):.3f} ms")
                p("  magnitude difference", f"{abs(lag)-abs(ts_cal*1000):+.2f} ms")
                print("  If the driver stamps at END-OF-FRAME, a magnitude difference of D ms")
                print(f"  implies an exposure difference of 2D = {2*abs(abs(lag)-abs(ts_cal*1000)):.1f} ms")
            q = abs(segd[k])
            p("  verdict", "strong, magnitude trustworthy" if q > 0.7 else
                           "moderate, indicative" if q > 0.4 else
                           "WEAK -- insufficient rotational excitation")
            J["offset_ms_magnitude"] = abs(lag)
            J["offset_correlation"] = float(q)

    sec("11. VIO SUITABILITY SUMMARY")
    flags = []
    if not np.all(np.diff(np.sort(ih)) > 0): flags.append("IMU stamps not strictly increasing")
    if (worst > 20).sum(): flags.append(f"{int((worst>20).sum())} images poorly bracketed by IMU")
    if e1 != e2: flags.append("camera and IMU on different clock epochs")
    if abs(np.median(an) - 9.7801) > 0.3: flags.append(f"|accel| median {np.median(an):.2f} far from local g")
    if dens.get("accel_z", 0) > 0.05:
        flags.append(f"high accel noise/vibration ({dens['accel_z']:.3f} m/s^2/sqrt(Hz) on z) "
                     f"-- a bench-derived noise model will make the filter over-trust it")
    if cv2 and imgs and np.mean(corn) < 100: flags.append("sparse texture (<100 FAST corners/frame)")
    if cv2 and imgs and np.mean(sat) > 3: flags.append(f"{np.mean(sat):.1f}% saturated pixels")
    if flags:
        for f in flags: print(f"  [!] {f}")
    else:
        print("  no blocking issues found")

    if args.json:
        json.dump(J, open(args.json, "w"), indent=2, default=str)
        print(f"\n  machine-readable summary -> {args.json}")


if __name__ == "__main__":
    main()
