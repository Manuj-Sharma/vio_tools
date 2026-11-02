#!/usr/bin/env python3
"""sweep_timeshift.py BAG [--cam cam0|cam1] [--chain YAML] [--skip SEC] [--step N] [--selftest MS]

Estimates the camera-IMU time offset from one bag WITHOUT running the estimator, by sweeping a
candidate offset and correlating how fast the camera saw the rig rotate against how fast the
gyro said it rotated.

  camera rate  : per frame pair (i, i+STEP) -> LK-tracked points, undistorted with the
                 calibration, essential matrix -> relative rotation -> |omega| = angle / dt.
  gyro rate    : |gyro| from /imu0.
  Both are MAGNITUDES, so the result does not depend on axis conventions, T_cam_imu or sign.
  (bag_quality.py correlates a single axis and is much weaker when the motion is not about
  the optical axis: correlation 0.35 on vio_20261022_184933.)

Convention (Kalibr): t_imu = t_cam + shift. A frame stamped t_cam is matched with the gyro at
t_cam + shift; the shift with the highest correlation is reported, with a parabolic refinement,
the width of the peak, and the calibrated value for comparison.

--selftest MS  delays the gyro by MS ms before the sweep. The reported shift must then move by
               +MS (checks the sign convention end to end). Use with the same bag.

Needs only cv2/numpy/yaml and the sqlite3 bag (single or split). ~1-2 min for a 60 s bag.
"""
import argparse
import glob
import os
import sqlite3
import sys

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu

CHAIN = "/home/vc/vio_ar0234_icm20948/openvins_config/ar0234_stereo/kalibr_imucam_chain.yaml"


def hdr(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read_bag(bag, cam_topic):
    files = sorted(glob.glob(os.path.join(bag, "*.db3")),
                   key=lambda f: int(f.rsplit("_", 1)[1].split(".")[0]))
    cam, imu = [], []
    for f in files:
        c = sqlite3.connect(f)
        ids = {n: i for i, n in c.execute("select id,name from topics")}
        if cam_topic in ids:
            for (d,) in c.execute("select data from messages where topic_id=? order by id", (ids[cam_topic],)):
                m = deserialize_message(d, Image)
                cam.append((hdr(m), np.frombuffer(m.data, np.uint8).reshape(m.height, m.width).copy()))
        if "/imu0" in ids:
            for (d,) in c.execute("select data from messages where topic_id=? order by id", (ids["/imu0"],)):
                m = deserialize_message(d, Imu)
                imu.append((hdr(m), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
    imu = np.array(sorted(imu))
    return cam, imu


def camera_rates(cam, K, D, step, skip_s):
    """|omega| from relative rotation between frames i and i+step. Returns (t_mid, rate)."""
    lk = dict(winSize=(31, 31), maxLevel=4,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    t0 = cam[0][0]
    out_t, out_w = [], []
    for i in range(0, len(cam) - step):
        ta, a = cam[i]
        tb, b = cam[i + step]
        if ta - t0 < skip_s:
            continue
        dt = tb - ta
        if not (0 < dt < 0.2):
            continue
        p0 = cv2.goodFeaturesToTrack(a, 400, 0.01, 10)
        if p0 is None or len(p0) < 40:
            continue
        p1, st, _ = cv2.calcOpticalFlowPyrLK(a, b, p0, None, **lk)
        ok = st.ravel() == 1
        if ok.sum() < 40:
            continue
        # undistort to normalised coordinates, then the essential matrix with identity K
        u0 = cv2.undistortPoints(p0[ok].reshape(-1, 1, 2), K, D).reshape(-1, 2)
        u1 = cv2.undistortPoints(p1[ok].reshape(-1, 1, 2), K, D).reshape(-1, 2)
        E, mask = cv2.findEssentialMat(u0, u1, np.eye(3), method=cv2.RANSAC, prob=0.999,
                                       threshold=1.0 / K[0, 0])
        if E is None or E.shape != (3, 3) or mask is None or mask.sum() < 30:
            continue
        _, R, _, _ = cv2.recoverPose(E, u0, u1, np.eye(3), mask=mask)
        ang = np.linalg.norm(cv2.Rodrigues(R)[0])
        out_t.append(0.5 * (ta + tb))
        out_w.append(ang / dt)
    return np.array(out_t), np.array(out_w)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--cam", default="cam0", choices=["cam0", "cam1"])
    ap.add_argument("--chain", default=CHAIN)
    ap.add_argument("--skip", type=float, default=4.0, help="skip the first SEC (recorder startup)")
    ap.add_argument("--step", type=int, default=2, help="frame spacing for the rotation estimate")
    ap.add_argument("--range-ms", type=float, default=80.0)
    ap.add_argument("--selftest", type=float, default=None, metavar="MS")
    ap.add_argument("--cache", default=None, help="npz file: reuse/save the camera rotation rates (the slow part)")
    ap.add_argument("--max-rate", type=float, default=8.0, help="drop camera rate samples above this (rad/s); failed pose fits")
    a = ap.parse_args()

    cal = yaml.safe_load(open(a.chain).read().replace("%YAML:1.0", ""))[a.cam]
    fx, fy, cx, cy = cal["intrinsics"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    D = np.array(cal["distortion_coeffs"], float)
    ts_cal = cal["timeshift_cam_imu"] * 1e3
    print(f"bag {a.bag}\n{a.cam}: fx {fx:.1f}  calibrated timeshift {ts_cal:+.3f} ms  ({os.path.basename(os.path.dirname(a.chain))})")

    if a.cache and os.path.exists(a.cache):
        z = np.load(a.cache)
        tc, wc, imu = z["tc"], z["wc"], z["imu"]
        print(f"loaded cache {a.cache}")
    else:
        cam, imu = read_bag(a.bag, f"/{a.cam}/image_raw")
        print(f"{len(cam)} frames, {len(imu)} imu samples")
        tc, wc = camera_rates(cam, K, D, a.step, a.skip)
        if a.cache:
            np.savez(a.cache, tc=tc, wc=wc, imu=imu)
    keep = wc < a.max_rate
    print(f"dropping {int((~keep).sum())} of {len(wc)} camera rate samples above {a.max_rate} rad/s")
    tc, wc = tc[keep], wc[keep]
    print(f"usable camera rate samples: {len(tc)}   median {np.median(wc):.3f} rad/s  p95 {np.percentile(wc,95):.3f}")
    ti = imu[:, 0] + ((a.selftest or 0.0) * 1e-3)
    wi = np.linalg.norm(imu[:, 1:4], axis=1)
    # smooth the gyro over the same window the camera pair spans, so both see equal bandwidth
    win = max(1, int(round(a.step / 60.0 * 238)))
    wi_s = np.convolve(wi, np.ones(win) / win, mode="same")

    shifts = np.arange(-a.range_ms, a.range_ms + 0.5, 1.0)
    corr = []
    rank = lambda v: np.argsort(np.argsort(v)).astype(float)   # Spearman: robust to the remaining outliers
    rw = rank(wc)
    for s in shifts:
        g = np.interp(tc + s * 1e-3, ti, wi_s)
        corr.append(np.corrcoef(rw, rank(g))[0, 1])
    corr = np.array(corr)
    k = int(np.argmax(corr))
    best = shifts[k]
    if 0 < k < len(shifts) - 1:
        y0, y1, y2 = corr[k - 1], corr[k], corr[k + 1]
        den = y0 - 2 * y1 + y2
        if den != 0:
            best += 0.5 * (y0 - y2) / den
    half = corr[k] - 0.5 * (corr[k] - np.median(corr))
    inside = np.where(corr >= half)[0]
    fwhm = shifts[inside.max()] - shifts[inside.min()] + 1
    print("\nrank correlation vs candidate shift (every 5 ms):")
    for s, c in zip(shifts[::5], corr[::5]):
        print(f"  {s:+6.0f} ms  {c:.3f}  " + "#" * int(max(c, 0) * 60))
    print(f"\npeak correlation {corr[k]:.3f} at shift {best:+.2f} ms   (FWHM ~{fwhm:.0f} ms)")
    print(f"calibrated        {ts_cal:+.2f} ms   -> difference {best - ts_cal:+.2f} ms")
    if a.selftest is not None:
        print(f"SELFTEST: gyro delayed by {a.selftest:+.1f} ms; expected shift = reference {a.selftest:+.1f} ms away.")
    verdict = ("strong" if corr[k] > 0.8 else "moderate" if corr[k] > 0.5 else "WEAK -- indicative only")
    print(f"verdict: {verdict}")


if __name__ == "__main__":
    sys.exit(main())
