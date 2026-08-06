#!/usr/bin/env python3
"""bag_quality.py -- audit a cam+IMU bag, including a MEASURED cam-IMU time offset.

Two independent things get checked:

  HYGIENE   monotonicity, rate, jitter, dropouts, epoch agreement, and whether
            every image is bracketed by IMU samples. These are the faults that
            make OpenVINS silently discard data.

  PHYSICS   does what the camera SAW agree with what the IMU FELT, at the
            timestamps claimed? Image-plane rotation between consecutive frames
            is compared against gyro-z rotated into the camera frame, and the
            two are cross-correlated. The lag at peak correlation IS the
            effective cam-IMU time offset, measured from this bag alone and
            independent of Kalibr.

  bag_quality.py ROS2_BAG_DIR [MAX_FRAMES]
"""
import sys

import cv2
import numpy as np
import yaml
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

BAG = sys.argv[1]
MAXF = int(sys.argv[2]) if len(sys.argv) > 2 else 4000
CHAIN = "/home/vc/sensor_ws/src/sensor_bringup/config/ov_9281_aerial_staticinit/kalibr_imucam_chain.yaml"
TS = get_typestore(Stores.ROS2_HUMBLE)

# calibration -- fx and R_cam_imu, and the timeshift we will compare against
raw = open(CHAIN).read().replace("%YAML:1.0", "")
cal = yaml.safe_load(raw)["cam0"]
FX, FY = cal["intrinsics"][0], cal["intrinsics"][1]
T = np.array(cal["T_cam_imu"])
R_cam_imu = T[:3, :3]
TS_CAL = cal["timeshift_cam_imu"]

print(f"bag: {BAG}")
print(f"calib: fx {FX:.1f}  timeshift {TS_CAL*1000:+.3f} ms\n")

cam_t, cam_img, imu_t, gyr, acc = [], [], [], [], []
with Reader(BAG) as r:
    n = 0
    for c, _, rawmsg in r.messages():
        m = TS.deserialize_cdr(rawmsg, c.msgtype)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if c.topic == "/cam0/image_raw":
            cam_t.append(t)
            n += 1
            if n <= MAXF:
                img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
                cam_img.append(cv2.resize(img, (m.width // 2, m.height // 2)))
        else:
            imu_t.append(t)
            gyr.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
            acc.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))

cam_t = np.array(cam_t); imu_t = np.array(imu_t)
gyr = np.array(gyr); acc = np.array(acc)
o = np.argsort(imu_t); imu_t, gyr, acc = imu_t[o], gyr[o], acc[o]

# ---------------------------------------------------------------- hygiene
print("=" * 68)
print("TIMESTAMP HYGIENE")
print("=" * 68)
for nm, t, want in (("camera", cam_t, None), ("imu", imu_t, None)):
    d = np.diff(np.sort(t)) * 1e3
    print(f"  {nm:<7} {len(t):6d} msgs  {(len(t)-1)/(t[-1]-t[0]):6.2f} Hz  "
          f"dt mean {d.mean():7.3f}  std {d.std():6.3f}  min {d.min():6.3f}  max {d.max():8.3f} ms")
    print(f"          monotonic={bool(np.all(d>0))}  dup/neg={int((d<=0).sum())}  "
          f"gaps>2x={int((d>2*np.median(d)).sum())}  gaps>5x={int((d>5*np.median(d)).sum())}")
epoch = lambda x: "UNIX" if x > 1e9 else "MONOTONIC"
print(f"  epoch: camera {epoch(cam_t[0])}  imu {epoch(imu_t[0])}  "
      f"{'AGREE' if epoch(cam_t[0])==epoch(imu_t[0]) else '*** DISAGREE ***'}")
idx = np.clip(np.searchsorted(imu_t, cam_t), 1, len(imu_t) - 1)
br = np.maximum(cam_t - imu_t[idx - 1], imu_t[idx] - cam_t) * 1e3
print(f"  imu brackets each image: median {np.median(br):.2f} ms  worst {br.max():.2f} ms  "
      f"images with >20 ms gap: {int((br>20).sum())}")
print(f"  imu spans camera: {imu_t.min()<=cam_t.min() and imu_t.max()>=cam_t.max()}")

# ------------------------------------------------------------ sensor sanity
an = np.linalg.norm(acc, axis=1); wn = np.linalg.norm(gyr, axis=1)
print(f"\n  |accel| median {np.median(an):.3f} m/s^2 (expect ~9.78)   "
      f"|gyro| median {np.median(wn):.3f}  p99 {np.percentile(wn,99):.3f}  max {wn.max():.3f} rad/s")
print(f"  NaN/inf: accel {int((~np.isfinite(acc)).sum())}  gyro {int((~np.isfinite(gyr)).sum())}")

# ------------------------------------------------------------ image quality
N = len(cam_img)
sub = range(0, N, max(1, N // 60))
means, clip, corners, sharp = [], [], [], []
fast = cv2.FastFeatureDetector_create(20)
for i in sub:
    im = cam_img[i]
    means.append(im.mean()); clip.append((im >= 254).mean() * 100)
    corners.append(len(fast.detect(im, None)))
    sharp.append(cv2.Laplacian(im, cv2.CV_64F).var())
print(f"\n  image: mean {np.mean(means):.1f}/255   clipped {np.mean(clip):.2f}%   "
      f"FAST corners {np.mean(corners):.0f}/frame (half-res)   sharpness {np.mean(sharp):.0f}")

# -------------------------------------------------- MEASURED cam-IMU offset
print("\n" + "=" * 68)
print("CAM-IMU TIME OFFSET, MEASURED FROM THIS BAG")
print("=" * 68)
# per-frame image rotation from LK-tracked points
wz_img, wz_t = [], []
prev = None
lk = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
for i in range(min(N, len(cam_t))):
    cur = cam_img[i]
    if prev is not None:
        p0 = cv2.goodFeaturesToTrack(prev, 300, 0.01, 8)
        if p0 is not None and len(p0) > 30:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, cur, p0, None, **lk)
            if st is not None and st.sum() > 20:
                a, inl = cv2.estimateAffinePartial2D(p0[st == 1], p1[st == 1],
                                                     method=cv2.RANSAC, ransacReprojThreshold=2.0)
                if a is not None:
                    dt = cam_t[i] - cam_t[i - 1]
                    if 0 < dt < 0.2:
                        wz_img.append(np.arctan2(a[1, 0], a[0, 0]) / dt)
                        wz_t.append(0.5 * (cam_t[i] + cam_t[i - 1]))
    prev = cur
wz_img = np.array(wz_img); wz_t = np.array(wz_t)

# gyro rotated into the camera frame
w_cam = (R_cam_imu @ gyr.T).T
gz = w_cam[:, 2]

grid = np.arange(max(wz_t[0], imu_t[0]), min(wz_t[-1], imu_t[-1]), 0.002)   # 2 ms
A = np.interp(grid, wz_t, wz_img); B = np.interp(grid, imu_t, gz)
A -= A.mean(); B -= B.mean()
A /= A.std(); B /= B.std()
maxlag = int(0.150 / 0.002)
full = np.correlate(A, B, "full") / len(A)
mid = len(A) - 1
seg = full[mid - maxlag: mid + maxlag + 1]
k = int(np.argmax(np.abs(seg)))
lag_ms = (k - maxlag) * 2.0
# parabolic refinement of the peak
if 0 < k < len(seg) - 1:
    y0, y1, y2 = seg[k - 1], seg[k], seg[k + 1]
    denom = (y0 - 2 * y1 + y2)
    if denom != 0:
        lag_ms += 2.0 * 0.5 * (y0 - y2) / denom
print(f"  frames with usable flow: {len(wz_img)} / {N}")
print(f"  image rotation rate: std {wz_img.std():.4f} rad/s   gyro-z std {gz.std():.4f} rad/s")
# Sign convention, validated against a synthetic known offset: a delay of the
# SECOND signal (gyro) by +X ms is reported by np.correlate as -X. So the offset
# in Kalibr's sense (t_imu = t_cam + shift) is -lag_ms.
shift_ms = -lag_ms
print(f"  peak |correlation| {abs(seg[k]):.3f} at raw lag {lag_ms:+.2f} ms")
print(f"  sign of peak: {'+' if seg[k]>0 else '-'} "
      f"({'image and gyro agree in sign' if seg[k]>0 else 'anti-correlated -- expected if axes oppose'})")
print(f"\n  calibrated timeshift  {TS_CAL*1000:+.3f} ms")
print(f"  measured here         {shift_ms:+.2f} ms   (t_imu = t_cam + shift)")
d = shift_ms - TS_CAL*1000
print(f"  difference            {d:+.2f} ms")
print(f"  end-of-frame stamping implies this bag ran at exposure "
      f"~{4000 - 2*d*1000:.0f} us (calibration was 4000 us)")
q = abs(seg[k])
print(f"\n  VERDICT: correlation {q:.2f} -> "
      + ("strong, offset is trustworthy" if q > 0.7 else
         "moderate, indicative only" if q > 0.4 else
         "WEAK -- not enough rotational excitation to measure the offset"))
