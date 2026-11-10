#!/usr/bin/env python3
"""mosaic_from_bag.py -- nadir-camera mosaic from a rosbag with FC position + attitude (no ROS needed; run with a venv that has numpy scipy opencv rosbags).
Fit a few global geometry corrections (FC<->camera time shift, yaw bias, camera roll/pitch mount bias, optional height scale) by making
overlapping frames agree (minimise the weighted per-pixel variance of high-passed frames across overlaps), then render sharp (best-view-per-pixel, no averaging).
usage: mosaic_from_bag.py --bag BAG --meta meta.npz --npz CUBE_imu_att.npz --out DIR [--tag NAME] [--dt-fit 3] [--dt 1.5] [--res 0.25]
meta.npz: P=[t,N,E,D] (/fc/local_position_ned), Q=[t,x,y,z,w] (/fc/attitude), t = bag seconds (receive time).  --npz: Cube log npz (extract_cube_imu.py) used only to fit the IMX->FC-body rotation on the gyro.
Writes OUT/mosaic_TAG.png (north-up gray image, 0.25 m/px, +x east) and OUT/mosaic_TAG.json: mpp, origin_px (col,row of the bag's local-NED origin = ekf_origin), ekf_origin lat/lon/alt,
fitted corrections and R_body_cam (camera axes x right,y down,z forward expressed in the FC body FRD frame) so another flight over the same site can use the mosaic as a map.
Assumes flat ground at the takeoff level. Terrain relief, rolling-shutter and exposure differences are NOT modelled."""
import argparse, json, os, time
import numpy as np, cv2
from scipy.spatial.transform import Rotation as Rot, Slerp
from scipy.signal import butter, sosfiltfilt
from scipy.optimize import minimize
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

ap = argparse.ArgumentParser()
ap.add_argument("--bag", required=True); ap.add_argument("--meta", required=True); ap.add_argument("--npz", required=True); ap.add_argument("--out", required=True)
ap.add_argument("--dt-fit", type=float, default=3.0); ap.add_argument("--dt", type=float, default=1.5); ap.add_argument("--res", type=float, default=0.25)
ap.add_argument("--hmin", type=float, default=190.0); ap.add_argument("--topic", default="/cam0/image_30hz"); ap.add_argument("--cube-offset", type=float, default=-0.14)
ap.add_argument("--tag", default=None); ap.add_argument("--also-unfitted", action="store_true"); ap.add_argument("--tmin", type=float, default=0); ap.add_argument("--tmax", type=float, default=1e9)
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)

K = np.array([[856.7938712247676, 0, 643.9987005563896], [0, 846.2930039349599, 317.1515866234702], [0, 0, 1]])
D = np.array([0.025044503836010295, -0.036757865790211376, -0.006300247070088697, 0.0024553501437199485]); W, H = 1280, 720
T_ci = np.array([[0.0028741963010919945, 0.9999165449821666, 0.012595319231765607], [-0.9999705872462018, 0.0027843319663957843, 0.007146477313172294],
                 [0.00711081135381671, -0.012615489147401389, 0.999895137399649]])
R_imx_cam = T_ci.T
M = np.load(a.meta); P = M["P"]; Q = M["Q"]
slerp = Slerp(Q[:, 0], Rot.from_quat(Q[:, 1:5]))
posf = lambda t: np.array([np.interp(t, P[:, 0], P[:, k]) for k in (1, 2, 3)])
ts = get_typestore(Stores.ROS2_HUMBLE)

# ---- IMX -> Cube body rotation (gyro Kabsch, as in v1) ----
Z = np.load(a.npz); tref, uref = Z["ref"]; c = Z["imu0"]; ct = uref + (c[:, 0] - tref) - a.cube_offset; fs = 100.0
with Reader(a.bag) as r:
    t0 = r.start_time; cons = [x for x in r.connections if x.topic == "/imu0"]; T = []; Wg = []
    for con, t, raw in r.messages(connections=cons, start=t0 + int(40e9), stop=t0 + int(540e9)):
        m = ts.deserialize_cdr(raw, con.msgtype); T.append(t / 1e9); Wg.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
T = np.array(T); Wg = np.array(Wg); tg = np.arange(T[0] + 2, T[-1] - 2, 1 / fs); IW = np.column_stack([np.interp(tg, T, Wg[:, i]) for i in range(3)])
CW = np.column_stack([np.interp(tg, ct, c[:, j]) for j in (1, 2, 3)]); lp = lambda x: sosfiltfilt(butter(4, 3 / (fs / 2), output="sos"), x, axis=0)
best = (-2, 0, None)
for Ls in np.arange(-0.2, 0.2001, 0.01):
    iw = np.column_stack([np.interp(tg + Ls, tg, lp(IW)[:, i]) for i in range(3)]); U, _, Vt = np.linalg.svd(iw.T @ lp(CW)); Rm = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    rr = np.mean([np.corrcoef((iw @ Rm)[:, i], lp(CW)[:, i])[0, 1] for i in range(3)])
    if rr > best[0]: best = (rr, Ls, Rm)
R_body_cam0 = best[2].T @ R_imx_cam

# ---- frames (all selected at dt; the fit uses every k-th) ----
with Reader(a.bag) as r:
    t0 = r.start_time; cons = [x for x in r.connections if x.topic == a.topic]; sel = []; last = -1e9
    for con, t, raw in r.messages(connections=cons):
        tt = (t - t0) / 1e9
        if tt < a.tmin or tt > a.tmax or tt - last < a.dt or -posf(tt)[2] < a.hmin: continue
        last = tt; sel.append((tt, raw, con.msgtype))
    items = []
    for tt, raw, mt in sel:
        m = ts.deserialize_cdr(raw, mt); items.append((tt, np.frombuffer(m.data, np.uint8).reshape(m.height, m.width).copy()))
print(len(items), "frames")
tt_all = np.array([x[0] for x in items]); undist = [cv2.undistort(im, K, D) for _, im in items]

# ---- geometry with global parameters p = [time shift s, yaw deg, roll deg, pitch deg, height scale] ----
def corners(tt, p, pts):
    pos = posf(tt + p[0]); pos = pos * np.array([1, 1, p[4]]); Rnb = (Rot.from_euler("z", p[1], degrees=True) * slerp(np.clip(tt + p[0], Q[0, 0], Q[-1, 0]))).as_matrix()
    Rcam = Rot.from_euler("xy", [p[2], p[3]], degrees=True).as_matrix() @ R_body_cam0
    d = (Rnb @ Rcam @ (np.linalg.inv(K) @ pts.T)).T; s = -pos[2] / d[:, 2]
    return pos[None, :2] + s[:, None] * d[:, :2], d[:, 2]

# ---- fit on a coarse canvas using small high-passed frames ----
FS = 4; ws, hs = W // FS, H // FS; Ks = K.copy(); Ks[:2] /= FS
sub = np.arange(0, len(items), max(1, int(round(a.dt_fit / a.dt))))
hp = []
for i in sub:
    sm = cv2.resize(undist[i], (ws, hs), interpolation=cv2.INTER_AREA).astype(np.float32); sm = sm - cv2.GaussianBlur(sm, (0, 0), 6); sm /= sm.std() + 1e-3; hp.append(sm)
yy, xx = np.mgrid[0:hs, 0:ws]; wimg = np.minimum(np.minimum(xx, ws - 1 - xx) / 15.0, np.minimum(yy, hs - 1 - yy) / 15.0).clip(0, 1).astype(np.float32) + 1e-3
srcs = np.float32([[0, 0], [ws, 0], [ws, hs], [0, hs]]); pts_full = np.array([[0, 0, 1], [W, 0, 1], [W, H, 1], [0, H, 1]], float)
g0 = np.array([corners(tt_all[i], [0, 0, 0, 0, 1], pts_full)[0] for i in sub]); CR = 1.0
Nmin, Nmax = g0[..., 0].min() - 60, g0[..., 0].max() + 60; Emin, Emax = g0[..., 1].min() - 60, g0[..., 1].max() + 60
cw, ch = int((Emax - Emin) / CR) + 1, int((Nmax - Nmin) / CR) + 1
def cost(p, detail=False):
    Sw = np.zeros((ch, cw), np.float32); Sx = np.zeros_like(Sw); Sxx = np.zeros_like(Sw); Nn = np.zeros_like(Sw)
    for k, i in enumerate(sub):
        g, dz = corners(tt_all[i], p, pts_full)
        if np.any(dz < 0.3): return 1e3
        dst = np.float32([[(e - Emin) / CR, (Nmax - n) / CR] for n, e in g]); Hm = cv2.getPerspectiveTransform(srcs, dst)
        wi = cv2.warpPerspective(wimg, Hm, (cw, ch)); wi[wi < 0.05] = 0; x = cv2.warpPerspective(hp[k], Hm, (cw, ch))
        Sw += wi; Sx += wi * x; Sxx += wi * x * x; Nn += (wi > 0)
    ov = Nn >= 2; v = (Sxx - Sx ** 2 / np.maximum(Sw, 1e-6))[ov].sum() / max(Sw[ov].sum(), 1e-6)
    return (v, ov.mean()) if detail else v
p0 = np.array([0.0, 0.0, 0.0, 0.0, 1.0]); c0 = cost(p0); print(f"fit on {len(sub)} frames; start cost {c0:.4f}")
scale = np.array([0.3, 3.0, 2.0, 2.0, 0.03]); t_s = time.time()
res = minimize(lambda z: cost(p0 + z * scale), np.zeros(5), method="Powell", options={"xtol": 1e-2, "ftol": 1e-4, "maxiter": 6})
pf = p0 + res.x * scale; print(f"fitted: time shift {pf[0]:+.3f} s, yaw {pf[1]:+.2f} deg, camera roll {pf[2]:+.2f} deg, pitch {pf[3]:+.2f} deg, height scale {pf[4]:.3f}; cost {res.fun:.4f} (start {c0:.4f}) in {time.time()-t_s:.0f} s")
np.save(f"{a.out}/fitted_params.npy", pf)

# ---- final sharp render (best view per pixel) ----
def render(p, name):
    gs = [corners(tt, p, pts_full) for tt in tt_all]; ok = [bool(np.all(dz > 0.3)) for _, dz in gs]; G = np.array([g for g, _ in gs])
    allg = G[ok].reshape(-1, 2); Nmn, Nmx = allg[:, 0].min(), allg[:, 0].max(); Emn, Emx = allg[:, 1].min(), allg[:, 1].max()
    CWp, CHp = int(np.ceil((Emx - Emn) / a.res)) + 1, int(np.ceil((Nmx - Nmn) / a.res)) + 1
    img = np.zeros((CHp, CWp), np.uint8); bw = np.zeros((CHp, CWp), np.float32)
    cy, cx = np.mgrid[0:H, 0:W]; wc = (1.0 - np.hypot((cx - W / 2) / (W / 2), (cy - H / 2) / (H / 2)) / 1.45).clip(0, 1).astype(np.float32) + 1e-3     # centre-weighted: nearest-nadir wins
    srcf = np.float32([[0, 0], [W, 0], [W, H], [0, H]])
    for tt, u, g, good in zip(tt_all, undist, G, ok):
        if not good: continue
        dst = np.float32([[(e - Emn) / a.res, (Nmx - n) / a.res] for n, e in g]); Hm = cv2.getPerspectiveTransform(srcf, dst)
        wi = cv2.warpPerspective(wc, Hm, (CWp, CHp)); wi[wi < 2e-3] = 0; m = wi > bw
        if not m.any(): continue
        im = cv2.warpPerspective(u, Hm, (CWp, CHp)); img[m] = im[m]; bw[m] = wi[m]
    cv2.imwrite(f"{a.out}/{name}.png", img); np.savez(f"{a.out}/{name}_meta.npz", Emin=Emn, Nmax=Nmx, res=a.res, params=p)
    print("wrote", name, img.shape, "coverage %.0f%%" % (100 * (bw > 0).mean())); return img, Emn, Nmx
tag = a.tag or os.path.basename(os.path.normpath(a.bag)).split("_", 1)[-1]
if a.also_unfitted: render(p0, f"mosaic_{tag}_unfitted")
img, Emn, Nmx = render(pf, f"mosaic_{tag}")
origin = None
with Reader(a.bag) as r:
    for con, t, raw in r.messages(connections=[x for x in r.connections if x.topic == "/fc/ekf_origin"]):
        o = ts.deserialize_cdr(raw, con.msgtype); origin = dict(lat=float(o.latitude), lon=float(o.longitude), alt=float(o.altitude)); break
Rcam = Rot.from_euler("xy", [pf[2], pf[3]], degrees=True).as_matrix() @ R_body_cam0
info = dict(image=f"mosaic_{tag}.png", source_bag=os.path.abspath(a.bag), mpp=a.res, origin_px=[(0.0 - Emn) / a.res, Nmx / a.res], ekf_origin=origin, frame="north-up gray image; x = east, y = south; origin_px = (col,row) of the local NED origin (ekf_origin)",
            fitted=dict(time_shift_s=float(pf[0]), yaw_deg=float(pf[1]), cam_roll_deg=float(pf[2]), cam_pitch_deg=float(pf[3]), height_scale=float(pf[4])), R_body_cam=Rcam.tolist(),
            camera=dict(K=K.tolist(), dist_radtan=D.tolist(), size=[W, H]), frames=len(items), flat_ground_at_takeoff_level=True)
json.dump(info, open(f"{a.out}/mosaic_{tag}.json", "w"), indent=1); print("wrote", f"{a.out}/mosaic_{tag}.json", "origin_px", np.round(info["origin_px"], 1))
