#!/usr/bin/env python3
"""gravity_rotation.py <tag> <t_start_s> <t_end_s> [out.npy]
Cube->IMX rotation with roll/pitch from GRAVITY and yaw from the gyro fit. Run in this folder after align.py (needs align_<tag>.npz,
cube_<tag>.npz, imx_<tag>.npz). No ROS needed.
  t_start/t_end : bag seconds (from the first /imu0 message) of a window where the vehicle is STANDING STILL and both IMUs have data
                  (the Cube log may only start at arming: use the gap between arming and motor spool-up, e.g. 22.5 23.9 on bag 110545).
Method: mean accel of both IMUs over the quiet 1-s blocks in the window = gravity in each frame. The gyro-only rotation from align.py is
kept, then corrected by the smallest rotation that makes the Cube gravity direction land on the IMX gravity direction. That correction
has no component about gravity, so yaw stays as the gyro found it.
Output: R (3x3, v_imx = R @ v_cube) -> out.npy (default R_cube_to_imx_gravity_gyro_<tag>.npy). Use it with CUBE_R=<file> make_cube_bag.py ..."""
import sys, numpy as np
tag, t1, t2 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]); out = sys.argv[4] if len(sys.argv) > 4 else 'R_cube_to_imx_gravity_gyro_%s.npy' % tag
def rv(M):   # rotation vector in deg
    a = np.arccos(np.clip((np.trace(M)-1)/2, -1, 1)); v = np.array([M[2,1]-M[1,2], M[0,2]-M[2,0], M[1,0]-M[0,1]])/(2*np.sin(a)+1e-12); return np.degrees(a*v)
def rot_between(a, b):   # smallest rotation taking unit a to unit b
    v = np.cross(a, b); s = np.linalg.norm(v); c = a@b; K = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]); return np.eye(3)+K+K@K*((1-c)/(s*s+1e-18))
Rg = np.load('align_%s.npz' % tag)['R']; C = np.load('cube_%s.npz' % tag); Z = np.load('imx_%s.npz' % tag); I = Z['I']; CLK = float(Z['CLK']); t0 = I[0,0]
tc = C['ta']-t0; th = I[:,1]+CLK-t0; blocks = []
for s in np.arange(t1, t2-1+1e-9, 0.5):
    mc = (tc >= s)&(tc < s+1); mi = (th >= s)&(th < s+1)
    if mc.sum() > 500 and mi.sum() > 100: blocks.append((np.linalg.norm(C['a'][mc].std(0)), np.linalg.norm(I[mi,5:8].std(0)), C['a'][mc].mean(0), I[mi,5:8].mean(0)))
q = [b for b in blocks if b[0] < 0.08 and b[1] < 0.12]   # quiet in both IMUs
if len(q) < 1: sys.exit('only %d quiet 1-s blocks of %d in %.1f-%.1f s: pick a window where the vehicle is still and both IMUs have data' % (len(q), len(blocks), t1, t2))
if len(q) < 3: print('WARNING: only %d quiet block(s); a longer still period gives a more reliable roll/pitch' % len(q))
gc = np.mean([b[2] for b in q], axis=0); gi = np.mean([b[3] for b in q], axis=0); uc = gc/np.linalg.norm(gc); ui = gi/np.linalg.norm(gi)
pred = Rg@uc; err = np.degrees(np.arccos(np.clip(pred@ui, -1, 1))); Rc = rot_between(pred, ui)@Rg
print('%s: %d quiet 1-s blocks in %.1f-%.1f s | Cube gravity %s (|g| %.4f) | IMX gravity %s (|g| %.4f)' % (tag, len(q), t1, t2, gc.round(4), np.linalg.norm(gc), gi.round(4), np.linalg.norm(gi)))
print('gyro-only rotation (deg x y z) %s | gravity mismatch left by it %.2f deg | gravity+gyro rotation %s' % (rv(Rg).round(2), err, rv(Rc).round(2)))
np.save(out, Rc); print('saved', out)
