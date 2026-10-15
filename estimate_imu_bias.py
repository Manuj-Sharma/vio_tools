#!/usr/bin/env python3
"""estimate_imu_bias.py BAG [-o out.png] -- gyro/accel bias off a static bench log.

    python3 estimate_imu_bias.py imu_data
    python3 estimate_imu_bias.py imu_data --t0 5 --t1 660       # manual trim
    python3 estimate_imu_bias.py imu_data --lat 12.97 --alt 920 # non-default site

Needs the IMU STATIONARY, in its mounted orientation, thermally settled -- same
requirement as allan_variance.py. Unlike allan_variance.py (which gives noise
DENSITY and bias INSTABILITY -- how much the bias wanders), this gives the bias
VALUE itself: the mean offset to seed init_dyn_bias_g / init_dyn_bias_a with.

GYRO is simple: no gravity component at rest, so bias_g = mean(gyro) directly.

ACCEL is not: a stationary accelerometer reads the reaction force holding it up
against gravity, so raw_mean = bias + g_local on whichever axis is vertical.
This script assumes Z is that axis (rig convention here) and subtracts a
LOCAL gravity computed from latitude + altitude via the IGF80 formula, not the
generic 9.81 -- at 12.97N that generic value overstates gravity by ~0.03 m/s^2,
which is the same order of magnitude as the bias itself. X/Y get no correction,
which only holds if the bench orientation has them genuinely horizontal; any
residual tilt leaks into their reported "bias" indistinguishably. A 6-position
tumble test (each axis up/down, average opposite pairs to cancel gravity
algebraically) removes this assumption if you need axis-independent numbers.

AUTO-TRIM: bench logs almost always spike at the very start/end (letting go of
/ picking up the sensor, or ros2 bag record startup). This script flags any
window where |a| deviates from the session median by more than --spike-thresh
and reports it; pass --t0/--t1 yourself once you've seen where they are, or
rely on --auto-trim to cut the first/last such window automatically.
"""
import argparse

import numpy as np

# International Gravity Formula 1980 (GRS80 ellipsoid), + first-order
# free-air altitude correction. Standard reference formula; not rig-specific.
def local_gravity(lat_deg: float, alt_m: float) -> float:
    phi = np.radians(lat_deg)
    g0 = 9.780327 * (1 + 0.0053024 * np.sin(phi) ** 2 - 0.0000058 * np.sin(2 * phi) ** 2)
    return g0 - 3.086e-6 * alt_m


def read_imu(bag, topic="/imu0"):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    t, g, a = [], [], []
    with Reader(bag) as r:
        cons = [c for c in r.connections if c.topic == topic]
        if not cons:
            raise SystemExit(f"no {topic} in {bag}; topics: {[c.topic for c in r.connections]}")
        for _, _s, raw in r.messages(connections=cons):
            m = ts.deserialize_cdr(raw, cons[0].msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            g.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
            a.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    return np.array(t), np.array(g), np.array(a)


def find_spikes(t, a, thresh):
    """Contiguous windows where |a| deviates from the session median by > thresh."""
    mag = np.linalg.norm(a, axis=1)
    dev = np.abs(mag - np.median(mag))
    bad = np.flatnonzero(dev > thresh)
    if len(bad) == 0:
        return []
    splits = np.flatnonzero(np.diff(bad) > 25)  # new window if gap > ~0.1s at 250Hz
    starts = [bad[0]] + list(bad[splits + 1])
    ends = list(bad[splits]) + [bad[-1]]
    return [(t[s], t[e], dev[s:e + 1].max()) for s, e in zip(starts, ends)]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/imu0")
    ap.add_argument("--t0", type=float, help="window start [s from bag start]")
    ap.add_argument("--t1", type=float, help="window end [s from bag start]")
    ap.add_argument("--auto-trim", action="store_true",
                    help="cut the first/last detected spike window automatically")
    ap.add_argument("--spike-thresh", type=float, default=0.1,
                    help="|a| deviation from median [m/s^2] to flag as a spike (default 0.1)")
    ap.add_argument("--lat", type=float, default=12.97, help="latitude, deg (default: Bangalore)")
    ap.add_argument("--alt", type=float, default=920.0, help="altitude, m MSL (default: Bangalore)")
    ap.add_argument("--gravity", type=float, help="override computed local gravity [m/s^2]")
    args = ap.parse_args()

    t, g, a = read_imu(args.bag, args.topic)
    t = t - t[0]
    dt = float(np.median(np.diff(t)))
    print(f"  {len(t)} samples over {t[-1]:.1f} s at ~{1/dt:.1f} Hz nominal\n")

    spikes = find_spikes(t, a, args.spike_thresh)
    if spikes:
        print(f"  spike windows (|a| dev > {args.spike_thresh} m/s^2 from median):")
        for s0, s1, mx in spikes:
            print(f"    {s0:7.1f}s - {s1:7.1f}s   (max dev {mx:.3f})")
        print()

    lo, hi = args.t0, args.t1
    if args.auto_trim and spikes:
        lo = spikes[0][1] if lo is None else lo
        hi = spikes[-1][0] if hi is None else hi
        print(f"  --auto-trim: using {lo:.1f}s - {hi:.1f}s\n")
    lo = t[0] if lo is None else lo
    hi = t[-1] if hi is None else hi

    m = (t >= lo) & (t <= hi)
    if m.sum() < 2:
        raise SystemExit(f"window {lo}-{hi}s selects {m.sum()} samples of {len(t)}")
    gw, aw = g[m], a[m]
    print(f"  using {lo:.2f}-{hi:.2f}s ({m.sum()} samples) for bias estimate\n")

    g_local = args.gravity if args.gravity is not None else local_gravity(args.lat, args.alt)
    if args.gravity is None:
        print(f"  local gravity @ lat {args.lat}, alt {args.alt}m (IGF80 + free-air): "
              f"{g_local:.4f} m/s^2  (vs generic 9.81, delta {g_local - 9.81:+.4f})\n")

    bias_g = gw.mean(axis=0)
    bias_a = aw.mean(axis=0)
    bias_a_corrected = bias_a.copy()
    bias_a_corrected[2] -= g_local  # assumes Z is the vertical axis in this orientation

    print(f"  {'channel':9s} {'mean':>12s} {'std':>12s}")
    for nm, x in zip(["gyro x", "gyro y", "gyro z"], gw.T):
        print(f"  {nm:9s} {x.mean():12.5f} {x.std():12.5f}")
    for nm, x in zip(["accel x", "accel y", "accel z"], aw.T):
        print(f"  {nm:9s} {x.mean():12.5f} {x.std():12.5f}")
    print("\n  gyro rad/s, accel m/s^2")

    print("\n  NOTE: accel x/y assumed horizontal (no gravity component) in this orientation --")
    print("        any residual bench tilt is indistinguishable from real bias on those axes.")
    print("        accel z has local gravity subtracted below.\n")

    print("  -- estimator_config.yaml --")
    print(f"  init_dyn_bias_g: [ {bias_g[0]:.5f}, {bias_g[1]:.5f}, {bias_g[2]:.5f} ]")
    print(f"  init_dyn_bias_a: [ {bias_a_corrected[0]:.5f}, {bias_a_corrected[1]:.5f}, "
          f"{bias_a_corrected[2]:.5f} ]")
    print(f"  gravity_mag: {g_local:.4f}")


if __name__ == "__main__":
    main()
