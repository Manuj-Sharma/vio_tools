#!/usr/bin/env python3
"""make_baro_bag.py -- turn the IMX-5 baro CSV into a ROS2 bag, ready to merge.

The CSV is DID_BAROMETER output (verified against inertial-sense-sdk
data_sets.h barometer_t): bar in kPa, mslBar in metres MSL, barTemp in Celsius.
Its t_ns is an ARRIVAL stamp on CLOCK_MONOTONIC -- the same clock the IMU daemon
and camera driver use -- NOT barometer_t.time, which is "seconds since boot up".
That is why the epochs already line up with the bag and no clock mapping is
needed.

Two topics are written:

  /baro/pressure      sensor_msgs/FluidPressure   PASCALS (the message is
                      defined in Pa, so kPa*1000), with variance from the
                      measured sample-to-sample noise. This is the raw
                      measurement, kept for later temperature compensation or a
                      different reference pressure.

  /baro/rel_altitude  geometry_msgs/PointStamped  z = metres above the FIRST
                      reading. Publishing relative altitude here rather than MSL
                      keeps the estimator's baro bias near zero instead of ~979,
                      so a diverging bias is obvious at a glance. The reference
                      belongs to the sensor stream, not the filter.

  make_baro_bag.py CSV OUT_BAG_DIR
"""
import sys

import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

CSV = sys.argv[1] if len(sys.argv) > 1 else "/home/vc/rosbags/field_data/seg01_baro.csv"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/vc/rosbags/field_data/baro_only"

ts = get_typestore(Stores.ROS2_HUMBLE)
FluidPressure = ts.types["sensor_msgs/msg/FluidPressure"]
PointStamped = ts.types["geometry_msgs/msg/PointStamped"]
Header = ts.types["std_msgs/msg/Header"]
Time = ts.types["builtin_interfaces/msg/Time"]
Point = ts.types["geometry_msgs/msg/Point"]

import csv as _csv
rows = list(_csv.DictReader(open(CSV)))
t_ns = np.array([int(r["t_ns"]) for r in rows])
kpa = np.array([float(r["bar_kPa"]) for r in rows])
msl = np.array([float(r["msl_m"]) for r in rows])

# measured sample-to-sample noise -> variance for the message
hp = kpa[2:-2] - np.convolve(kpa, np.ones(5) / 5, "same")[2:-2]
var_pa = (hp.std() * 1000.0) ** 2

# reference = mean of the first second, not one sample: with 0.29 m of noise a
# single reading would bake ~0.29 m of error into every subsequent value
n_ref = max(1, int(1.0 * (len(t_ns) - 1) / ((t_ns[-1] - t_ns[0]) / 1e9)))
h_ref = float(np.mean(msl[:n_ref]))

print(f"  {len(rows)} samples, {(len(t_ns)-1)/((t_ns[-1]-t_ns[0])/1e9):.2f} Hz")
print(f"  pressure {kpa.min():.4f}..{kpa.max():.4f} kPa -> {kpa.min()*1000:.1f}..{kpa.max()*1000:.1f} Pa")
print(f"  variance from measured noise: {var_pa:.3f} Pa^2  (sigma {np.sqrt(var_pa):.3f} Pa)")
print(f"  altitude reference: mean of first {n_ref} samples = {h_ref:.3f} m MSL")
print(f"  relative altitude range: {msl.min()-h_ref:+.2f} .. {msl.max()-h_ref:+.2f} m")

with Writer(OUT, version=8) as w:   # version 8 = rosbag2 metadata schema Humble reads
    c_p = w.add_connection("/baro/pressure", FluidPressure.__msgtype__, typestore=ts)
    c_a = w.add_connection("/baro/rel_altitude", PointStamped.__msgtype__, typestore=ts)
    for i, tn in enumerate(t_ns):
        hdr = Header(stamp=Time(sec=int(tn // 10**9), nanosec=int(tn % 10**9)),
                     frame_id="baro")
        mp = FluidPressure(header=hdr, fluid_pressure=float(kpa[i] * 1000.0),
                           variance=float(var_pa))
        w.write(c_p, int(tn), ts.serialize_cdr(mp, FluidPressure.__msgtype__))
        ma = PointStamped(header=hdr,
                          point=Point(x=0.0, y=0.0, z=float(msl[i] - h_ref)))
        w.write(c_a, int(tn), ts.serialize_cdr(ma, PointStamped.__msgtype__))

print(f"  -> {OUT}")
