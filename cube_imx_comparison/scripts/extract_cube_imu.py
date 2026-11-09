#!/usr/bin/env python3
"""Extract the Cube's IMU[0..2], ATT, key IMU parameters and a GPS->unix-time reference from a dataflash .BIN into an .npz.

usage: extract_cube_imu.py LOG.BIN OUT.npz
Memory: ~1 GB per 300 MB log (pymavlink DFReader). On the Jetson run at most TWO at a time.

npz keys:  ref   = [log TimeUS(s) of the first valid GPS record, unix epoch (s) of that record]   (GPS time - 18 s leap seconds)
           imu0/imu1/imu2 = rows [TimeUS(s), GyrX, GyrY, GyrZ (rad/s), AccX, AccY, AccZ (m/s^2)]   (IMU frame of each instance)
           att   = rows [TimeUS(s), Roll, Pitch, Yaw (deg)]
           parm  = JSON string of INS_ACCEL_FILTER, INS_GYRO_FILTER, INS_HNTCH_*, INS_LOG_BAT_MASK, ...
Unix time of a log sample = ref[1] + (TimeUS - ref[0]).   IMU records exist only while armed (LOG_DISARMED=0)."""
import sys, json, datetime
import numpy as np
from pymavlink import mavutil

KEEP = ("INS_ACCEL_FILTER", "INS_GYRO_FILTER", "INS_HNTCH_ENABLE", "INS_HNTCH_MODE", "INS_LOG_BAT_MASK", "INS_GYRO_RATE",
        "INS_FAST_SAMPLE", "LOG_BITMASK", "INS_USE", "INS_USE2", "INS_USE3")
src, out = sys.argv[1], sys.argv[2]
m = mavutil.mavlink_connection(src)
GPS0 = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc)
ref = None; rows = {0: [], 1: [], 2: []}; att = []; parm = {}
while True:
    x = m.recv_match(type=["GPS", "IMU", "ATT", "PARM"], blocking=False)
    if x is None:
        break
    ty = x.get_type()
    if ty == "GPS" and x.Status >= 3 and ref is None:
        ref = (x.TimeUS * 1e-6, (GPS0 + datetime.timedelta(weeks=x.GWk, milliseconds=x.GMS) - datetime.timedelta(seconds=18)).timestamp())
    elif ty == "IMU":
        rows[getattr(x, "I", 0)].append((x.TimeUS * 1e-6, x.GyrX, x.GyrY, x.GyrZ, x.AccX, x.AccY, x.AccZ))
    elif ty == "ATT":
        att.append((x.TimeUS * 1e-6, x.Roll, x.Pitch, x.Yaw))
    elif ty == "PARM":
        nm = str(x.Name).strip("\x00")
        if nm in KEEP and nm not in parm:
            parm[nm] = float(x.Value)
if ref is None:
    sys.exit("no valid GPS record in this log: cannot map log time to unix time")
np.savez(out, ref=np.array(ref), imu0=np.array(rows[0]), imu1=np.array(rows[1]), imu2=np.array(rows[2]), att=np.array(att), parm=json.dumps(parm))
print({k: len(v) for k, v in rows.items()}, "ATT", len(att), "ref", ref, "parm", parm)
