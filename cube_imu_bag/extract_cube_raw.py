#!/usr/bin/env python3
"""Extract the Cube's RAW-rate ACC/GYR (instance 0 by default) + GPS->unix reference from a dataflash .BIN into an .npz.
usage: extract_cube_raw.py LOG.BIN OUT.npz [instance]
npz: ref=[log s, unix s] (same as extract_cube_imu.py); acc = rows [SampleUS(s), AccX,AccY,AccZ]; gyr = rows [SampleUS(s), GyrX,GyrY,GyrZ]
Unix time of a sample = ref[1] + (SampleUS - ref[0]).  Raw = before ArduPilot's software LPF/notch (chip DLPF still applies, rate is decimated)."""
import sys, datetime, numpy as np
from pymavlink import mavutil
src, out = sys.argv[1], sys.argv[2]; inst = int(sys.argv[3]) if len(sys.argv) > 3 else 0
m = mavutil.mavlink_connection(src); GPS0 = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc)
ref = None; A = []; G = []
while True:
    x = m.recv_match(type=["GPS", "ACC", "GYR"], blocking=False)
    if x is None: break
    ty = x.get_type()
    if ty == "GPS" and x.Status >= 3 and ref is None:
        ref = (x.TimeUS * 1e-6, (GPS0 + datetime.timedelta(weeks=x.GWk, milliseconds=x.GMS) - datetime.timedelta(seconds=18)).timestamp())
    elif ty == "ACC" and x.I == inst: A.append((x.SampleUS * 1e-6, x.AccX, x.AccY, x.AccZ))
    elif ty == "GYR" and x.I == inst: G.append((x.SampleUS * 1e-6, x.GyrX, x.GyrY, x.GyrZ))
if ref is None: sys.exit("no GPS ref")
np.savez(out, ref=np.array(ref), acc=np.array(A), gyr=np.array(G)); print(out, len(A), len(G), ref)
