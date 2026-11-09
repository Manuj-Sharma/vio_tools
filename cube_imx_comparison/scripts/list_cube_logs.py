#!/usr/bin/env python3
"""List the dataflash logs stored on a connected Cube (ArduPilot) over MAVLink. READ-ONLY (LOG_REQUEST_LIST only).

usage: list_cube_logs.py            (auto-detects /dev/serial/by-id/*CubeOrange*-if00; override with CUBE_DEV=...)

The Cube's serial port must not be in use by anything else (FC node, preflight tool, another download).
NOTE: the time printed is the Cube's `time_utc` for the log. It is NOT a reliable start or end time (log 29 was listed
at 17:45 IST but is the flight recorded at 16:55 IST). Match logs to bags with match_bag_to_log.py instead.
1980-01-01 means the log was written without a GPS time."""
import os, sys, glob, time, datetime
from pymavlink import mavutil

dev = os.environ.get("CUBE_DEV") or (sorted(glob.glob("/dev/serial/by-id/*CubeOrange*-if00")) or sys.exit("no Cube -if00 device found"))[0]
m = mavutil.mavlink_connection(dev, baud=115200, source_system=250, source_component=191)
hb = m.wait_heartbeat(timeout=10)
if not hb:
    sys.exit("NO HEARTBEAT in 10 s")
print(f"{dev}\nheartbeat sys {m.target_system} comp {m.target_component}, armed={bool(hb.base_mode & 128)}")
ent = {}
for attempt in range(4):
    m.mav.log_request_end_send(m.target_system, m.target_component)      # stop any transfer left over from an earlier run
    time.sleep(1.5)
    while m.recv_match(blocking=False) is not None:
        pass
    m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xFFFF)
    t0 = time.time()
    while time.time() - t0 < 15:
        e = m.recv_match(type="LOG_ENTRY", blocking=True, timeout=2)
        if e is None:
            if ent:
                break
            continue
        ent[e.id] = e
        if len(ent) >= e.num_logs:
            break
    if ent:
        break
IST = datetime.timezone(datetime.timedelta(hours=5, minutes=30))
print(f"{len(ent)} logs")
for i in sorted(ent):
    e = ent[i]
    t = datetime.datetime.fromtimestamp(e.time_utc, datetime.timezone.utc) if e.time_utc else None
    print(f"  log {i:3d}  {e.size/1e6:8.2f} MB  " + (f"{t:%Y-%m-%d %H:%M:%S} UTC = {t.astimezone(IST):%H:%M:%S} IST" if t else "no timestamp"))
