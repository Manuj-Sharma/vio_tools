#!/usr/bin/env python3
"""Download Cube dataflash logs over MAVLink (LOG_REQUEST_DATA), resumable. usage: download_logs.py OUTDIR ID [ID ...]
Speed ~0.8 MB/s over USB (300 MB = ~6 min). .part files resume. Sends LOG_REQUEST_END first so a killed earlier run cannot block the list.
Reads only: asks for the log list and log data; changes nothing on the Cube. Needs exclusive use of the Cube's -if00 serial port."""
import sys, os, time, datetime
from pymavlink import mavutil
import glob
DEV = os.environ.get("CUBE_DEV") or (sorted(glob.glob("/dev/serial/by-id/*CubeOrange*-if00")) or sys.exit("no Cube -if00 device found"))[0]
out = sys.argv[1]; want = [int(x) for x in sys.argv[2:]]
IST = datetime.timezone(datetime.timedelta(hours=5, minutes=30))
m = mavutil.mavlink_connection(DEV, baud=115200, source_system=250, source_component=191)
if not m.wait_heartbeat(timeout=10): sys.exit("no heartbeat")
ent = {}
for attempt in range(4):
    m.mav.log_request_end_send(m.target_system, m.target_component)   # stop any transfer left over from an earlier run
    time.sleep(1.5)
    while m.recv_match(blocking=False) is not None: pass               # drain stale LOG_DATA
    m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xFFFF)
    t0 = time.time()
    while time.time() - t0 < 15:
        e = m.recv_match(type="LOG_ENTRY", blocking=True, timeout=2)
        if e is None:
            if ent: break
            continue
        ent[e.id] = e
        if len(ent) >= e.num_logs: break
    if ent: break
    print(f"no log list yet (attempt {attempt+1}/4)", flush=True)
print(f"{len(ent)} logs listed", flush=True)
for i in want:
    e = ent.get(i)
    if e is None: print(f"log {i}: not on the Cube", flush=True); continue
    t = datetime.datetime.fromtimestamp(e.time_utc, datetime.timezone.utc).astimezone(IST) if e.time_utc else None
    name = f"log_{i:03d}_{t:%Y%m%d_%H%M%S}IST.BIN" if t else f"log_{i:03d}_nogpstime.BIN"
    path = os.path.join(out, name); part = path + ".part"
    if os.path.exists(path) and os.path.getsize(path) == e.size: print(f"{name}: already complete", flush=True); continue
    nxt = os.path.getsize(part) if os.path.exists(part) else 0
    f = open(part, "ab"); last = time.time(); t_start = time.time(); start_ofs = nxt; lastprint = 0
    m.mav.log_request_data_send(m.target_system, m.target_component, i, nxt, 0xFFFFFFFF if nxt == 0 else e.size - nxt)
    while nxt < e.size:
        d = m.recv_match(type="LOG_DATA", blocking=True, timeout=1)
        if d is None or d.id != i:
            if time.time() - last > 3:     # stalled: re-request from where we are
                m.mav.log_request_data_send(m.target_system, m.target_component, i, nxt, e.size - nxt); last = time.time()
            continue
        last = time.time()
        if d.ofs != nxt: continue          # out of order: ignore, re-request on stall
        f.write(bytes(d.data[:d.count])); nxt += d.count
        if time.time() - lastprint > 15:
            lastprint = time.time(); mb = (nxt - start_ofs) / 1e6; el = time.time() - t_start
            print(f"{name}: {nxt/1e6:7.1f}/{e.size/1e6:.1f} MB  {mb/max(el,1e-3):.2f} MB/s", flush=True)
    f.close(); os.rename(part, path)
    m.mav.log_request_end_send(m.target_system, m.target_component)
    print(f"{name}: DONE {os.path.getsize(path)/1e6:.1f} MB (expected {e.size/1e6:.1f})", flush=True)
print("ALL DONE", flush=True)
