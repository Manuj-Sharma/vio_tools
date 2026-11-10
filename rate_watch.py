#!/usr/bin/env python3
"""Live /imu0 + camera rate watcher for regression tests (no SPI access, read-only ROS subscriptions).

  rate_watch.py [--interval 5] [--heartbeat 30] [--csv /tmp/vio_logs/rate_watch.csv] [--alert-only]
                [--imu-low 900] [--cam-low 40] [--gap-ms 30] [--cam-gap-ms 100]
Subscribes RAW (no deserialisation) to /imu0, /cam0/image_raw, /cam1/image_raw and reads only the header stamp from the CDR bytes.
Every --interval s it computes Hz (messages arriving in the window / window length) and the largest header-stamp gap, appends a CSV
row, and prints:
  - an ALERT line when a rate is below its threshold (imu < --imu-low, cam < --cam-low), a stamp gap > --gap-ms, or a topic has gone silent;
  - a HEARTBEAT line every --heartbeat s (mean / min of the window rates) so silence never looks like success. --alert-only drops the
    per-interval status lines (the CSV still gets every interval).
Cost: it is one more subscriber on the image topics (about the load of a recorder's subscription, without the disk writes).
Needs ROS sourced; run with python3 (NOT -I).
"""
import argparse, struct, time, csv, os, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, Image

ap = argparse.ArgumentParser()
ap.add_argument("--interval", type=float, default=5.0); ap.add_argument("--heartbeat", type=float, default=30.0)
ap.add_argument("--csv", default="/tmp/vio_logs/rate_watch.csv"); ap.add_argument("--alert-only", action="store_true")
ap.add_argument("--imu-low", type=float, default=900.0); ap.add_argument("--cam-low", type=float, default=40.0)
ap.add_argument("--gap-ms", type=float, default=30.0, help="IMU header-stamp gap alert (ms)")
ap.add_argument("--cam-gap-ms", type=float, default=100.0, help="camera header-stamp gap alert (ms); one dropped frame at ~46 Hz is ~43 ms")
a = ap.parse_args()

TOPICS = [("imu", "/imu0", Imu), ("cam0", "/cam0/image_raw", Image), ("cam1", "/cam1/image_raw", Image)]

class W(Node):
    def __init__(self):
        super().__init__("rate_watch")
        self.n = {k: 0 for k, _, _ in TOPICS}; self.last = {k: None for k, _, _ in TOPICS}; self.gap = {k: 0.0 for k, _, _ in TOPICS}
        for k, t, ty in TOPICS:
            # IMU: deep queue. The default sensor-data queue is 5 messages; at 1 kHz a Python subscriber overflows it and under-reads the
            # rate by ~1% (seen 2026-11-10: watcher 987 Hz vs the node's own 1000 Hz and `ros2 topic hz` 1000 Hz).
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5000) if k == "imu" else qos_profile_sensor_data
            self.create_subscription(ty, t, (lambda m, k=k: self.cb(k, m)), qos, raw=True)
    def cb(self, k, m):
        sec, ns = struct.unpack_from("<iI", m, 4); t = sec + ns * 1e-9
        l = self.last[k]
        if l is not None and t > l: self.gap[k] = max(self.gap[k], t - l)
        self.last[k] = t; self.n[k] += 1

rclpy.init(); w = W()
os.makedirs(os.path.dirname(a.csv), exist_ok=True)
new = not os.path.exists(a.csv); f = open(a.csv, "a"); cw = csv.writer(f)
if new: cw.writerow(["time", "imu_hz", "cam0_hz", "cam1_hz", "imu_maxgap_ms", "cam0_maxgap_ms", "cam1_maxgap_ms"])
print(f"{time.strftime('%H:%M:%S')} rate_watch started: interval {a.interval:g}s, alerts: imu<{a.imu_low:g} cam<{a.cam_low:g} imu gap>{a.gap_ms:g}ms cam gap>{a.cam_gap_ms:g}ms; csv {a.csv}", flush=True)
t_prev = time.time(); hist = []; last_hb = time.time()
while rclpy.ok():
    end = time.time() + a.interval
    while time.time() < end: rclpy.spin_once(w, timeout_sec=max(0.0, min(0.05, end - time.time())))
    now = time.time(); dt = now - t_prev; t_prev = now
    hz = {k: w.n[k] / dt for k in w.n}; gp = {k: w.gap[k] * 1e3 for k in w.gap}
    cw.writerow([time.strftime("%H:%M:%S")] + [f"{hz[k]:.1f}" for k in ("imu", "cam0", "cam1")] + [f"{gp[k]:.1f}" for k in ("imu", "cam0", "cam1")]); f.flush()
    alerts = []
    for k, _, _ in TOPICS:
        low = a.imu_low if k == "imu" else a.cam_low
        if hz[k] == 0: alerts.append(f"{k} SILENT (no messages in {dt:.0f}s)")
        elif hz[k] < low: alerts.append(f"{k} LOW {hz[k]:.1f} Hz")
        if gp[k] > (a.gap_ms if k == "imu" else a.cam_gap_ms): alerts.append(f"{k} gap {gp[k]:.0f} ms")
    stamp = time.strftime("%H:%M:%S")
    line = f"{stamp} imu {hz['imu']:7.1f} Hz (gap {gp['imu']:4.0f} ms) | cam0 {hz['cam0']:5.1f} (gap {gp['cam0']:3.0f}) | cam1 {hz['cam1']:5.1f} (gap {gp['cam1']:3.0f})"
    hist.append(hz)
    if alerts: print("ALERT " + line + "  <-- " + "; ".join(alerts), flush=True)
    elif not a.alert_only: print(line, flush=True)
    if now - last_hb >= a.heartbeat and hist:
        n = len(hist); mean = {k: sum(h[k] for h in hist) / n for k in ("imu", "cam0", "cam1")}; mn = {k: min(h[k] for h in hist) for k in ("imu", "cam0", "cam1")}
        print(f"{stamp} HEARTBEAT last {n*a.interval:.0f}s: imu mean {mean['imu']:.1f} min {mn['imu']:.1f} | cam0 mean {mean['cam0']:.1f} min {mn['cam0']:.1f} | cam1 mean {mean['cam1']:.1f} min {mn['cam1']:.1f}", flush=True)
        hist = []; last_hb = now
    for k in w.n: w.n[k] = 0; w.gap[k] = 0.0
