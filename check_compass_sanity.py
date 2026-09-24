#!/usr/bin/env python3
"""check_compass_sanity.py -- live compass sanity monitor, run directly in a
terminal (Konsole), Ctrl+C to stop.

Two independent checks, either one alone can catch a different failure mode:

1. RAW MAGNETIC FIELD MAGNITUDE (sqrt(xmag^2+ymag^2+zmag^2) from SCALED_IMU).
   Earth's field magnitude is roughly constant regardless of vehicle
   orientation -- a baseline is established from the first ~5s of readings,
   then any later reading that deviates more than +-MAG_DEVIATION_PCT from
   that baseline is flagged. This is the most direct, orientation-independent
   interference check (e.g. a parked/moving car -- confirmed the real cause of
   a wild /fc/ekf_yaw oscillation on 2026-09-24).

2. COMPASS-DERIVED YAW RATE vs GYRO-DERIVED YAW RATE. Raw heading
   (atan2(-ymag, xmag), same formula ArduPilot's own Compass::calculate_heading
   uses) is unwrapped and differentiated over a short smoothing window, then
   compared against the raw gyro's zgyro. A real rotation should make BOTH
   agree; a compass being corrupted by interference (without the gyro also
   being affected) shows up as a sustained disagreement between the two rates
   even though the compass alone might look like plausible smooth motion.

Reads directly from /dev/ttyACM0 -- requires nothing else (fc node, other
pymavlink scripts) to be connected to the port at the same time, or it'll
steal bytes from whichever one loses (no OS exclusive lock on this port).

AUTO-DETECTS which compass instance is actually feeding the EKF by querying
COMPASS_USE/COMPASS_USE2/COMPASS_USE3 fresh at startup, every run -- does NOT
hardcode instance 0, because slot assignment has been confirmed unstable
across reboots on this Cube (see compass-investigation memory) and the active
instance itself has been deliberately switched at least once (2026-09-23,
590114 <-> 592905). Pass --instance to override if you ever need to watch a
specific instance regardless of which one is USE=1.

Usage:
    python3 ~/vio_tools/check_compass_sanity.py
    python3 ~/vio_tools/check_compass_sanity.py --mag-deviation-pct 15 --rate-disagree-thresh 25
    python3 ~/vio_tools/check_compass_sanity.py --instance 1   # force SCALED_IMU2 regardless of COMPASS_USE2
"""
import argparse
import math
import sys
import time
from collections import deque

from pymavlink import mavutil

MAG_MSG_BY_INSTANCE = {
    0: ("SCALED_IMU", mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU),
    1: ("SCALED_IMU2", mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2),
    2: ("SCALED_IMU3", mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3),
}


def detect_active_compass_instance(m, timeout=8):
    """Query COMPASS_USE/COMPASS_USE2/COMPASS_USE3 fresh and return the first
    instance with USE=1, plus its dev_id for a clear log line. None if none
    (or more than one, ambiguous) come back set -- caller should fall back to
    --instance or refuse rather than guess."""
    params = ["COMPASS_USE", "COMPASS_USE2", "COMPASS_USE3",
              "COMPASS_DEV_ID", "COMPASS_DEV_ID2", "COMPASS_DEV_ID3"]
    for p in params:
        m.mav.param_request_read_send(m.target_system, m.target_component, p.encode(), -1)

    results = {}
    t0 = time.time()
    while time.time() - t0 < timeout and len(results) < len(params):
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg is None:
            continue
        pid = msg.param_id.strip("\x00") if isinstance(msg.param_id, str) else msg.param_id
        if pid in params:
            results[pid] = msg.param_value

    use = {0: results.get("COMPASS_USE"), 1: results.get("COMPASS_USE2"), 2: results.get("COMPASS_USE3")}
    dev_id = {0: results.get("COMPASS_DEV_ID"), 1: results.get("COMPASS_DEV_ID2"), 2: results.get("COMPASS_DEV_ID3")}
    active = [i for i, v in use.items() if v == 1.0]

    print(f"COMPASS_USE/USE2/USE3 = {use.get(0)}/{use.get(1)}/{use.get(2)}  "
          f"(dev_ids: {dev_id.get(0)}/{dev_id.get(1)}/{dev_id.get(2)})")

    if len(active) == 1:
        i = active[0]
        print(f"-> auto-detected active instance {i} (dev_id {dev_id.get(i)})")
        return i
    if len(active) == 0:
        print("-> WARNING: no compass instance has USE=1 -- nothing is feeding the EKF?")
    else:
        print(f"-> WARNING: multiple instances have USE=1 ({active}) -- ambiguous")
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--instance", type=int, choices=[0, 1, 2], default=None,
                     help="force a specific compass instance instead of auto-detecting via COMPASS_USE*")
    ap.add_argument("--mag-deviation-pct", type=float, default=15.0,
                     help="flag when raw field magnitude deviates more than this %% from baseline (default: 15)")
    ap.add_argument("--rate-disagree-thresh", type=float, default=20.0,
                     help="flag when |compass_rate - gyro_rate| exceeds this many deg/s, sustained (default: 20)")
    ap.add_argument("--baseline-window-s", type=float, default=5.0,
                     help="seconds of initial readings used to establish the field-magnitude baseline (default: 5)")
    ap.add_argument("--smoothing-window-s", type=float, default=0.5,
                     help="window used to compute compass/gyro rates (default: 0.5)")
    args = ap.parse_args()

    print(f"connecting to {args.port} ...")
    m = mavutil.mavlink_connection(args.port, baud=args.baud)
    m.wait_heartbeat(timeout=10)

    instance = args.instance
    if instance is None:
        # First-connection param sync occasionally drops one reply (observed live,
        # 2026-09-24) -- one retry before giving up.
        instance = detect_active_compass_instance(m)
        if instance is None:
            print("retrying auto-detection once...")
            instance = detect_active_compass_instance(m)
        if instance is None:
            sys.exit("could not auto-detect active compass instance after retry -- pass --instance explicitly")
    else:
        print(f"using --instance {instance} (auto-detection skipped)")

    msg_name, msg_id = MAG_MSG_BY_INSTANCE[instance]
    print(f"connected. streaming {msg_name} at 20Hz. Ctrl+C to stop.\n")

    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, 50000, 0, 0, 0, 0, 0)  # 20Hz

    baseline_samples = []
    baseline_norm = None
    heading_hist = deque()  # (t, unwrapped_heading_rad)
    last_unwrapped = None
    t0 = time.time()

    anomaly_active = {"mag": False, "rate": False}

    try:
        while True:
            msg = m.recv_match(type=msg_name, blocking=True, timeout=2)
            if msg is None:
                print(f"... no {msg_name} message received (link stalled?)")
                continue
            t = time.time() - t0
            xmag, ymag, zmag = msg.xmag, msg.ymag, msg.zmag
            zgyro_deg_s = msg.zgyro / 1000.0 * 180.0 / math.pi  # mrad/s -> deg/s

            norm = math.sqrt(xmag ** 2 + ymag ** 2 + zmag ** 2)

            if baseline_norm is None:
                baseline_samples.append(norm)
                if t >= args.baseline_window_s:
                    baseline_norm = sorted(baseline_samples)[len(baseline_samples) // 2]  # median
                    print(f"\n[baseline established] field magnitude = {baseline_norm:.1f} mGauss "
                          f"(from {len(baseline_samples)} samples over {args.baseline_window_s}s)\n")
                mag_flag = False
                mag_dev_pct = 0.0
            else:
                mag_dev_pct = 100.0 * (norm - baseline_norm) / baseline_norm
                mag_flag = abs(mag_dev_pct) > args.mag_deviation_pct

            raw_heading = math.atan2(-ymag, xmag)
            if last_unwrapped is None:
                last_unwrapped = raw_heading
            else:
                d = raw_heading - (last_unwrapped % (2 * math.pi))
                while d > math.pi:
                    d -= 2 * math.pi
                while d < -math.pi:
                    d += 2 * math.pi
                last_unwrapped = last_unwrapped + d

            heading_hist.append((t, last_unwrapped))
            while heading_hist and t - heading_hist[0][0] > args.smoothing_window_s:
                heading_hist.popleft()

            compass_rate_deg_s = 0.0
            rate_flag = False
            if len(heading_hist) >= 2:
                dt = heading_hist[-1][0] - heading_hist[0][0]
                if dt > 0.05:
                    dh = heading_hist[-1][1] - heading_hist[0][1]
                    compass_rate_deg_s = math.degrees(dh) / dt
                    rate_disagree = abs(compass_rate_deg_s - zgyro_deg_s)
                    rate_flag = rate_disagree > args.rate_disagree_thresh

            flags = []
            if mag_flag:
                flags.append(f"MAG-ANOMALY ({mag_dev_pct:+.1f}% from baseline)")
            if rate_flag:
                flags.append(f"RATE-DISAGREE (compass={compass_rate_deg_s:+.1f} gyro={zgyro_deg_s:+.1f} deg/s)")

            status = " ".join(flags) if flags else "ok"
            sys.stdout.write(
                f"\rt={t:6.1f}s  |field|={norm:7.1f}mG ({mag_dev_pct:+5.1f}%)  "
                f"compass_rate={compass_rate_deg_s:+7.1f}  gyro_rate={zgyro_deg_s:+7.1f} deg/s  "
                f"[{status}]"
                + " " * 10
            )
            sys.stdout.flush()

            if mag_flag and not anomaly_active["mag"]:
                print(f"\n*** MAGNETIC INTERFERENCE STARTED at t={t:.1f}s -- "
                      f"field {norm:.1f}mG, {mag_dev_pct:+.1f}% from baseline ***")
            if not mag_flag and anomaly_active["mag"]:
                print(f"\n*** magnetic interference ended at t={t:.1f}s ***")
            anomaly_active["mag"] = mag_flag

            if rate_flag and not anomaly_active["rate"]:
                print(f"\n*** COMPASS/GYRO RATE DISAGREEMENT STARTED at t={t:.1f}s -- "
                      f"compass={compass_rate_deg_s:+.1f} gyro={zgyro_deg_s:+.1f} deg/s ***")
            if not rate_flag and anomaly_active["rate"]:
                print(f"\n*** rate disagreement ended at t={t:.1f}s ***")
            anomaly_active["rate"] = rate_flag

    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()
