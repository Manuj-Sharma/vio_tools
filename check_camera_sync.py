#!/usr/bin/env python3
"""check_camera_sync.py [-d SEC] -- is a stereo pair actually hardware-synced?

    python3 check_camera_sync.py
    python3 check_camera_sync.py --cam0 /cam0/image_raw --cam1 /cam1/image_raw -d 10
    python3 check_camera_sync.py --trigger-hz 60 --thresh-ms 3

Subscribes LIVE to both camera topics (no bag mode -- sync is a property of the
running capture pipeline, not something a recorded bag can tell you anything
new about) and, for the given duration:

  1. per-camera frame period (mean/std) -- is each camera's own rate stable?
  2. cross-camera offset -- for each cam0 frame, nearest cam1 frame by
     header.stamp, report the distribution of (cam1_t - cam0_t)
  3. (if --trigger-hz given) whether the per-camera frame period lands near an
     integer multiple of the trigger period -- a camera genuinely locked to an
     external trigger should sit on 1x/2x/3x/... of that period, not drift

WHAT GOOD LOOKS LIKE: genuine hardware trigger sync means both cameras' shutter
opens on the same electrical edge, so the cross-camera offset should be tight
and centered near zero -- sub-millisecond in principle, a few ms in practice
once you account for independent readout/USB/publish latency per camera. A
wide spread (offset std more than ~30% of one frame period) means something
downstream of capture -- buffer readout, format conversion, ROS publish
scheduling -- is adding jitter independently per camera, even if the capture
trigger itself is solid. This script can't tell you WHICH stage; it can only
tell you whether the jitter is small enough not to matter for VIO.

CAVEAT: this measures PUBLISH timestamps (header.stamp, whatever the driver
stamped -- V4L2 buffer time if use_v4l2_timestamp is on), not true photon
capture time. If both cameras stamp the same way this is still a fair
cross-camera comparison; it is not a measurement of absolute trigger latency.

A near-zero-frame-count run (either camera) most likely means the hardware
trigger is off, or the camera node is stuck/not streaming -- see
~/vio_ar0234_icm20948/README.md and trigger.sh; this script does not start or
stop the trigger, only observes.
"""
import argparse
import sys
import time

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam0", default="/cam0/image_raw")
    ap.add_argument("--cam1", default="/cam1/image_raw")
    ap.add_argument("-d", "--duration", type=float, default=6.0, help="capture window, seconds")
    ap.add_argument("--warmup", type=float, default=2.0,
                    help="seconds to discard at the start -- first frames after subscribing can be "
                         "a queued burst (QoS depth) rather than live-rate frames, and skews dt/rate")
    ap.add_argument("--trigger-hz", type=float, default=None,
                    help="external trigger rate, to check frame period is a clean multiple of it")
    ap.add_argument("--thresh-ms", type=float, default=5.0,
                    help="cross-camera offset std [ms] above which sync is flagged as poor")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    class Probe(Node):
        def __init__(self):
            super().__init__("check_camera_sync")
            self.stamps = {"cam0": [], "cam1": []}
            self.create_subscription(Image, args.cam0, lambda m: self.cb("cam0", m), 10)
            self.create_subscription(Image, args.cam1, lambda m: self.cb("cam1", m), 10)

        def cb(self, name, msg):
            self.stamps[name].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    rclpy.init()
    node = Probe()
    total = args.warmup + args.duration
    print(f"  watching {args.cam0} / {args.cam1} for {total:.1f}s "
          f"({args.warmup:.1f}s warmup discarded + {args.duration:.1f}s measured) ...")
    t_start = time.time()
    t_end = t_start + total
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    # Discard by WALL-CLOCK warmup window, not by dropping the first N samples -- rate
    # differs per camera so a fixed sample count would drop uneven amounts of real time.
    cutoff = t_start + args.warmup
    c0_all = np.array(node.stamps["cam0"])
    c1_all = np.array(node.stamps["cam1"])
    # stamps are sensor-clock (monotonic from the driver), not wall-clock -- use arrival
    # order/count proportional to warmup fraction as the cut instead, since we don't have
    # a wall-clock/sensor-clock mapping here. Simplest robust cut: drop the first
    # warmup/total fraction of EACH stream by its own timestamp span.
    def apply_warmup(t):
        if len(t) < 2:
            return t
        span0 = t[0]
        cut = span0 + args.warmup
        return t[t >= cut]

    c0 = apply_warmup(c0_all)
    c1 = apply_warmup(c1_all)

    print(f"\n  {args.cam0}: {len(c0_all)} frames captured, {len(c0)} after warmup discard")
    print(f"  {args.cam1}: {len(c1_all)} frames captured, {len(c1)} after warmup discard\n")

    if len(c0) == 0 or len(c1) == 0:
        print("  FAIL: no frames on at least one topic -- camera not streaming or trigger is off.")
        print("        this is not a sync problem, it's a no-data problem. check the node/trigger first.")
        sys.exit(1)

    def rate_stats(name, t):
        if len(t) < 2:
            print(f"  {name}: only {len(t)} frame(s), can't compute rate")
            return None
        dt = np.diff(t)
        mean_ms, std_ms = dt.mean() * 1000, dt.std() * 1000
        print(f"  {name} dt: mean {mean_ms:6.1f}ms  std {std_ms:5.1f}ms  (rate {1/dt.mean():5.1f} Hz)")
        if args.trigger_hz:
            trig_period_ms = 1000.0 / args.trigger_hz
            n = mean_ms / trig_period_ms
            nearest = round(n)
            err_pct = 100 * abs(n - nearest) / nearest if nearest else float("inf")
            flag = "" if err_pct < 5 else "  <-- NOT a clean multiple of the trigger period"
            print(f"       {mean_ms:.1f}ms / {trig_period_ms:.2f}ms trigger period = {n:.2f}x{flag}")
        return dt

    d0 = rate_stats(args.cam0, c0)
    d1 = rate_stats(args.cam1, c1)
    print()

    # nearest-neighbor cross-camera offset: for each cam0 stamp, closest cam1 stamp
    offsets = np.array([c1[np.argmin(np.abs(c1 - t))] - t for t in c0])
    mean_ms, std_ms = offsets.mean() * 1000, offsets.std() * 1000
    min_ms, max_ms = offsets.min() * 1000, offsets.max() * 1000
    print(f"  cross-camera offset (cam1 - cam0, nearest match):")
    print(f"    mean {mean_ms:+7.2f}ms   std {std_ms:6.2f}ms   min {min_ms:+7.2f}ms   max {max_ms:+7.2f}ms")

    print()
    if std_ms <= args.thresh_ms:
        print(f"  PASS: offset std {std_ms:.2f}ms <= threshold {args.thresh_ms:.1f}ms -- looks genuinely synced.")
    else:
        frame_ms = (d0.mean() * 1000) if d0 is not None else float("nan")
        pct = 100 * std_ms / frame_ms if frame_ms == frame_ms else float("nan")
        print(f"  FAIL: offset std {std_ms:.2f}ms > threshold {args.thresh_ms:.1f}ms "
              f"({pct:.0f}% of one frame period).")
        print("        either the trigger isn't actually gating both cameras, or jitter is being")
        print("        added downstream of capture (readout/conversion/ROS publish scheduling).")
        print("        cut the trigger (trigger.sh off) and re-run: 0 frames on both = trigger really")
        print("        is required; frames keep flowing = at least one camera is free-running.")


if __name__ == "__main__":
    main()
