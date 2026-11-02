#!/usr/bin/env python3
"""imu_resample.py IN_BAG OUT_BAG [--hz 200] [--method avg|interp|nearest] [--topic /imu0]
                    [--max-gap-ms 50] [--version 8|9]

Writes OUT_BAG = IN_BAG with the IMU topic resampled to a uniform --hz grid. Every other topic is copied
byte for byte with its original bag timestamp (no deserialising, so a 12 GB camera bag is just streamed).

The IMU stamps in a bag are NOT on a regular grid (the driver polls, so ~86-92% of gaps are 3 ms and the
rest are 1-2 ms or 6+ ms), so resampling is done on the header stamps, never by "every Nth message".

  avg      (default) box-average over each 1/HZ bin of the piecewise-linear signal between the original samples
           (exact integral, so gaps are handled correctly), output stamp = bin centre. This is an anti-alias
           filter, so the white-noise density seen by Kalibr / an Allan fit is preserved. Per-sample noise is
           LOWER than the raw bag's (several raw samples averaged), which is right for the lower rate.
  interp   linear interpolation of the signal at the grid times. Uniform stamps, no anti-aliasing.
  nearest  the raw sample closest to each grid time (within half a bin). Real readings, but +-1.5 ms timing
           jitter, and un-filtered decimation aliases noise: a noise density estimated from the output comes
           out ~sqrt(old_rate/new_rate) too high.

Grid bins that fall inside an original gap longer than --max-gap-ms are skipped (not invented).
The bag timestamp of each new IMU message = its header stamp + the median (receive time - header stamp)
of the original, so it sorts against the camera messages like the original did.
Remember to set update_rate in imu.yaml to the new --hz for Kalibr.

Needs the `rosbags` python package (the one the other vio_tools scripts use). Check the result with
`ros2 bag info OUT_BAG`. rosbags writes bag metadata version 8 or 9 (--version); if your ros2 refuses 9, use 8.
"""
import argparse
import os
import sys

import numpy as np
import yaml
from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS2_HUMBLE)
FIELDS = ("angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
          "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z")


def qos_str(profiles):
    """Humble writes offered_qos_profiles into metadata.yaml as one YAML *string* (a list inside a string)."""
    out = []
    for q in profiles:
        out.append(f"- history: {q.history.value}\n  depth: {q.depth}\n  reliability: {q.reliability.value}\n"
                   f"  durability: {q.durability.value}\n  deadline:\n    sec: {q.deadline.sec}\n    nsec: {q.deadline.nsec}\n"
                   f"  lifespan:\n    sec: {q.lifespan.sec}\n    nsec: {q.lifespan.nsec}\n  liveliness: {q.liveliness.value}\n"
                   f"  liveliness_lease_duration:\n    sec: {q.liveliness_lease_duration.sec}\n"
                   f"    nsec: {q.liveliness_lease_duration.nsec}\n"
                   f"  avoid_ros_namespace_conventions: {str(q.avoid_ros_namespace_conventions).lower()}")
    return "\n".join(out)


def humble_metadata(out_bag, conns):
    """rosbags writes metadata v8/9 (QoS as a YAML list), which Humble's ros2 bag cannot parse. Rewrite it as v5."""
    path = os.path.join(out_bag, "metadata.yaml")
    m = yaml.safe_load(open(path))["rosbag2_bagfile_information"]
    qos = {c.topic: qos_str(c.ext.offered_qos_profiles) for c in conns}
    topics = [{"topic_metadata": {"name": t["topic_metadata"]["name"], "type": t["topic_metadata"]["type"],
                                  "serialization_format": t["topic_metadata"]["serialization_format"],
                                  "offered_qos_profiles": qos.get(t["topic_metadata"]["name"], "")},
               "message_count": t["message_count"]} for t in m["topics_with_message_count"]]
    v5 = {"rosbag2_bagfile_information": {
        "version": 5, "storage_identifier": m["storage_identifier"], "duration": m["duration"],
        "starting_time": m["starting_time"], "message_count": m["message_count"],
        "topics_with_message_count": topics, "compression_format": "", "compression_mode": "",
        "relative_file_paths": m["relative_file_paths"], "files": m["files"]}}
    yaml.safe_dump(v5, open(path, "w"), sort_keys=False, default_flow_style=False, width=10**6)


def read_imu(bag, topic):
    t, v, rx, tmpl = [], [], [], None
    with Reader(bag) as r:
        conns = [c for c in r.connections if c.topic == topic]
        if not conns:
            sys.exit(f"topic {topic} not in {bag}")
        for c, ts, raw in r.messages(connections=conns):
            m = TS.deserialize_cdr(raw, c.msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            v.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                      m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
            rx.append(ts * 1e-9)
            if tmpl is None:
                tmpl = (c.msgtype, raw)
    t, v, rx = np.array(t), np.array(v), np.array(rx)
    o = np.argsort(t, kind="stable")
    return t[o], v[o], rx[o], tmpl


def cum_integral(t, v, x):
    """Integral from t[0] to x of the piecewise-linear signal through (t, v); v is (n, k). x sorted, in range."""
    seg = np.diff(t)
    inc = 0.5 * (v[1:] + v[:-1]) * seg[:, None]
    base = np.vstack([np.zeros((1, v.shape[1])), np.cumsum(inc, axis=0)])
    i = np.clip(np.searchsorted(t, x, side="right") - 1, 0, len(t) - 2)
    s = x - t[i]
    slope = (v[i + 1] - v[i]) / seg[i][:, None]
    return base[i] + v[i] * s[:, None] + 0.5 * slope * (s * s)[:, None]


def resample(t, v, hz, method, max_gap):
    P = 1.0 / hz
    gaps = np.diff(t)
    if method == "avg":
        n = int((t[-1] - t[0]) / P)
        edges = t[0] + np.arange(n + 1) * P
        centre = edges[:-1] + 0.5 * P
        F = cum_integral(t, v, edges)
        out = (F[1:] - F[:-1]) / P
    else:
        n = int((t[-1] - t[0]) / P) + 1
        centre = t[0] + np.arange(n) * P
        if method == "interp":
            out = np.stack([np.interp(centre, t, v[:, k]) for k in range(v.shape[1])], axis=1)
        else:  # nearest
            j = np.clip(np.searchsorted(t, centre), 1, len(t) - 1)
            j = np.where(np.abs(t[j - 1] - centre) <= np.abs(t[j] - centre), j - 1, j)
            keep = np.abs(t[j] - centre) <= 0.5 * P
            centre, out = centre[keep], v[j[keep]]
    # drop bins that sit inside an original gap longer than max_gap
    i = np.clip(np.searchsorted(t, centre, side="right") - 1, 0, len(t) - 2)
    ok = gaps[i] <= max_gap
    return centre[ok], out[ok], int((~ok).sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("in_bag"); ap.add_argument("out_bag")
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--method", choices=("avg", "interp", "nearest"), default="avg")
    ap.add_argument("--topic", default="/imu0")
    ap.add_argument("--max-gap-ms", type=float, default=50.0)
    ap.add_argument("--version", type=int, choices=(8, 9), default=9)
    a = ap.parse_args()
    if os.path.exists(a.out_bag):
        sys.exit(f"{a.out_bag} exists -- refusing to overwrite")

    t, v, rx, (msgtype, raw0) = read_imu(a.in_bag, a.topic)
    lag = float(np.median(rx - t))
    tn, vn, skipped = resample(t, v, a.hz, a.method, a.max_gap_ms * 1e-3)
    d = np.diff(tn) * 1e3
    print(f"in : {len(t)} msgs over {t[-1]-t[0]:.1f} s = {len(t)/(t[-1]-t[0]):.1f} Hz  (stamp dt median {np.median(np.diff(t))*1e3:.2f} ms)")
    print(f"out: {len(tn)} msgs, {a.method} @ {a.hz:g} Hz, stamp dt median {np.median(d):.3f} ms, "
          f"min {d.min():.3f} max {d.max():.3f}; bins skipped in gaps >{a.max_gap_ms:g} ms: {skipped}")
    print(f"     bag timestamp = header stamp + {lag:.6f} s (median receive time - header stamp of the original; "
          f"huge when headers are on CLOCK_MONOTONIC and the bag on wall clock)")

    tmpl = TS.deserialize_cdr(raw0, msgtype)
    with Reader(a.in_bag) as r, Writer(a.out_bag, version=a.version) as w:
        in_conns = list(r.connections)
        mapping = {}
        for c in r.connections:
            mapping[c.id] = w.add_connection(c.topic, c.msgtype, typestore=TS,
                                             serialization_format=c.ext.serialization_format,
                                             offered_qos_profiles=c.ext.offered_qos_profiles)
        n_copied = 0
        for c, ts, raw in r.messages():
            if c.topic == a.topic:
                continue
            w.write(mapping[c.id], ts, raw)
            n_copied += 1
        imu_conn = next(mapping[c.id] for c in r.connections if c.topic == a.topic)
        for ti, vi in zip(tn, vn):
            m = tmpl
            sec = int(ti); m.header.stamp.sec = sec; m.header.stamp.nanosec = int(round((ti - sec) * 1e9))
            if m.header.stamp.nanosec >= 10**9:
                m.header.stamp.sec += 1; m.header.stamp.nanosec -= 10**9
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (float(x) for x in vi[:3])
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (float(x) for x in vi[3:])
            w.write(imu_conn, int(round((ti + lag) * 1e9)), TS.serialize_cdr(m, msgtype))
    humble_metadata(a.out_bag, in_conns)
    print(f"wrote {a.out_bag}: {n_copied} other messages copied unchanged + {len(tn)} resampled {a.topic}")


if __name__ == "__main__":
    main()
