#!/usr/bin/env python3
"""imu_repeat_check.py BAG_DIR [--topic /imu0]

Do IMU samples repeat? For each consecutive pair of /imu0 messages in a rosbag2 (sqlite3) bag:
 - duplicate header stamp (same stamp twice)
 - identical full sample (all 6 gyro/accel values bit-equal) = the same IMX sample read twice
 - per-axis equal values
and the header-stamp step histogram (the IMX sample interval as the driver stamps it).
"""
import glob, sqlite3, sys, collections
import rclpy.serialization as ser
from sensor_msgs.msg import Imu

topic = sys.argv[sys.argv.index("--topic") + 1] if "--topic" in sys.argv else "/imu0"
bag = sys.argv[1]
rows = []
for f in sorted(glob.glob(bag + "/*.db3")):
    c = sqlite3.connect(f"file:{f}?immutable=1", uri=True)
    t = c.execute("select id from topics where name=?", (topic,)).fetchone()
    if not t:
        continue
    for ts, data in c.execute("select timestamp, data from messages where topic_id=? order by timestamp", (t[0],)):
        m = ser.deserialize_message(bytes(data), Imu)
        rows.append((m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, ts,
                     (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                      m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)))
n = len(rows)
dup_stamp = sum(1 for a, b in zip(rows, rows[1:]) if a[0] == b[0])
same_all = sum(1 for a, b in zip(rows, rows[1:]) if a[2] == b[2])
axis_same = [sum(1 for a, b in zip(rows, rows[1:]) if a[2][k] == b[2][k]) for k in range(6)]
steps = collections.Counter(round((b[0] - a[0]) / 1e6) for a, b in zip(rows, rows[1:]))
distinct = len({r[2] for r in rows})
print(f"{bag}\n{n} msgs")
print(f"  duplicate header stamps (consecutive): {dup_stamp}")
print(f"  consecutive identical 6-axis samples:  {same_all}  ({100*same_all/max(1,n-1):.2f}%)")
print(f"  distinct 6-axis tuples overall:        {distinct}  ({100*distinct/n:.1f}% of msgs)")
print("  consecutive-equal per axis [gx gy gz ax ay az]: " + " ".join(f"{100*x/max(1,n-1):.1f}%" for x in axis_same))
print("  header-stamp step (ms): " + ", ".join(f"{k}ms x{v}" for k, v in sorted(steps.items())[:12]))
