#!/usr/bin/env python3
"""select_still_frames.py - build a ROS 2 bag with only the SHARPEST / STILLEST frames of an image topic.

For a camera-intrinsics recording where the board is moved by hand: frames taken while the board moves are motion-blurred and
have noisy corners (Kalibr reprojection error ~1 px instead of ~0.3 px). Per window of WINDOW seconds, this keeps the single
frame with the smallest frame-to-frame motion (mean abs difference to BOTH neighbours, on a 4x down-sampled image) whose
sharpness (variance of the Laplacian) is not far below the median. Everything else is dropped.

  select_still_frames.py <ros2 bag dir> <out bag dir> [--topic /cam1/image_raw] [--window 0.5] [--sharp-frac 0.8]

Reads the sqlite3 .db3 directly (read-only, immutable) and writes a new .db3 + metadata.yaml (message counts fixed).
Needs ROS 2 Humble sourced (rclpy deserialisation) and cv2.
"""
import argparse, os, shutil, sqlite3, sys
import numpy as np
import cv2
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

ap = argparse.ArgumentParser()
ap.add_argument("src"); ap.add_argument("dst")
ap.add_argument("--topic", default="/cam1/image_raw")
ap.add_argument("--window", type=float, default=0.5, help="keep one frame per this many seconds")
ap.add_argument("--sharp-frac", type=float, default=0.8, help="frame must have >= this fraction of the median sharpness")
a = ap.parse_args()

dbs = [f for f in os.listdir(a.src) if f.endswith(".db3")]
if len(dbs) != 1: sys.exit("expected exactly one .db3 in %s" % a.src)
src_db = os.path.join(a.src, dbs[0])
c = sqlite3.connect("file:%s?immutable=1" % src_db, uri=True)
tid = c.execute("select id from topics where name=?", (a.topic,)).fetchone()[0]
rows = c.execute("select id,timestamp from messages where topic_id=? order by timestamp", (tid,)).fetchall()
ids = [r[0] for r in rows]; ts = np.array([r[1] for r in rows]) / 1e9
print("topic %s: %d frames over %.1f s" % (a.topic, len(ids), ts[-1] - ts[0]))

small, sharp = [], []
for k, mid in enumerate(ids):
    data = c.execute("select data from messages where id=?", (mid,)).fetchone()[0]
    m = deserialize_message(data, Image)
    img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width]
    sm = cv2.resize(img, (m.width // 4, m.height // 4), interpolation=cv2.INTER_AREA).astype(np.float32)
    small.append(sm)
    sharp.append(cv2.Laplacian(cv2.resize(img, (m.width // 2, m.height // 2), interpolation=cv2.INTER_AREA), cv2.CV_64F).var())
    if k % 1000 == 0: print("  read %d/%d" % (k, len(ids)), flush=True)
sharp = np.array(sharp)
md = np.array([np.abs(small[i] - small[i - 1]).mean() if i > 0 else np.nan for i in range(len(small))])  # diff to previous
motion = np.full(len(small), np.nan)
for i in range(1, len(small) - 1): motion[i] = max(md[i], md[i + 1])
print("sharpness (Laplacian var): median %.0f  p10 %.0f  p90 %.0f" % (np.median(sharp), np.percentile(sharp, 10), np.percentile(sharp, 90)))
print("motion score: median %.2f  p10 %.2f  p90 %.2f" % (np.nanmedian(motion), np.nanpercentile(motion, 10), np.nanpercentile(motion, 90)))

thr = a.sharp_frac * np.median(sharp)
keep = []
t0 = ts[0]
for w in range(int((ts[-1] - t0) / a.window) + 1):
    idx = [i for i in range(len(ids)) if w * a.window <= ts[i] - t0 < (w + 1) * a.window and not np.isnan(motion[i]) and sharp[i] >= thr]
    if idx: keep.append(min(idx, key=lambda i: motion[i]))
keep_ids = [ids[i] for i in keep]
print("kept %d frames (%.1f Hz); their motion score: median %.2f (all frames %.2f); sharpness median %.0f (all %.0f)" %
      (len(keep), len(keep) / (ts[-1] - t0), np.median(motion[keep]), np.nanmedian(motion), np.median(sharp[keep]), np.median(sharp)))

os.makedirs(a.dst, exist_ok=True)
name = os.path.basename(a.dst.rstrip("/")) + "_0.db3"
out_db = os.path.join(a.dst, name)
if os.path.exists(out_db): os.remove(out_db)
o = sqlite3.connect(out_db)
for (sql,) in c.execute("select sql from sqlite_master where sql is not null and name not like 'sqlite_%'"):
    o.execute(sql)
for r in c.execute("select * from topics where id=?", (tid,)): o.execute("insert into topics values (%s)" % ",".join("?" * len(r)), r)
for n, mid in enumerate(keep_ids, 1):
    mid_, topic_id, stamp, data = c.execute("select id,topic_id,timestamp,data from messages where id=?", (mid,)).fetchone()
    o.execute("insert into messages (id,topic_id,timestamp,data) values (?,?,?,?)", (n, topic_id, stamp, data))
o.commit(); o.close()

meta = open(os.path.join(a.src, "metadata.yaml")).read().split("\n")
with open(os.path.join(a.dst, "metadata.yaml"), "w") as f:
    for line in meta:
        s = line.strip()
        if s.startswith("- ") and ".db3" in s and "path:" not in s: line = line[:line.index("-")] + "- " + name
        elif "path:" in s and ".db3" in s: line = line[:line.index("path:")] + "path: " + name
        elif s.startswith("message_count:"): line = line[:len(line) - len(line.lstrip())] + "message_count: %d" % len(keep_ids)
        f.write(line + "\n")
print("wrote", out_db, "(%d MB)" % (os.path.getsize(out_db) // 1e6))
