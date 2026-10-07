#!/usr/bin/env python3
"""brightness_check.py BAG_DIR -- mean brightness and clipped-pixel share per camera (every 20th frame).

    python3 brightness_check.py ~/rosbags/vio_20261007_193717

Used 2026-10-07 to confirm the 1-8 ms exposure cap did not darken indoor images (about 124/255, 3-5% clipped).
"""
import sqlite3, sys, numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
n=sys.argv[1].rstrip("/").split("/")[-1]; db=sqlite3.connect(f"{sys.argv[1]}/{n}_0.db3")
for t in ("/cam0/image_raw","/cam1/image_raw"):
    tid=db.execute("select id from topics where name=?",(t,)).fetchone()[0]; v=[]
    for k,(d,) in enumerate(db.execute("select data from messages where topic_id=? order by id",(tid,))):
        if k%20: continue
        m=deserialize_message(d,Image); im=np.frombuffer(m.data,np.uint8); v.append((im.mean(),(im>=250).mean()))
    v=np.array(v); print(n[-6:],t,"mean brightness",v[:,0].mean().round(1),"min/max",v[:,0].min().round(0),v[:,0].max().round(0),"clipped %",(100*v[:,1].mean()).round(2))
