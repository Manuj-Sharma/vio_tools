#!/usr/bin/env python3
"""stereo_slot_check.py BAG_DIR -- per-camera rate, trigger-slot gaps and stereo offset from a recorded bag.

    source /opt/ros/humble/setup.bash
    python3 stereo_slot_check.py ~/rosbags/vio_20261007_193717

Written 2026-10-07 for the AR0234 stereo timing work (see memory camera-stereo-sync-investigation-2026-10-07).
Reads header stamps of /cam0/image_raw and /cam1/image_raw directly from the sqlite3 bag. Reports:
  * rate and frame-interval percentiles per camera
  * slot gaps: interval / (1/60 s) rounded -- 1 = every trigger delivered, 2 = one skipped, ...
  * stereo offset cam1-cam0 by nearest stamp, share within +-3 / +-10 ms, frames reused / unpaired
  * cam0 rate per quarter of the bag (does the rate sag over time?)
Header stamps are HOST-side (taken after VxGetImage), not exposure times, so offsets include stamp noise.
Assumes a 60 Hz trigger (P below).
"""
import sqlite3, sys, numpy as np
from collections import Counter
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
n=sys.argv[1].rstrip("/").split("/")[-1]
db=sqlite3.connect(f"{sys.argv[1]}/{n}_0.db3"); T={}
for t in ("/cam0/image_raw","/cam1/image_raw"):
    tid=db.execute("select id from topics where name=?",(t,)).fetchone()[0]
    T[t]=np.array([(lambda m:m.header.stamp.sec+m.header.stamp.nanosec*1e-9)(deserialize_message(d,Image)) for (d,) in db.execute("select data from messages where topic_id=? order by id",(tid,))])
a,b=T["/cam0/image_raw"],T["/cam1/image_raw"]; P=1/60
for nm,x in (("cam0",a),("cam1",b)):
    dt=np.diff(x)*1e3
    print(nm,"n",len(x),"span s",round(x[-1]-x[0],1),"rate",round((len(x)-1)/(x[-1]-x[0]),1),"dt pct 1/10/50/90/99",np.percentile(dt,[1,10,50,90,99]).round(1))
    print("  slot gaps (x16.67ms):",sorted(Counter(np.round(np.diff(x)/P).astype(int)).items()))
i=np.clip(np.searchsorted(b,a),1,len(b)-1); c=np.where(abs(b[i]-a)<abs(b[i-1]-a),i,i-1); off=(b[c]-a)*1e3
print("offset ms pct 5/25/50/75/95:",np.percentile(off,[5,25,50,75,95]).round(1),"| |off|<3ms:",np.mean(abs(off)<3).round(2),"<10ms:",np.mean(abs(off)<10).round(2),"| >25ms:",np.mean(abs(off)>25).round(3))
print("cam1 frames used >1x:",len(c)-len(set(c)),"| cam1 frames unused:",len(b)-len(set(c)))
# cam0 gaps over time (is cam0 slow throughout?)
h=len(a)//4; print("cam0 rate per quarter:",[round(h/(a[min((k+1)*h,len(a)-1)]-a[k*h]),1) for k in range(4)])
