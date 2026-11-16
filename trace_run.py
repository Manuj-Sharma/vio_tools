#!/usr/bin/env python3
"""trace_run.py <run_folder> [step_s] -- time series of a run: VIO height vs /fc/ekf_alt, and distance-from-start of VIO vs GPS.
Shows WHEN a run departs from the truth. Needs: source /opt/ros/humble/setup.bash; plain python3."""
import sys,os,glob,math,struct,sqlite3,numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
run=os.path.abspath(sys.argv[1]); step=float(sys.argv[2]) if len(sys.argv)>2 else 10
meta=dict(l.strip().split(' ',1) for l in open(run+'/meta.txt') if ' ' in l); dbs=sorted(glob.glob(meta['bag']+'/*.db3'))
imu0=None;alt=[];gps=[]
for p in dbs:
    c=sqlite3.connect(p); ids={n:i for i,n in c.execute("select id,name from topics")}
    if imu0 is None and '/imu0' in ids:
        ts,d=c.execute("select timestamp,data from messages where topic_id=? order by timestamp limit 1",(ids['/imu0'],)).fetchone(); m=deserialize_message(d,Imu); imu0=(ts*1e-9,m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
    if '/fc/ekf_alt' in ids:
        for ts,d in c.execute("select timestamp,data from messages where topic_id=? order by timestamp",(ids['/fc/ekf_alt'],)): alt.append((ts*1e-9,struct.unpack('<f',d[4:8])[0]))
    if '/fc/gps1' in ids:
        for ts,d in c.execute("select timestamp,data from messages where topic_id=? order by timestamp",(ids['/fc/gps1'],)):
            m=deserialize_message(d,NavSatFix); gps.append((ts*1e-9,m.latitude,m.longitude))
off=imu0[0]-imu0[1]; alt=np.array(alt); alt[:,1]-=alt[:20,1].mean(); gps=np.array(gps)
oc=sqlite3.connect(glob.glob(run+'/odom_out/*.db3')[0]); oids={n:i for i,n in oc.execute("select id,name from topics")}
r=[]
for ts,d in oc.execute("select timestamp,data from messages where topic_id=? order by timestamp",(oids['/ov_msckf/odomimu'],)):
    o=deserialize_message(d,Odometry); p=o.pose.pose.position; r.append((o.header.stamp.sec+o.header.stamp.nanosec*1e-9+off,p.x,p.y,p.z))
r=np.array(r); t0=r[0,0]; R0=6378137.
def gps_xy(t):
    la=np.interp(t,gps[:,0],gps[:,1]); lo=np.interp(t,gps[:,0],gps[:,2]); la0,lo0=gps[0,1],gps[0,2]
    return np.array([math.radians(lo-lo0)*R0*math.cos(math.radians(la0)),math.radians(la-la0)*R0])
g0=gps_xy(t0+0)
import re
ba=[];bg=[]
for l in open(run+'/openvins.log',errors='ignore'):
    m=re.search(r'bg = ([-\d.,]+) \| ba = ([-\d.,]+)',re.sub(r'\x1b\[[0-9;]*m','',l))
    if m: bg.append([float(x) for x in m.group(1).split(',')]); ba.append([float(x) for x in m.group(2).split(',')])
ba=np.array(ba); bg=np.array(bg)
print('(bias lines %d vs odom msgs %d: aligned by fraction of run)'%(len(ba),len(r)))
print('%-6s %-8s %-8s %-7s | %-10s %-10s | %-26s %-8s'%('t(s)','VIO z','GPS h','ratio','VIO dist','GPS dist','accel bias x y z (m/s2)','|bg|'))
for T in np.arange(0,r[-1,0]-t0,step):
    i=np.searchsorted(r[:,0],t0+T)
    if i>=len(r): break
    h=np.interp(r[i,0],alt[:,0],alt[:,1]); gd=np.linalg.norm(gps_xy(r[i,0])-g0); vd=math.hypot(r[i,1]-r[0,1],r[i,2]-r[0,2])
    k=min(len(ba)-1,int(i*len(ba)/len(r))) if len(ba) else None
    bs='%7.3f %7.3f %7.3f'%tuple(ba[k]) if k is not None else '-'
    gs='%.4f'%np.linalg.norm(bg[k]) if k is not None else '-'
    print('%-6d %-8.1f %-8.1f %-7s | %-10.0f %-10.0f | %-26s %-8s'%(T,r[i,3],h,'%.2f'%(r[i,3]/h) if h>5 else '-',vd,gd,bs,gs))
