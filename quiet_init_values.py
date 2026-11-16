#!/usr/bin/env python3
"""quiet_init_values.py <bag_dir> <search_from_s> <search_to_s> [window_s=10] -- find the quietest IMU window and write init values.
Quiet = lowest accel scatter (init-style sqrt(sum var)) over sliding windows inside [from,to] (s from first IMU).
Writes ~/vio_tools/init_values/<bag>_quiet.yaml with gyro bias, mean accel (IMU frame), |a|, up direction, tilt, accel bias implied by gravity_mag.
Needs: source /opt/ros/humble/setup.bash ; plain python3."""
import sys,os,glob,sqlite3,yaml,numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
bag=os.path.abspath(os.path.expanduser(sys.argv[1])); lo=float(sys.argv[2]); hi=float(sys.argv[3]); W=float(sys.argv[4]) if len(sys.argv)>4 else 10.0
rows=[]
for p in sorted(glob.glob(bag+'/*.db3')):
    c=sqlite3.connect('file:%s?mode=ro'%p,uri=True); ids={n:i for i,n in c.execute("select id,name from topics")}
    if '/imu0' not in ids: continue
    for ts,d in c.execute("select timestamp,data from messages where topic_id=? order by timestamp",(ids['/imu0'],)):
        m=deserialize_message(d,Imu); rows.append((ts*1e-9,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z))
        if rows[-1][0]-rows[0][0]>hi+1: break
    if rows and rows[-1][0]-rows[0][0]>hi+1: break
a=np.array(rows); t=a[:,0]-a[0,0]
best=None
for s in np.arange(lo,hi-W,1.0):
    m=(t>=s)&(t<s+W); acc=a[m,4:7]; g=a[m,1:4]
    sd=np.sqrt(((acc-acc.mean(0))**2).sum(1).mean()); gs=np.linalg.norm(g.std(0))
    if best is None or sd<best[0]: best=(sd,gs,s)
sd,gs,s=best; m=(t>=s)&(t<s+W); g=a[m,1:4]; acc=a[m,4:7]; n=m.sum()
bg=g.mean(0); am=acc.mean(0); an=np.linalg.norm(am); up=am/an
GM=9.7801; ba=am-GM*up
tilt=np.degrees(np.arccos(abs(up[2])))
out={'bag':os.path.basename(bag),'window_s_from_first_imu':[float(s),float(s+W)],'n_samples':int(n),
 'gyro_bias_rad_s':[float(x) for x in bg],'gyro_std_rad_s':[float(x) for x in g.std(0)],'gyro_bias_stderr':[float(x) for x in g.std(0)/np.sqrt(n)],
 'accel_mean_imu_frame':[float(x) for x in am],'accel_norm':float(an),'up_direction_imu_frame':[float(x) for x in up],'tilt_from_imu_z_deg':float(tilt),
 'accel_std_initstyle':float(sd),'gravity_mag_assumed':GM,'accel_bias_implied_by_gravity_mag':[float(x) for x in ba],'accel_mean_stderr':[float(x) for x in acc.std(0)/np.sqrt(n)]}
op=os.path.expanduser('~/vio_tools/init_values/%s_quiet.yaml'%os.path.basename(bag)); yaml.safe_dump(out,open(op,'w'),sort_keys=False); print(yaml.safe_dump(out,sort_keys=False)); print('written',op)
