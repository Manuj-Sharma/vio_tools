#!/usr/bin/env python3
"""feat_tap.py [seconds] [csv]: live tap of OpenVINS feature clouds: number of MSCKF / SLAM points per message and VIO height, 1 line per second."""
import sys, time, rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
dur=float(sys.argv[1]) if len(sys.argv)>1 else 10; out=open(sys.argv[2],'w') if len(sys.argv)>2 else None
rclpy.init(); n=Node('feat_tap'); S={'m':[], 's':[], 'z':None, 'v':None}
n.create_subscription(PointCloud2,'/ov_msckf/points_msckf',lambda m:S['m'].append(m.width*m.height),10)
n.create_subscription(PointCloud2,'/ov_msckf/points_slam',lambda m:S['s'].append(m.width*m.height),10)
def od(m): S['z']=m.pose.pose.position.z; S['v']=np.linalg.norm([m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z]); S['t']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
n.create_subscription(Odometry,'/ov_msckf/odomimu',od,qos_profile_sensor_data)
t0=time.time(); nxt=t0+1
while time.time()-t0<dur and rclpy.ok():
    rclpy.spin_once(n,timeout_sec=0.05)
    if time.time()>=nxt:
        nxt+=1; line='%.0f,%s,%s,%d,%.1f,%d,%.1f'%(S.get('t',0),'%.1f'%S['z'] if S['z'] is not None else '', '%.1f'%S['v'] if S['v'] is not None else '',len(S['m']),np.mean(S['m']) if S['m'] else 0,len(S['s']),np.mean(S['s']) if S['s'] else 0)
        print(line,flush=True); out and (out.write(line+'\n'),out.flush()); S['m']=[];S['s']=[]
