#!/usr/bin/env python3
"""umeyama_report.py <run_folder> [...]: align the whole VIO track to GPS (/fc/gps1) with Umeyama and report what is left.
A) full similarity (scale, rotation, translation)  B) rigid, scale fixed to 1  C) yaw + translation only, scale 1 (both frames gravity-aligned).
Also: best GPS-VIO time shift, tilt of the fitted rotation, and mean residual (E,N,U) per third of the flight -> shows constant offsets / drifts."""
import sys, os, glob, math, sqlite3, numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
def um(X,Y,scale=True):
    mx,my=X.mean(0),Y.mean(0);Xc,Yc=X-mx,Y-my;U,D,Vt=np.linalg.svd(Yc.T@Xc/len(X));S=np.eye(3)
    if np.linalg.det(U)*np.linalg.det(Vt)<0:S[2,2]=-1
    R=U@S@Vt; s=(D*np.diag(S)).sum()/(Xc**2).sum()*len(X) if scale else 1.0; return s,R,my-s*R@mx
def yaw_fit(X,Y):
    mx,my=X.mean(0),Y.mean(0);a=X-mx;b=Y-my; th=math.atan2((a[:,0]*b[:,1]-a[:,1]*b[:,0]).sum(),(a[:,0]*b[:,0]+a[:,1]*b[:,1]).sum()); c,s=math.cos(th),math.sin(th); R=np.array([[c,-s,0],[s,c,0],[0,0,1]]); return 1.0,R,my-R@mx
GC={}
def gps_of(bag):
    if not glob.glob(bag+'/*.db3'):   # derived Cube bag deleted -> GPS and clock offset from the original bag (same clocks)
        for k,o in (('092757','vio_20261103_092757_ctrl'),('110545','vio_20261104_110545'),('165506','vio_20261103_165506')):
            if k in bag: bag=os.path.expanduser('~/rosbags/'+o)
    if bag in GC: return GC[bag]
    g=[];off=None
    for p in sorted(glob.glob(bag+'/*.db3')):
        c=sqlite3.connect('file:%s?mode=ro'%p,uri=True); ids={n:i for i,n in c.execute("select id,name from topics")}
        if off is None and '/imu0' in ids:
            r=c.execute("select timestamp,data from messages where topic_id=? order by timestamp limit 1",(ids['/imu0'],)).fetchone()
            if r: m=deserialize_message(r[1],Imu); off=r[0]*1e-9-(m.header.stamp.sec+m.header.stamp.nanosec*1e-9); t0=r[0]*1e-9
        if '/fc/gps1' in ids:
            for ts,d in c.execute("select timestamp,data from messages where topic_id=? order by timestamp",(ids['/fc/gps1'],)): m=deserialize_message(d,NavSatFix); g.append((ts*1e-9,m.latitude,m.longitude,m.altitude))
    g=np.array(g); lat0,lon0,a0=g[0,1:]; R0=6378137.; E=np.c_[g[:,0],np.radians(g[:,2]-lon0)*R0*math.cos(math.radians(lat0)),np.radians(g[:,1]-lat0)*R0,g[:,3]-a0]; GC[bag]=(E,off,t0); return GC[bag]
for run in sys.argv[1:]:
    run=os.path.abspath(run); meta=dict(l.strip().split(' ',1) for l in open(run+'/meta.txt') if ' ' in l.strip()); E,off,t0=gps_of(meta['bag']); r=[]
    for p in sorted(glob.glob(run+'/odom_out/*.db3')):
        c=sqlite3.connect('file:%s?mode=ro'%p,uri=True); ids={n:i for i,n in c.execute("select id,name from topics")}
        for k,(ts,d) in enumerate(c.execute("select timestamp,data from messages where topic_id=? order by timestamp",(ids['/ov_msckf/odomimu'],))):
            if k%6==0: o=deserialize_message(d,Odometry); r.append((o.header.stamp.sec+o.header.stamp.nanosec*1e-9+off,o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z))
    r=np.array(r); sel=(E[:,0]>r[0,0]+3)&(E[:,0]<r[-1,0]-1); G=E[sel]; tg=G[:,0]; Y=G[:,1:]
    vio=lambda dt: np.c_[[np.interp(tg+dt,r[:,0],r[:,k]) for k in (1,2,3)]].T
    def rms_of(dt): X=vio(dt); s,R,t=um(X,Y); return math.sqrt((((s*(R@X.T)).T+t-Y)**2).sum(1).mean())
    dts=np.arange(-1.5,1.51,0.05); rr=[rms_of(d) for d in dts]; dtb=dts[int(np.argmin(rr))]
    X=vio(0.0); path=np.linalg.norm(np.diff(Y,axis=0),axis=1).sum()
    print('\n=== %s | bag %s | fit window bag t %.0f-%.0f s (%d GPS points, GPS path %.0f m, height range %.0f m)'%(meta['tag'],os.path.basename(meta['bag']),tg[0]-t0,tg[-1]-t0,len(tg),path,np.ptp(Y[:,2])))
    print('    best GPS-VIO time shift %.2f s (rms %.2f m vs %.2f m at 0)'%(dtb,min(rr),rms_of(0.0)))
    for name,(s,R,t) in (('A similarity   ',um(X,Y)),('B rigid scale=1',um(X,Y,False)),('C yaw-only s=1 ',yaw_fit(X,Y))):
        F=(s*(R@X.T)).T+t; e=F-Y; tilt=math.degrees(math.acos(np.clip(R[2,2],-1,1))); yaw=math.degrees(math.atan2(R[1,0],R[0,0])); n=len(e)//3
        print('  %s scale %.3f | yaw %7.2f deg, tilt %.2f deg | rms %.2f m (horiz %.2f, vert %.2f) | mean residual E,N,U per third: %s'%(name,s,yaw,tilt,math.sqrt((e**2).sum(1).mean()),math.sqrt((e[:,:2]**2).sum(1).mean()),math.sqrt((e[:,2]**2).mean()),' | '.join('%+5.1f %+5.1f %+5.1f'%tuple(e[i*n:(i+1)*n].mean(0)) for i in range(3))))
