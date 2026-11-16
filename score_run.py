#!/usr/bin/env python3
"""score_run.py <run_folder> [--static-window LO HI] -- score one run_bag.sh run against the bag's own /fc data.
Writes <run>/results.txt and appends one row to ~/vio_tools/records/runs.csv.
Needs: source /opt/ros/humble/setup.bash.   Run with plain python3 (not -I; rclpy needs PYTHONPATH).
Reports: init gravity error vs the bag's static accel, init biases vs true static bias, bias max/trajectory,
height ratio vs /fc/ekf_alt at fixed GPS heights, GPS-track fit (scale/rms) when /fc/gps1 exists, divergence flag."""
import sys,os,re,glob,math,struct,sqlite3,csv,datetime,numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
run=os.path.abspath(sys.argv[1]); lo,hi=(5.0,25.0)
if '--static-window' in sys.argv: i=sys.argv.index('--static-window'); lo,hi=float(sys.argv[i+1]),float(sys.argv[i+2])
meta=dict(l.strip().split(' ',1) for l in open(run+'/meta.txt') if ' ' in l)
bag=meta['bag']; dbs=sorted(glob.glob(bag+'/*.db3'))
imu=[];alt=[];gps=[]
for p in dbs:
    c=sqlite3.connect(p); ids={n:i for i,n in c.execute("select id,name from topics")}
    q=lambda n: c.execute("select timestamp,data from messages where topic_id=? order by timestamp",(ids[n],))
    if '/imu0' in ids:
        for ts,d in q('/imu0'):
            m=deserialize_message(d,Imu); imu.append((ts*1e-9,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,*[getattr(m.angular_velocity,k) for k in 'xyz'],*[getattr(m.linear_acceleration,k) for k in 'xyz']))
    if '/fc/ekf_alt' in ids:
        for ts,d in q('/fc/ekf_alt'): alt.append((ts*1e-9,struct.unpack('<f',d[4:8])[0]))
    if '/fc/gps1' in ids:
        for ts,d in q('/fc/gps1'):
            m=deserialize_message(d,NavSatFix); gps.append((ts*1e-9,m.latitude,m.longitude,m.altitude))
imu=np.array(imu);alt=np.array(alt);gps=np.array(gps); off=imu[0,0]-imu[0,1]; t=imu[:,0]-imu[0,0]
alt[:,1]-=alt[:20,1].mean()
st=(t>lo)&(t<hi); gb=imu[st,2:5].mean(0); am=imu[st,5:8].mean(0); amn=am/np.linalg.norm(am)
def Rq(q):
    x,y,z,w=q; return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
oc=sqlite3.connect(glob.glob(run+'/odom_out/*.db3')[0]); oids={n:i for i,n in oc.execute("select id,name from topics")}
rows=[];q0=None
for ts,d in oc.execute("select timestamp,data from messages where topic_id=? order by timestamp",(oids['/ov_msckf/odomimu'],)):
    o=deserialize_message(d,Odometry); p=o.pose.pose.position
    if q0 is None: q0=o.pose.pose.orientation
    rows.append((o.header.stamp.sec+o.header.stamp.nanosec*1e-9+off,p.x,p.y,p.z))
r=np.array(rows); t0=r[0,0]
out=[]; P=out.append
P('RUN %s | bag %s | offset %s | core %s | config_md5 %s | open_vins %s'%(meta['tag'],os.path.basename(bag),meta['offset'],meta['core'],meta['config_md5'],meta['open_vins_git']))
g=Rq((q0.x,q0.y,q0.z,q0.w)).T@np.array([0,0,1.0]); gerr=math.degrees(math.acos(np.clip(g@amn,-1,1)))
tilt=math.degrees(math.acos(abs(amn[2])))
P('init: first odom at bag t=%.1f s | static window %g-%g s true tilt %.2f deg, gyro bias %s | init gravity-direction error %.2f deg'%(t0-imu[0,0],lo,hi,tilt,np.round(gb,4),gerr))
txt=re.sub(r'\x1b\[[0-9;]*m','',open(run+'/openvins.log',errors='ignore').read()).split('\n')
ib=ia=None;bs=[]
for i,l in enumerate(txt):
    if '[init]: bias gyro' in l and ib is None:
        ib=[float(x) for x in l.split('=')[1].split(',')]; ia=[float(x) for x in [y for y in txt[i:i+4] if 'bias accel' in y][0].split('=')[1].split(',')]
    m=re.search(r'bg = ([-\d.,]+) \| ba = ([-\d.,]+)',l)
    if m:
        try: bs.append((np.array(m.group(1).split(','),float),np.array(m.group(2).split(','),float)))
        except ValueError: pass   # truncated / garbled log line
maxba=max((np.linalg.norm(b[1]) for b in bs),default=float('nan')); maxbg=max((np.linalg.norm(b[0]) for b in bs),default=float('nan'))
P('init biases: gyro %s (true %s) accel %s | later max |ba| %.3f m/s2, max |bg| %.4f rad/s'%(ib,np.round(gb,4),ia,maxba,maxbg))
P('height vs /fc/ekf_alt: VIO height when GPS height first reaches 10/20/40/60/100/150 m (and ratio)')
hs=[];ratios=[]
for L in (10,20,40,60,100,150):
    k=np.argmax(alt[:,1]>=L)
    if alt[k,1]<L: hs.append('%d:n/a'%L); ratios.append(float('nan')); continue
    i=np.searchsorted(r[:,0],alt[k,0]); 
    if i>=len(r) or r[0,0]>alt[k,0]: hs.append('%d:-'%L); ratios.append(float('nan')); continue
    hs.append('%d:%.1f(%.2f)'%(L,r[i,3],r[i,3]/alt[k,1])); ratios.append(r[i,3]/alt[k,1])
P('  '+'  '.join(hs))
zmax=r[:,3].max(); amax=alt[:,1].max(); xy=np.hypot(r[:,1],r[:,2]).max()
div=bool((np.abs(r[:,3])>3*amax+50).any())
P('max VIO z %.1f m vs max GPS height %.1f m | max horizontal from start %.0f m | diverged(|z|>3*max+50): %s'%(zmax,amax,xy,div))
fit=[]
if len(gps)>10:
    lat0,lon0,a0=gps[0,1:]; R0=6378137.; E=np.c_[np.radians(gps[:,2]-lon0)*R0*math.cos(math.radians(lat0)),np.radians(gps[:,1]-lat0)*R0,gps[:,3]-a0]
    def um(X,Y):
        mx,my=X.mean(0),Y.mean(0);Xc,Yc=X-mx,Y-my;U,D,Vt=np.linalg.svd(Yc.T@Xc/len(X));S=np.eye(3)
        if np.linalg.det(U)*np.linalg.det(Vt)<0:S[2,2]=-1
        Rm=U@S@Vt;s=(D*np.diag(S)).sum()/(Xc**2).sum()*len(X);return s,Rm,my-s*Rm@mx
    P('GPS-track fit (similarity), windows in s after first odom: scale GPS/VIO, rms m, GPS path length')
    for a,b in ((10,60),(10,100),(10,150)):
        sel=(r[:,0]>t0+a)&(r[:,0]<t0+b)
        if sel.sum()<200: continue
        rr=r[sel][::25]; gi=np.c_[[np.interp(rr[:,0],gps[:,0],E[:,k]) for k in range(3)]].T
        s,Rm,tt=um(rr[:,1:],gi); f=(s*(Rm@rr[:,1:].T)).T+tt; rms=math.sqrt(((f-gi)**2).sum(1).mean())
        P('  %3d-%3d s: scale %.2f  rms %.1f m  path %.0f m'%(a,b,s,rms,np.linalg.norm(np.diff(gi,axis=0),axis=1).sum())); fit.append((s,rms))
res='\n'.join(out); print(res); open(run+'/results.txt','w').write(res+'\n')
csvp=os.path.expanduser('~/vio_tools/records/runs.csv'); new=not os.path.exists(csvp)
if not new and any(l.split(',')[1]==meta['tag'] for l in open(csvp).read().split('\n')[1:] if l.count(',')>1):
    print('(tag %s already in runs.csv; not appending again)'%meta['tag']); sys.exit(0)
with open(csvp,'a',newline='') as f:
    w=csv.writer(f)
    if new: w.writerow(['date','tag','bag','offset','core','config_md5','open_vins','init_grav_err_deg','init_gyro_bias','max_ba','max_bg']+['ratio_h%d'%L for L in (10,20,40,60,100,150)]+['max_vio_z','max_gps_h','diverged','fit10_60_scale','fit10_60_rms','fit10_100_scale','fit10_100_rms','notes'])
    fv=[round(fit[0][0],2),round(fit[0][1],1)] if len(fit)>0 else ['','']
    fw=[round(fit[1][0],2),round(fit[1][1],1)] if len(fit)>1 else ['','']
    w.writerow([datetime.date.today(),meta['tag'],os.path.basename(bag),meta['offset'],meta['core'],meta['config_md5'],meta['open_vins_git'],round(gerr,2),np.round(ib,4).tolist() if ib else '',round(maxba,3),round(maxbg,4)]+[round(x,2) for x in ratios]+[round(zmax,1),round(amax,1),div]+fv+fw+[''])
