#!/usr/bin/env python3
"""make_cube_bag.py <variant> <src_bag> <tag> <dst_bag> [t_end_s]
Copy <src_bag> (up to t_end_s from its start), replacing /imu0 (IMX-5) by the Cube's raw IMU0 (ACC/GYR from the dataflash log).
Needs align.py outputs for <tag> in this folder (imx_<tag>.npz, cube_<tag>.npz, align_<tag>.npz).
variant: lp<Hz> = zero-phase 4th-order Butterworth low-pass at <Hz> on the ~1989 Hz raw stream, then sampled on a uniform 3 ms grid
         raw    = no filter, point-sampled on the same grid (aliased on purpose)
         clp<Hz> = CAUSAL 2-pole Butterworth IIR low-pass at <Hz> (one biquad, bilinear transform, run sample by sample as a real-time
                   filter would), then sampled on the same grid. Stamps are NOT shifted, so the data lag their stamps by the filter delay
                   (sqrt(2)/(2 pi fc): 5.6 ms at 40 Hz) -> add that delay to timeshift_cam_imu to compensate.
- Cube data are rotated into the IMX frame and stamped on the IMX header clock (rotation + linear lag model from align.py), so the
  IMX camera-IMU calibration and timeshift stay valid. Lever arm Cube->IMX ignored.
- Where the Cube log has no data yet (it starts at arming), the IMX stream is used with a constant offset so its mean matches the
  Cube's over the first quiet 1.5 s of overlap (vehicle is stationary there)."""
import sys, os, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu
var,SRC,TAG,DST=sys.argv[1:5]; TEND=float(sys.argv[5]) if len(sys.argv)>5 else 1e9
A=np.load('align_%s.npz'%TAG); Rm,CLK,lagp,H0=A['R'],float(A['CLK']),A['lagp'],float(A['H0']); C=np.load('cube_%s.npz'%TAG); I=np.load('imx_%s.npz'%TAG)['I']
if os.environ.get('CUBE_R'): Rm=np.load(os.environ['CUBE_R']); print('rotation override from',os.environ['CUBE_R'])  # e.g. gravity(roll/pitch)+gyro(yaw) rotation
def zlp(X,fs,fc,order=4):
    pad=int(fs*2); N=len(X)+2*pad; f=np.fft.rfftfreq(N,1/fs); G=1/np.sqrt(1+(f/fc)**(2*order))
    return np.column_stack([np.fft.irfft(np.fft.rfft(np.pad(X[:,i],pad,mode='reflect'))*G,N)[pad:pad+len(X)] for i in range(3)])
def clp(X,fs,fc):   # causal 2-pole Butterworth (direct form I), state started at the first sample so there is no start-up transient
    K=np.tan(np.pi*fc/fs); n=1/(1+np.sqrt(2)*K+K*K); b0=K*K*n; b1=2*b0; b2=b0; a1=2*(K*K-1)*n; a2=(1-np.sqrt(2)*K+K*K)*n
    Y=np.empty_like(X); x1=x2=y1=y2=X[0].copy()
    for i in range(len(X)):
        x=X[i]; y=b0*x+b1*x1+b2*x2-a1*y1-a2*y2; Y[i]=y; x2=x1; x1=x; y2=y1; y1=y
    return Y
tg,g,ta,a=C['tg'],C['g'],C['ta'],C['a']; fs=(len(tg)-1)/(tg[-1]-tg[0]); CAUSAL=var.startswith('clp'); fc=float(var[3:]) if CAUSAL else float(var[2:]) if var.startswith('lp') else None
if CAUSAL: zlp=lambda X,fs_,fc_: clp(np.ascontiguousarray(X,dtype=float),fs_,fc_)
if fc: g=zlp(g,fs,fc); a=zlp(a,fs,fc)
Hc=I[:,1]+CLK; grid=np.arange(Hc[0],min(Hc[-1],Hc[0]+TEND),0.003)
lag=np.polyval(lagp,grid-H0); tq=grid-lag
G=np.column_stack([np.interp(tq,tg,g[:,i]) for i in range(3)])@Rm.T; Ac=np.column_stack([np.interp(tq,ta,a[:,i]) for i in range(3)])@Rm.T
t_first=max(tg[0],ta[0])+0.3; nofill=tq>=t_first; pre=~nofill
if pre.any():
    fsi=(len(Hc)-1)/(Hc[-1]-Hc[0]); Xi=I[:,2:8]
    if fc and not CAUSAL: Xi=np.column_stack([zlp(I[:,2:5],fsi,fc),zlp(I[:,5:8],fsi,fc)])
    Ii=np.column_stack([np.interp(grid,Hc,Xi[:,i]) for i in range(6)]); ov=nofill&(tq<t_first+1.5)
    if CAUSAL: Ii=clp(Ii,1/0.003,fc)   # IMX has dropped samples: put it on the uniform 3 ms grid first, then the same causal filter
    dG=G[ov].mean(0)-Ii[ov,:3].mean(0); dA=Ac[ov].mean(0)-Ii[ov,3:].mean(0); G[pre]=Ii[pre,:3]+dG; Ac[pre]=Ii[pre,3:]+dA
    print('pre-arm fill with IMX for first %.2f s; offset applied gyro'%(grid[pre][-1]-grid[0]),dG.round(5),'accel',dA.round(4),'| overlap std cube',Ac[ov].std(0).round(3),'imx',Ii[ov,3:].std(0).round(3))
print(var,'out samples',len(grid),'lag %.4f..%.4f s'%(lag[0],lag[-1]),'| accel std xyz',Ac.std(0).round(3),'gyro std',G.std(0).round(4))
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=SRC,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
w=rosbag2_py.SequentialWriter(); w.open(rosbag2_py.StorageOptions(uri=DST,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
for t in r.get_all_topics_and_types(): w.create_topic(rosbag2_py.TopicMetadata(name=t.name,type=t.type,serialization_format='cdr'))
tmpl=None; k=0; n=0; tstop=int((I[0,0]+TEND)*1e9)
def flush(upto_ns):
    global k
    while k<len(grid) and int(grid[k]*1e9)<=upto_ns:
        m=Imu(); m.header.frame_id=tmpl.header.frame_id; hs=grid[k]-CLK; m.header.stamp.sec=int(hs); m.header.stamp.nanosec=int(round((hs-int(hs))*1e9))%1000000000
        m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=map(float,G[k]); m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,Ac[k])
        m.orientation=tmpl.orientation; m.orientation_covariance=tmpl.orientation_covariance; m.angular_velocity_covariance=tmpl.angular_velocity_covariance; m.linear_acceleration_covariance=tmpl.linear_acceleration_covariance
        w.write('/imu0',serialize_message(m),int(grid[k]*1e9)); k+=1
while r.has_next():
    topic,data,t=r.read_next()
    if t>tstop: break
    if topic.startswith('/cam1'): continue
    if topic=='/imu0':
        if tmpl is None: tmpl=deserialize_message(data,Imu)
        continue
    if tmpl is not None: flush(t)
    w.write(topic,data,t); n+=1
flush(10**19); print('done: other msgs',n,'imu written',k,'->',DST)
