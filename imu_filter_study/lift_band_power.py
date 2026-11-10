import sys,glob,sqlite3,numpy as np
from scipy import signal
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
db=glob.glob(sys.argv[1]+"/*.db3")[0]; c=sqlite3.connect(f"file:{db}?mode=ro&immutable=1",uri=True)
t=[];x=[]
for (b,) in c.execute("select data from messages order by timestamp"):
    m=deserialize_message(bytes(b),Imu); t.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
    x.append([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
t=np.array(t)-t[0];x=np.array(x)
seg=t>3.55; t=t[seg];x=x[seg]
tk=np.rint((t-t[0])*1000); g=np.arange(tk[0],tk[-1]+1); X=np.stack([np.interp(g,tk,x[:,k]) for k in range(6)],1)
am=np.linalg.norm(X[:,3:],axis=1); base=np.median(am)
big=np.abs(am-base)>3; ei=np.convolve(big.astype(float),np.ones(501),mode="same")>0
print("lift ticks",ei.sum(),"of",len(g))
for name,sel in (("lift windows",ei),("between lifts (still)",~ei)):
    for ch,lab in ((slice(0,3),"gyro"),(slice(3,6),"accel")):
        Y=X[sel][:,ch]; Y=Y-Y.mean(0)
        f,P=signal.welch(Y,fs=1000,nperseg=256,axis=0); P=P.sum(1); tot=P.sum()
        fr=lambda lo,hi:100*P[(f>=lo)&(f<hi)].sum()/tot
        print(f"{name:22s} {lab:5s} power %: <20 Hz {fr(0,20):5.1f} | 20-50 {fr(20,50):5.1f} | 50-100 {fr(50,100):5.1f} | 100-150 {fr(100,150):5.1f} | >150 {fr(150,501):5.1f}")
