import sys,glob,sqlite3,numpy as np
from scipy import signal
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
db=glob.glob(sys.argv[1]+"/*.db3")[0]; c=sqlite3.connect(f"file:{db}?mode=ro&immutable=1",uri=True)
t=[];x=[]
for (b,) in c.execute("select data from messages order by timestamp"):
    m=deserialize_message(bytes(b),Imu); t.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
    x.append([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
t=np.array(t)-t[0];x=np.array(x); s=t>3.55; t=t[s];x=x[s]
tk=np.rint((t-t[0])*1000); g=np.arange(tk[0],tk[-1]+1); X=np.stack([np.interp(g,tk,x[:,k]) for k in range(6)],1); T=g/1000
base=X[:2500].mean(0)
lifts=[9.4,21.7,33.3,56.7,67.2]
print("clean lifts only (+-0.4 s around the first strong-accel moment of each; delay removed, error vs raw as % of lift rms; peak = filtered/raw |a-a0|)")
print(" ord  fc   gd_ms   peak_ratio   err_% (accel)   err_% (gyro)")
for o,fc in [(2,20),(2,30),(2,40),(2,50),(2,75),(2,100),(4,50),(4,75),(4,100)]:
    sos=signal.butter(o,fc,btype="low",fs=1000,output="sos")
    Y=np.stack([signal.sosfilt(sos,X[:,k],zi=signal.sosfilt_zi(sos)*X[0,k])[0] for k in range(6)],1)
    f=np.linspace(.01,300,30000); h=signal.sosfreqz(sos,worN=f,fs=1000)[1]; gd=-np.gradient(np.unwrap(np.angle(h)),2*np.pi*f)[0]; sh=int(round(gd*1000))
    pk=[];ea=[];eg=[]
    for tl in lifts:
        i=int((tl-0.4)*1000); j=int((tl+0.4)*1000); d=X[i:j]-base; yd=Y[i+sh:j+sh]-base
        pk.append(np.max(np.linalg.norm(yd[:,3:],axis=1))/np.max(np.linalg.norm(d[:,3:],axis=1)))
        ea.append(100*np.sqrt(np.mean((yd[:,3:]-d[:,3:])**2)/np.mean(d[:,3:]**2))); eg.append(100*np.sqrt(np.mean((yd[:,:3]-d[:,:3])**2)/np.mean(d[:,:3]**2)))
    print(f" {o:>3} {fc:>4} {gd*1e3:6.2f}   {np.mean(pk):8.3f}   {np.mean(ea):10.1f}   {np.mean(eg):10.1f}")
