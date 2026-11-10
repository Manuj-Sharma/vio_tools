import sys,glob,sqlite3,numpy as np
from scipy import signal
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
db=glob.glob(sys.argv[1]+"/*.db3")[0]; c=sqlite3.connect(f"file:{db}?mode=ro&immutable=1",uri=True)
t=[];x=[]
for (b,) in c.execute("select data from messages order by timestamp"):
    m=deserialize_message(bytes(b),Imu); t.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
    x.append([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
t=np.array(t)-t[0];x=np.array(x); s=(t>0.5)&(t<29.75); t=t[s];x=x[s]
tk=np.rint((t-t[0])*1000); g=np.arange(tk[0],tk[-1]+1); X=np.stack([np.interp(g,tk,x[:,k]) for k in range(6)],1); T=g/1000+t[0]
D=X[:,3:]-np.median(X[:2500,3:],0); dn=np.linalg.norm(D,axis=1)
# individual knock = local maxima of |a-a0| > 15 m/s^2, at least 0.2 s apart
pk,_=signal.find_peaks(dn,height=15,distance=200)
print(len(pk),"knocks (|a-a0|>15 m/s^2): t(s), peak m/s^2")
print("  ",[(round(T[i],2),round(dn[i])) for i in pk])
print("\nper knock: spectrum of the 0.25 s AFTER the peak (ring-down), strongest lines 20-450 Hz per axis, prominence >6 dB")
res={0:[],1:[],2:[]}
for i in pk:
    seg=D[i+20:i+270]
    if len(seg)<250: continue
    out=[]
    for k in range(3):
        f,P=signal.welch(seg[:,k]-seg[:,k].mean(),fs=1000,nperseg=250,noverlap=0); P=np.convolve(P,np.ones(1),mode="same")
        db_=10*np.log10(P+1e-12); m=(f>=20)&(f<=450); p,_=signal.find_peaks(db_,prominence=6)
        p=[q for q in p if m[q]]; p=sorted(p,key=lambda q:-db_[q])[:3]; res[k]+=[f[q] for q in p]; out.append(", ".join(f"{f[q]:.0f}" for q in sorted(p)))
    print(f"  t={T[i]:5.1f} s   ax: {out[0]:<18} ay: {out[1]:<18} az: {out[2]}")
print("\nfrequencies that recur (histogram of per-knock peaks, 12 Hz bins):")
for k,n in enumerate("xyz"):
    h,e=np.histogram(res[k],bins=np.arange(20,460,12)); top=np.argsort(h)[::-1][:4]
    print(f"  a{n}: "+", ".join(f"{e[j]+6:.0f} Hz (x{h[j]})" for j in sorted(top) if h[j]>0))
