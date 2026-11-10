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
print("raw max |accel| per axis:",np.abs(x[:,3:]).max(0).round(1),"m/s^2 ; max|gyro|",np.abs(x[:,:3]).max(0).round(2))
for k,n in enumerate("xyz"):
    v=x[:,3+k]; print(f" a{n}: n at max value {int((np.abs(v)>=np.abs(v).max()-1e-6).sum())}, top 5 distinct |values|: {np.unique(np.abs(v).round(3))[-5:]}")
sel=(t>0.5)&(t<29.75); t=t[sel];x=x[sel]
tk=np.rint((t-t[0])*1000); g=np.arange(tk[0],tk[-1]+1); X=np.stack([np.interp(g,tk,x[:,k]) for k in range(6)],1); T=g/1000+t[0]
base=np.median(X[:2500],0); d=X[:,3:]-base[3:]; dn=np.linalg.norm(d,axis=1)
idx=np.where(dn>1.0)[0]; cl=[]
for i in idx:
    if cl and i-cl[-1][1]<700: cl[-1][1]=i
    else: cl.append([i,i])
print(len(cl),"knock clusters (|a-a0|>1 m/s^2, merged within 0.7 s):")
print("  t(s)  dur(s) peak|a|(m/s2)  peak axis  f90  f99  ringdown peak freq (accel, 20-300Hz)")
for a,b in cl:
    a0=max(0,a-100); b0=min(len(X),b+400); seg=d[a0:b0]; pk=np.abs(seg).max(0); ax=np.argmax(pk)
    f,P=signal.welch(seg-seg.mean(0),fs=1000,nperseg=min(256,len(seg)),axis=0); Pt=P.sum(1); cs=np.cumsum(Pt)/Pt.sum()
    m=(f>20)&(f<300); fp=f[m][np.argmax(Pt[m])]
    print(f"  {T[a]:6.1f}  {(b-a)/1000:5.2f}   {pk.max():8.1f}      a{'xyz'[ax]}      {f[np.searchsorted(cs,.9)]:4.0f} {f[np.searchsorted(cs,.99)]:4.0f}   {fp:5.0f}")
print("\nknock windows (accel, all 3 axes): power above/below 150 Hz as % of RAW knock-window power, after each causal filter")
print("  raw: below 150 Hz %.1f%%, above (would alias at 300 Hz out) %.1f%%" % tuple(
    (lambda f,P:(100*P[f<150].sum()/P.sum(),100*P[f>=150].sum()/P.sum()))(*(lambda L:(L[0],sum(L[1])))(
    [signal.welch(d[a:b][:,0],fs=1000,nperseg=256)[0],[signal.welch(d[a:b][:,k]-d[a:b][:,k].mean(),fs=1000,nperseg=256)[1] for k in range(3)] ])) for a,b in [(cl[0][0]-100,cl[-1][1]+400)])[0])
w=(cl[0][0]-100,cl[-1][1]+400)
def pw(Y):
    P=0
    for k in range(3): f,p=signal.welch(Y[w[0]:w[1],k]-Y[w[0]:w[1],k].mean(),fs=1000,nperseg=256); P=P+p
    return f,P
f,P0=pw(d); T0=P0.sum()
print(" ord  fc   kept<150 Hz   aliasing>150 Hz   (both % of raw total)")
for o,fc in [(2,50),(2,75),(2,100),(3,75),(3,100),(4,75),(4,100)]:
    sos=signal.butter(o,fc,btype="low",fs=1000,output="sos"); Y=np.stack([signal.sosfilt(sos,d[:,k]) for k in range(3)],1)
    f,P=pw(Y); print(f" {o:>3} {fc:>4}   {100*P[f<150].sum()/T0:9.1f}      {100*P[f>=150].sum()/T0:9.2f}")
