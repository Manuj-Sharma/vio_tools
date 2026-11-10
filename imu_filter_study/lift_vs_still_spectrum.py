import sys,glob,sqlite3,numpy as np
from scipy import signal
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
db=glob.glob(sys.argv[1]+"/*.db3")[0]; c=sqlite3.connect(f"file:{db}?mode=ro&immutable=1",uri=True)
t=[];x=[]
for (b,) in c.execute("select data from messages order by timestamp"):
    m=deserialize_message(bytes(b),Imu); t.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
    x.append([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
t=np.array(t)-t[0];x=np.array(x); s=t>3.55; t=t[s];x=x[s]
tk=np.rint((t-t[0])*1000); g=np.arange(tk[0],tk[-1]+1); X=np.stack([np.interp(g,tk,x[:,k]) for k in range(6)],1); T=g/1000
am=np.linalg.norm(X[:,3:],axis=1); base=np.median(am[:2500]); dev=np.abs(am-base)
# lifts = contiguous clusters of |a|-g > 3, merged if within 0.5 s
idx=np.where(dev>3)[0]; cl=[]; 
for i in idx:
    if cl and i-cl[-1][1]<500: cl[-1][1]=i
    else: cl.append([i,i])
print(len(cl),"lift clusters at t(s) =",[round(T[a],1) for a,b in cl])
# 'still' = rolling 0.3 s std of |a| tiny AND of gyro tiny, away from lifts by 1 s
w=300; r=lambda v:np.sqrt(np.convolve(v**2,np.ones(w)/w,mode="same")-np.convolve(v,np.ones(w)/w,mode="same")**2)
quiet=(r(am)<0.06)&(r(np.linalg.norm(X[:,:3],axis=1))<0.01)
for a,b in cl: quiet[max(0,a-1000):b+1000]=False
print("still ticks",quiet.sum(),"s",quiet.sum()/1000)
def psd(sel_segments,ch,nper=256):
    P=0;n=0
    for a,b in sel_segments:
        if b-a<nper: continue
        f,p=signal.welch(X[a:b,ch]-X[a:b,ch].mean(),fs=1000,nperseg=nper); P=P+p*(b-a); n+=b-a
    return f,P/n
# still segments: contiguous runs of quiet
qs=[];i=0
while i<len(quiet):
    if quiet[i]:
        j=i
        while j<len(quiet) and quiet[j]: j+=1
        qs.append((i,j)); i=j
    else: i+=1
ls=[(max(0,a-250),b+250) for a,b in cl]   # lift = +-0.25 s around the strong-accel moments
N=["gx","gy","gz","ax","ay","az"]; fig,ax=plt.subplots(2,3,figsize=(16,8),sharex=True)
bands=[(0,5),(5,10),(10,20),(20,40),(40,60),(60,100),(100,150),(150,300),(300,500)]
for ch in range(6):
    f,Pl=psd(ls,ch); f,Pq=psd(qs,ch); A=ax.flat[ch]
    A.loglog(f[1:],Pl[1:],label="lift windows"); A.loglog(f[1:],Pq[1:],label="still"); A.set_title(N[ch]); A.grid(alpha=.3,which="both")
    ex=Pl-Pq; tot=ex[(f>0)&(f<500)].clip(0).sum()
    print(f"{N[ch]}: excess power (lift - still) share by band Hz: "+" | ".join(f"{lo}-{hi}: {100*ex[(f>=lo)&(f<hi)].clip(0).sum()/tot:4.1f}%" for lo,hi in bands)+f"   ; lift/still ratio @ 100 Hz {np.interp(100,f,Pl)/np.interp(100,f,Pq):.1f}x, @ 150 Hz {np.interp(150,f,Pl)/np.interp(150,f,Pq):.1f}x, @ 250 Hz {np.interp(250,f,Pl)/np.interp(250,f,Pq):.1f}x")
ax.flat[0].legend(); fig.tight_layout(); fig.savefig(sys.argv[2],dpi=100)
# per-lift: frequency below which 90/99 % of accel-dev energy lies, and 50 Hz-500 Hz share
print("per lift (accel, 3-axis sum): t(s) | peak |a-g| | f90 | f99 | share >50 Hz | share >150 Hz")
for a,b in cl:
    a0,b0=max(0,a-250),b+250; P=0
    for ch in (3,4,5):
        f,p=signal.welch(X[a0:b0,ch]-X[a0:b0,ch].mean(),fs=1000,nperseg=min(256,b0-a0)); P=P+p
    cs=np.cumsum(P)/P.sum(); print(f"  {T[a]:6.1f}  {dev[a:b+1].max():5.1f}  {f[np.searchsorted(cs,.9)]:5.0f}  {f[np.searchsorted(cs,.99)]:5.0f}  {100*P[f>50].sum()/P.sum():5.1f}%  {100*P[f>150].sum()/P.sum():5.1f}%")
