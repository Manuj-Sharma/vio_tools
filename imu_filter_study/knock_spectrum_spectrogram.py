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
t=np.array(t)-t[0];x=np.array(x); s=(t>0.5)&(t<29.75); t=t[s];x=x[s]
tk=np.rint((t-t[0])*1000); g=np.arange(tk[0],tk[-1]+1); X=np.stack([np.interp(g,tk,x[:,k]) for k in range(6)],1); T=g/1000+t[0]
base=np.median(X[:2500],0); D=X-base; dn=np.linalg.norm(D[:,3:],axis=1)
# knock = +-0.15 s around each sample with |a-a0|>3 ; still = 1 s away from anything >0.3
big=dn>3; kn=np.convolve(big.astype(float),np.ones(301),mode="same")>0
act=np.convolve((dn>0.3).astype(float),np.ones(2001),mode="same")>0; st=~act
print(f"knock ticks {kn.sum()} ({kn.sum()/1000:.1f} s), still ticks {st.sum()} ({st.sum()/1000:.1f} s)")
def runs(m):
    o=[];i=0
    while i<len(m):
        if m[i]:
            j=i
            while j<len(m) and m[j]: j+=1
            o.append((i,j)); i=j
        else: i+=1
    return o
def psd(m,ch,nper=128):
    P=0;n=0
    for a,b in runs(m):
        if b-a<nper: continue
        f,p=signal.welch(X[a:b,ch]-X[a:b,ch].mean(),fs=1000,nperseg=nper); P=P+p*(b-a); n+=b-a
    return f,P/n
N=["gx","gy","gz","ax","ay","az"]
fig,ax=plt.subplots(3,2,figsize=(15,12))
for col,ch in enumerate((5,3)):
    f,Pk=psd(kn,ch); f,Ps=psd(st,ch)
    A=ax[0,col]; A.semilogy(f,Pk,label="knock windows"); A.semilogy(f,Ps,label="still"); A.set_title(f"{N[ch]} PSD"); A.set_xlabel("Hz"); A.grid(alpha=.3,which="both"); A.legend()
    A=ax[1,col]; A.plot(f,10*np.log10(Pk/Ps)); A.set_title(f"{N[ch]}: knock/still ratio (dB)"); A.set_xlabel("Hz"); A.grid(alpha=.3)
    pk,_=signal.find_peaks(10*np.log10(Pk/Ps),prominence=3); top=pk[np.argsort((Pk/Ps)[pk])[::-1][:8]]
    print(N[ch],"knock/still peaks (Hz, dB):",[(round(float(f[i])),round(float(10*np.log10(Pk[i]/Ps[i])))) for i in sorted(top)])
    print("   knock PSD max (20-500 Hz) at",round(float(f[(f>=20)][np.argmax(Pk[f>=20])])),"Hz")
    A=ax[2,col]
    ff,tt,Sxx=signal.spectrogram(X[:,ch]-X[:,ch].mean(),fs=1000,nperseg=128,noverlap=96)
    im=A.pcolormesh(tt+T[0],ff,10*np.log10(Sxx+1e-14),shading="auto",vmin=-80,vmax=10); A.set_ylim(0,500); A.set_title(f"{N[ch]} spectrogram (dB)"); A.set_xlabel("s"); A.set_ylabel("Hz"); fig.colorbar(im,ax=A)
fig.tight_layout(); fig.savefig(sys.argv[2],dpi=100)
