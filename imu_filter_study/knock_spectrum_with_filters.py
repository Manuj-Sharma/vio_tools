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
tk=np.rint((t-t[0])*1000); g=np.arange(tk[0],tk[-1]+1); X=np.stack([np.interp(g,tk,x[:,k]) for k in range(6)],1)
D=X-np.median(X[:2500],0); dn=np.linalg.norm(D[:,3:],axis=1)
kn=np.convolve((dn>3).astype(float),np.ones(301),mode="same")>0
runs=[];i=0
while i<len(kn):
    if kn[i]:
        j=i
        while j<len(kn) and kn[j]: j+=1
        runs.append((i,j)); i=j
    else: i+=1
cfg=[(2,50),(2,75),(2,100),(4,75),(4,100)]; col=dict(zip(cfg,["tab:green","tab:blue","tab:orange","tab:purple","tab:red"]))
def psd(Y,ch):
    P=0;n=0
    for a,b in runs:
        if b-a<128: continue
        f,p=signal.welch(Y[a:b,ch]-Y[a:b,ch].mean(),fs=1000,nperseg=128); P=P+p*(b-a); n+=b-a
    return f,P/n
fig,ax=plt.subplots(2,2,figsize=(15,9))
for k,(ch,nm) in enumerate(((5,"az"),(3,"ax"))):
    f,P0=psd(X,ch); A=ax[0,k]; A.semilogy(f,P0,"k",lw=1.5,label="raw knock windows")
    for (o,fc) in cfg:
        sos=signal.butter(o,fc,btype="low",fs=1000,output="sos"); Y=np.stack([signal.sosfilt(sos,X[:,q],zi=signal.sosfilt_zi(sos)*X[0,q])[0] for q in range(6)],1)
        f,P=psd(Y,ch); A.semilogy(f,P,color=col[(o,fc)],lw=1,label=f"order {o}, {fc} Hz")
    A.axvline(150,color="r",ls=":"); A.text(152,P0.max()*0.5,"150 Hz\n(output Nyquist\nat 300 Hz)",color="r",fontsize=8,va="top")
    A.set_ylim(P0.min()/1e3,P0.max()*3); A.set_title(f"{nm}: knock-window PSD, raw vs filtered (1 kHz, before resampling)"); A.set_xlabel("Hz"); A.grid(alpha=.3,which="both"); A.legend(fontsize=8)
    A=ax[1,k]; fr=np.linspace(0,500,2000)
    for (o,fc) in cfg:
        sos=signal.butter(o,fc,btype="low",fs=1000,output="sos"); h=signal.sosfreqz(sos,worN=fr,fs=1000)[1]; A.plot(fr,20*np.log10(abs(h)+1e-12),color=col[(o,fc)],label=f"order {o}, {fc} Hz")
    for fl,lab in ((62,"62"),(141,"141"),(211,"211"),(328,"328"),(367,"367")) if nm=="az" else ((70,"70"),(141,"141"),(328,"328")):
        A.axvline(fl,color="gray",ls="--",lw=.6); A.text(fl+2,-3,lab,fontsize=8,color="gray")
    A.axvline(150,color="r",ls=":"); A.set_ylim(-80,3); A.set_title(f"filter magnitude (dB); grey = {nm} knock humps"); A.set_xlabel("Hz"); A.grid(alpha=.3); A.legend(fontsize=8)
fig.tight_layout(); fig.savefig(sys.argv[2],dpi=100)
