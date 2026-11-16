#!/usr/bin/env python3
"""align.py: estimate Cube(IMU0 raw, log 28) -> IMX time lag and rotation on bag vio_20261103_092757_ctrl. Saves align.npz + imx_ctrl.npz."""
import numpy as np, sqlite3, glob, sys
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
BAG,RAW,OFF,TAG=sys.argv[1],sys.argv[2],float(sys.argv[3]),sys.argv[4]   # bag dir, cube raw npz, Cube-minus-bag clock offset, output tag
rows=[]
for p in sorted(glob.glob(BAG+'/*.db3')):
    c=sqlite3.connect('file:%s?mode=ro'%p,uri=True); tid=[i for i,n in c.execute("select id,name from topics") if n=='/imu0'][0]
    for ts,d in c.execute("select timestamp,data from messages where topic_id=? order by timestamp",(tid,)):
        m=deserialize_message(d,Imu); rows.append((ts*1e-9,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z))
I=np.array(rows); R,Hraw=I[:,0],I[:,1]; CLK=np.median(R-Hraw); H=Hraw+CLK; np.savez('imx_%s.npz'%TAG,I=I,CLK=CLK)
print('header clock offset (rx-header median) %.4f'%CLK)
dh=np.diff(H); print('IMX n',len(I),'rx span %.3f..%.3f'%(R[0],R[-1]),'header-rx median %.4f std %.4f'%(np.median(H-R),np.std(H-R)),'| header dt ms quantiles',np.round(np.quantile(dh,[0,.01,.5,.99,1])*1e3,2))
Z=np.load(RAW); tref,uref=Z['ref']
def reg(a,name):
    t=uref+(a[:,0]-tref)-OFF; m=(t>R[0]-5)&(t<R[-1]+5); t=t[m]; print(name,'cube covers bag s %.2f..%.2f'%(t[0]-R[0],t[-1]-R[0])); x=a[m,1:4]; k=np.arange(len(t)); p=np.polyfit(k,t,1); res=t-np.polyval(p,k)
    print(name,'n',len(t),'rate %.2f Hz'%(1/p[0]),'residual ms: max %.2f, p1 %.2f p99 %.2f, slow drift(1s medians) range %.2f'%(np.abs(res).max()*1e3,*np.quantile(res,[.01,.99])*1e3,np.ptp([np.median(r) for r in np.array_split(res,100)])*1e3),'| dSampleUS us uniq',np.unique(np.round(np.diff(a[m,0])*1e6),return_counts=True)[0][:12])
    B=200; nb=len(res)//B; cm=np.median(res[:nb*B].reshape(nb,B),axis=1); ck=np.arange(nb)*B+B/2; sm=np.interp(k,ck,cm)   # follow dropped-sample steps / drift (0.1 s medians)
    print(name,'after local correction: residual ms p1 %.2f p99 %.2f; total slow correction range %.1f ms'%(*np.quantile(res-sm,[.01,.99])*1e3,np.ptp(cm)*1e3))
    return np.polyval(p,k)+sm,x,1/p[0],t
tg,g,fg,tg_raw=reg(Z['gyr'],'gyr'); ta,a,fa,ta_raw=reg(Z['acc'],'acc')
np.savez('cube_%s.npz'%TAG,tg=tg,g=g,ta=ta,a=a,tg_raw=tg_raw,ta_raw=ta_raw)
FS=1000.; grid=np.arange(max(H[0],tg[0])+2,min(H[-1],tg[-1])-2,1/FS)
# common 1 kHz grid, low-pass 8 Hz
def zlp(X,fs,fc,order=4):
    pad=int(fs*2); N=len(X)+2*pad; f=np.fft.rfftfreq(N,1/fs); G=1/np.sqrt(1+(f/fc)**(2*order))
    return np.column_stack([np.fft.irfft(np.fft.rfft(np.pad(X[:,i],pad,mode='reflect'))*G,N)[pad:pad+len(X)] for i in range(X.shape[1])])
lp=lambda t,x: zlp(np.column_stack([np.interp(grid,t,x[:,i]) for i in range(3)]),FS,8.)
gi=lp(H,I[:,2:5]); gc=lp(tg,g); ai=lp(H,I[:,5:8]); ac=lp(ta,a)
def kabsch(P,Q):  # R with Q ~ R P
    U,S,Vt=np.linalg.svd((Q-0).T@(P-0)); d=np.sign(np.linalg.det(U@Vt)); return U@np.diag([1,1,d])@Vt
def lag_of(x,y,maxl=400):  # y(t) ~ x(t-lag): lag>0 => IMX later
    n=len(x); best=[]
    for L in range(-maxl,maxl+1):
        if L>=0: c=np.sum(x[:n-L]*y[L:])
        else: c=np.sum(x[-L:]*y[:n+L])
        best.append(c)
    best=np.array(best); k=np.argmax(best); 
    if 0<k<len(best)-1: y0,y1,y2=best[k-1:k+2]; k=k+0.5*(y0-y2)/(y0-2*y1+y2)
    return (k-maxl)/FS
Rm=np.eye(3); lag=0
for it in range(3):
    sh=int(round(lag*FS)); P=gc[400:-400]; Q=gi[400+sh:len(gi)-400+sh]; Rm=kabsch(P,Q); gr=gc@Rm.T
    lag=lag_of((gr**2).sum(1)**.5 if it==0 else gr[:,2],(gi**2).sum(1)**.5 if it==0 else gi[:,2])
    lags=[lag_of(gr[:,i],gi[:,i]) for i in range(3)]; lag=np.median(lags); print('iter',it,'lag s per axis',np.round(lags,4),'R\n',np.round(Rm,4))
sh=int(round(lag*FS)); gr=(gc@Rm.T)[400:-400]; Q=gi[400+sh:len(gi)-400+sh]
print('gyro after align: corr per axis',[round(np.corrcoef(gr[:,i],Q[:,i])[0,1],4) for i in range(3)],'rms diff',np.round((gr-Q).std(0),4),'rms signal',np.round(Q.std(0),4),'mean diff (bias)',np.round((Q-gr).mean(0),5))
ar=(ac@Rm.T)[400:-400]; Qa=ai[400+sh:len(ai)-400+sh]
print('accel after same R: corr',[round(np.corrcoef(ar[:,i],Qa[:,i])[0,1],4) for i in range(3)],'mean IMX',np.round(Qa.mean(0),3),'mean Cube->IMX',np.round(ar.mean(0),3),'rms diff',np.round((ar-Qa).std(0),3),'rms signal',np.round(Qa.std(0),3))
# drift of lag across the bag
n=len(gi); WT=[];WL=[]
for s_ in range(0,n-20000,max(20000,(n-20000)//24)):
    xs=(gc@Rm.T)[s_:s_+20000]; ys=gi[s_:s_+20000]; l3=[lag_of(xs[:,i],ys[:,i]) for i in (2,0,1)]; WT.append(grid[s_]+10-H[0]); WL.append(np.median(l3)); print('  window %5.1f s lag z %.4f x %.4f y %.4f'%(WT[-1],*l3))
WT=np.array(WT);WL=np.array(WL); lagp=np.polyfit(WT,WL,1); r_=WL-np.polyval(lagp,WT); ok=np.abs(r_)<3*np.median(np.abs(r_))+1e-3; lagp=np.polyfit(WT[ok],WL[ok],1)
print('lag model: %.4f s + %.3e s/s * t ; residual ms'%(lagp[1],lagp[0]),np.round((WL-np.polyval(lagp,WT))*1e3,1))
np.savez('align_%s.npz'%TAG,R=Rm,lag=lag,CLK=CLK,lagp=lagp,H0=H[0])
