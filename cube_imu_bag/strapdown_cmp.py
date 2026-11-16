#!/usr/bin/env python3
"""strapdown_cmp.py <tag> <bag> <t_quiet0> <t_quiet1> <t_end>: IMU-only vertical velocity (no vision) for IMX vs Cube lp40 vs /fc/climb.
Attitude: gravity direction + gyro bias from the quiet window, then gyro integration. Times in s from first IMU of the bag."""
import sys, numpy as np, sqlite3, glob, struct
TAG,BAG=sys.argv[1:3]; q0,q1,te=map(float,sys.argv[3:6])
A=np.load('align_%s.npz'%TAG); Rm,CLK,lagp,H0=A['R'],float(A['CLK']),A['lagp'],float(A['H0']); C=np.load('cube_%s.npz'%TAG); I=np.load('imx_%s.npz'%TAG)['I']; t0=I[0,0]
def zlp(X,fs,fc,order=4):
    pad=int(fs*2); N=len(X)+2*pad; f=np.fft.rfftfreq(N,1/fs); G=1/np.sqrt(1+(f/fc)**(2*order))
    return np.column_stack([np.fft.irfft(np.fft.rfft(np.pad(X[:,i],pad,mode='reflect'))*G,N)[pad:pad+len(X)] for i in range(3)])
clm=[]
for p in sorted(glob.glob(BAG+'/*.db3')):
    c=sqlite3.connect('file:%s?mode=ro'%p,uri=True); ids={n:i for i,n in c.execute("select id,name from topics")}
    if '/fc/climb' in ids: clm+=[(ts*1e-9-t0,struct.unpack('<f',d[4:8])[0]) for ts,d in c.execute("select timestamp,data from messages where topic_id=? and timestamp<? order by timestamp",(ids['/fc/climb'],int((t0+te+2)*1e9)))]
clm=np.array(clm)
def strap(t,g,a,name):
    m=(t>=q0)&(t<te); t=t[m]; g=g[m]; a=a[m]; q=(t<q1); bg=g[q].mean(0); am=a[q].mean(0); G0=np.linalg.norm(am); up=am/G0
    xb=np.array([1.,0,0]); xw=xb-xb@up*up; xw/=np.linalg.norm(xw); R=np.vstack([xw,np.cross(up,xw),up]); v=0.; V=[]
    for k in range(1,len(t)):
        dt=t[k]-t[k-1]; w=(g[k]-bg)*dt; th=np.linalg.norm(w)
        if th>0: K=np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])/th; R=R@(np.eye(3)+np.sin(th)*K+(1-np.cos(th))*K@K)
        v+=((R@a[k])[2]-G0)*dt; V.append(v)
    return t[1:],np.array(V)
ti=I[:,1]+CLK-t0; tI,vI=strap(ti,I[:,2:5],I[:,5:8],'imx')
fs=(len(C['tg'])-1)/(C['tg'][-1]-C['tg'][0]); grid=np.arange(max(C['tg'][0],C['ta'][0])+0.3,t0+te,0.003); out={}
for nm,fc in (('cube lp40',40.),('cube raw 2 kHz',None)):
    g=zlp(C['g'],fs,fc) if fc else C['g']; a=zlp(C['a'],fs,fc) if fc else C['a']
    if fc: tt=grid; gg=np.column_stack([np.interp(grid,C['tg'],g[:,i]) for i in range(3)])@Rm.T; aa=np.column_stack([np.interp(grid,C['ta'],a[:,i]) for i in range(3)])@Rm.T
    else: tt=C['tg']; n=min(len(g),len(a)); tt=tt[:n]; gg=g[:n]@Rm.T; aa=np.column_stack([np.interp(tt,C['ta'],a[:,i]) for i in range(3)])@Rm.T
    out[nm]=strap(tt+np.polyval(lagp,tt-H0)-t0,gg,aa,nm)
print('t(s) | FC climb m/s | IMX strapdown | '+' | '.join(out))
for s in np.arange(np.ceil(q1),te,2.0):
    print('%5.1f | %6.2f | %6.2f | '%(s,np.interp(s,clm[:,0],clm[:,1]),np.interp(s,tI,vI))+' | '.join('%6.2f'%np.interp(s,*out[k]) for k in out))
