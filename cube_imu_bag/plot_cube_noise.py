#!/usr/bin/env python3
"""plot_cube_noise.py: amplitude spectral density (same unit as the OpenVINS noise densities) of Cube IMU0 raw, Cube 40 Hz low-pass @333 Hz
(the stream fed to OpenVINS) and IMX, during the climb of the 200 m bag (110545, 30-110 s); + band table incl. stationary (092757 slice)."""
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
def zlp(X,fs,fc,order=4):
    pad=int(fs*2); N=len(X)+2*pad; f=np.fft.rfftfreq(N,1/fs); G=1/np.sqrt(1+(f/fc)**(2*order))
    return np.column_stack([np.fft.irfft(np.fft.rfft(np.pad(X[:,i],pad,mode='reflect'))*G,N)[pad:pad+len(X)] for i in range(3)])
def welch(x,fs,sec=4.0):
    n=int(fs*sec); w=np.hanning(n); st=n//2; P=[]
    for s in range(0,len(x)-n+1,st): seg=x[s:s+n]; P.append(np.abs(np.fft.rfft((seg-seg.mean())*w))**2)
    P=np.mean(P,axis=0)*2/(fs*(w**2).sum()); return np.fft.rfftfreq(n,1/fs),np.sqrt(P)
def streams(tag,w0,w1):
    A=np.load('align_%s.npz'%tag); Rm,CLK=A['R'],float(A['CLK']); C=np.load('cube_%s.npz'%tag); I=np.load('imx_%s.npz'%tag)['I']; t0=I[0,0]
    fs=(len(C['tg'])-1)/(C['tg'][-1]-C['tg'][0]); out={}
    gr=np.arange(t0+w0,t0+w1,1/fs); raw={'acc':np.column_stack([np.interp(gr,C['ta'],C['a'][:,i]) for i in range(3)])@Rm.T,'gyr':np.column_stack([np.interp(gr,C['tg'],C['g'][:,i]) for i in range(3)])@Rm.T}
    g3=np.arange(t0+w0,t0+w1,0.003); H=I[:,1]+CLK
    for k,src,tt,c0 in (('acc',C['a'],C['ta'],5),('gyr',C['g'],C['tg'],2)):
        lp=zlp(src,fs,40.); out[k]={'Cube IMU0 raw (1989 Hz)':(raw[k],fs),'Cube 40 Hz low-pass, 333 Hz (fed to OpenVINS)':(np.column_stack([np.interp(g3,tt,lp[:,i]) for i in range(3)])@Rm.T,1000/3.),
                'IMX-5 (333 Hz)':(np.column_stack([np.interp(g3,H,I[:,c0+i]) for i in range(3)]),1000/3.)}
    return out
bands=[(1,5),(5,15),(15,40),(40,100),(100,160)]
def table(S,title,sec):
    print(title,'| median ASD in bands',bands,'Hz')
    for k,u in (('acc','m/s2/rtHz'),('gyr','rad/s/rtHz')):
        for nm,(x,fs) in S[k].items():
            for i,ax in enumerate('XYZ'):
                f,a=welch(x[:,i],fs,sec); print('  %s %s %-46s'%(k,ax,nm),' '.join('%.1e'%np.median(a[(f>=lo)&(f<hi)]) if hi<=fs/2+1 else '   -   ' for lo,hi in bands),u)
F=streams('110545',30,110); table(F,'FLIGHT 110545 climb 30-110 s',4.0)
Q=streams('ctrl',1,13); table(Q,'STATIONARY 092757 slice 121-133 s (unarmed)',2.0)
col={'Cube IMU0 raw (1989 Hz)':'#9aa0a6','Cube 40 Hz low-pass, 333 Hz (fed to OpenVINS)':'#2a78d6','IMX-5 (333 Hz)':'#eb6834'}
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.color':'#e6e5e1','grid.linewidth':.6})
fig,ax=plt.subplots(2,3,figsize=(16,8.5),sharex=True,sharey='row',constrained_layout=True)
for r,(k,nm,u,refs,yl) in enumerate((('acc','Accelerometer','m/s² / √Hz',[(0.2,'config now 0.2'),(0.05,'test 0.05'),(0.02,'test 0.02')],(1e-4,3)),('gyr','Gyroscope','rad/s / √Hz',[(1.4e-3,'config 0.0014')],(1e-5,0.1)))):
    for i,a_ in enumerate('XYZ'):
        a=ax[r,i]
        for name,(x,fs) in F[k].items():
            f,s=welch(x[:,i],fs); a.loglog(f[1:],s[1:],color=col[name],lw=1.0 if 'raw' in name else 1.5,label=name,zorder=2 if 'raw' in name else 3)
        for v,lab in refs:
            a.axhline(v,color='#444',lw=.9,ls='--',zorder=1)
            if i==2: a.text(900,v*1.12,lab,ha='right',va='bottom',fontsize=9,color='#444')
        a.axvline(40,color='#888',lw=.7,ls=':'); a.set_xlim(0.5,1000); a.set_ylim(*yl); a.set_title('%s %s'%(nm,a_),loc='left',fontsize=11)
        if r==1: a.set_xlabel('Frequency (Hz)')
        if i==0: a.set_ylabel('Noise density  '+u)
h,l=ax[0,0].get_legend_handles_labels(); fig.legend(h,l,loc='upper center',ncol=3,frameon=False,bbox_to_anchor=(0.5,1.045))
fig.suptitle('IMU noise density in flight: Cube vs IMX (200 m bag 110545, climb 30–110 s; dashed = OpenVINS noise settings, dotted = 40 Hz cutoff)',y=1.08,x=0.01,ha='left',fontsize=12)
out='/home/unmannd/Manuj/cube_logs_nov3-4_2026/figures/cube_imu_noise_density_110545.png'; fig.savefig(out,dpi=110,bbox_inches='tight'); print(out)
