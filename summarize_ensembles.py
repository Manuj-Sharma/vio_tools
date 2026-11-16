#!/usr/bin/env python3
"""summarize_ensembles.py <outfile.md> <prefix> [<prefix> ...] -- table of climb ratios (VIO height / GPS height) per run and min/mean/max per ensemble.
Reads results.txt of ~/rosbags/claude_tmp/runs_<date>/<prefix>_o*/ . Ratio columns: GPS height first reaching 10/20/40/60 m."""
import sys,glob,os,re,numpy as np
out=sys.argv[1]; prefixes=sys.argv[2:]; R=os.path.expanduser('~/rosbags/claude_tmp/runs_20261106/')
L=['# Ensemble summary (straight-climb bag vio_20261103_092757, start offsets 126.00-126.17 s)','',
   'Climb ratio = VIO height / flight-controller height when the FC height first reaches 10/20/40/60 m (1.00 = correct scale).','']
agg=[]
for pf in prefixes:
    runs=sorted(glob.glob(R+pf+'_o*/results.txt'))
    L.append('## '+pf); L.append(''); L.append('| offset | 10 m | 20 m | 40 m | 60 m | max VIO z / GPS max | diverged |'); L.append('|---|---|---|---|---|---|---|')
    rat=[]
    for f in runs:
        t=open(f).read(); m=re.search(r'10:[\d.]+\(([\d.]+)\)\s+20:[\d.]+\(([\d.]+)\)\s+40:[\d.]+\(([\d.]+)\)\s+60:[\d.]+\(([\d.]+)\)(?:\s+100:(?:n/a|[\d.]+\(([\d.]+)\)))?(?:\s+150:(?:n/a|[\d.]+\(([\d.]+)\)))?',t)
        z=re.search(r'max VIO z ([\d.]+) m vs max GPS height ([\d.]+) m',t); dv=re.search(r'diverged\([^)]*\): (\w+)',t)
        off=os.path.basename(os.path.dirname(f)).split('_o')[-1]
        if m:
            v=[float(x) for x in m.groups()[:4]]; ext=[m.group(5),m.group(6)]; rat.append(v); L.append('| %s | %.2f | %.2f | %.2f | %.2f | %s | %s |'%(off,*v,('%s / %s'%z.groups() if z else '-')+('  (100 m: %s, 150 m: %s)'%(ext[0] or '-',ext[1] or '-') if (ext[0] or ext[1]) else ''),dv.group(1) if dv else '-'))
        else: L.append('| %s | (not scored) | | | | | |'%off)
    if rat:
        a=np.array(rat); L.append('| **min** | %.2f | %.2f | %.2f | %.2f | | |'%tuple(a.min(0))); L.append('| **mean** | %.2f | %.2f | %.2f | %.2f | | |'%tuple(a.mean(0))); L.append('| **max** | %.2f | %.2f | %.2f | %.2f | | |'%tuple(a.max(0)))
        agg.append((pf,a[:,3].mean(),a[:,3].min(),a[:,3].max(),len(a)))
    L.append('')
L.append('## Overview: climb ratio at 60 m'); L.append(''); L.append('| ensemble | runs | min | mean | max |'); L.append('|---|---|---|---|---|')
for pf,mu,lo,hi,n in agg: L.append('| %s | %d | %.2f | %.2f | %.2f |'%(pf,n,lo,mu,hi))
open(out,'w').write('\n'.join(L)+'\n'); print('\n'.join(L[-(len(agg)+5):]))
