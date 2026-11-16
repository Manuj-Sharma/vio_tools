#!/usr/bin/env bash
# make_export.sh -- (re)build ~/vio_export_20261106 (+ .tgz): configs, calibration, code patch + patched sources, scripts, results.
set -e
X=$HOME/vio_export_20261106; E=$HOME/rosbags/claude_tmp/bagE; rm -rf $X $X.tgz; mkdir -p $X/{configs,calibration,code,scripts,results}
cp -r $E/cfg_frozen_cam0_px2_tf40_acc02_330hz_extoff $X/configs/A_baseline_scalar_accel0.2
cp -r $E/cfg_cam0_px2_peraxis_x01_y01_z05           $X/configs/B_peraxis_accel_x0.1_y0.1_z0.5
cp -r $E/cfg_cam0_px2_vibadapt_base002_k1.0          $X/configs/C_vibration_adaptive_base0.02_gain1.0
cp -r $E/cfg_cam0_px2_vibadapt_base002_k2.0          $X/configs/D_vibration_adaptive_base0.02_gain2.0
chmod -R u+w $X/configs; rm -f $X/configs/*/FROZEN.md
cp $HOME/rosbags/claude_tmp/calib_extract_20261106/cam0_nt/cam0_imu_nt-camchain-imucam.yaml $X/calibration/; cp $HOME/rosbags/claude_tmp/calib_extract_20261106/cam0_nt/cam0_imu_nt-results-imucam.txt $X/calibration/
cd ~/open_vins_axis; git diff 1e24f52 > $X/code/per-axis-accel-noise.patch
tar -czf $X/code/open_vins_patched_sources.tgz ov_core ov_init ov_msckf
echo "base: git commit 1e24f52 (branch bagC-feature-diagnostics of ~/open_vins_debug, upstream rpng/open_vins 6948812 + local commits). patch: per-axis-accel-noise.patch (uncommitted work on branch per-axis-accel-noise in worktree ~/open_vins_axis)." > $X/code/BASE.txt
cp ~/vio_tools/{run_bag.sh,run_ensemble.sh,score_run.py,trace_run.py,summarize_ensembles.py,quiet_init_values.py} $X/scripts/; cp ~/vio_tools/gt_oneshot_pub.py $X/scripts/
cp ~/vio_tools/init_values/vio_20261103_092757_quiet.yaml $X/results/ 2>/dev/null || true
python3 ~/vio_tools/summarize_ensembles.py $X/results/ensemble_summary.md ensPA_newbuild_scalar02 ensPB_newbuild_peraxis_z05 ensV1_vibadapt_k1 ensV2_vibadapt_k2 > /tmp/claude-1000/export_overview.txt 2>&1 || true
python3 - <<'PY'
import os,re
X=os.path.expanduser('~/vio_export_20261106'); s=open(X+'/results/ensemble_summary.md').read()
ov=s.split('## Overview')[-1] if '## Overview' in s else '(no results yet)'
rows=[l for l in ov.split('\n') if l.startswith('| ens')]
names={'ensPA':'A (baseline, scalar accel noise 0.2)','ensPB':'B (per-axis accel x0.1 y0.1 z0.5)','ensV1':'C (vibration-adaptive, gain 1.0)','ensV2':'D (vibration-adaptive, gain 2.0)'}
best=None
for r in rows:
    c=[x.strip() for x in r.strip('|').split('|')]
    try: mean=float(c[3]); nruns=int(c[1])
    except: continue
    if nruns<3: continue
    if best is None or abs(mean-1.0)<abs(best[1]-1.0): best=(names.get(c[0][:5],c[0]),mean,c[0])
readme=open(os.path.expanduser('~/vio_tools/export_README.md')).read()
readme=readme.replace('@@BEST@@','%s (mean climb ratio at 60 m = %.2f; 1.00 is perfect; above 1 = VIO climbs too fast, below 1 = too slow; only ensembles with 3 or more runs are ranked)'%(best[0],best[1]) if best else '(no ensemble with 3 or more runs yet)').replace('@@OVERVIEW@@','## Overview'+ov)
open(X+'/README.md','w').write(readme)
PY
cd $HOME && tar -czf vio_export_20261106.tgz vio_export_20261106 && ls -la vio_export_20261106.tgz | awk '{print $5/1e6" MB"}'
