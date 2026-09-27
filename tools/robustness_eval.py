"""Deterministic synthetic robustness evidence; no model tuning or GNSS input.

The original seven stress cases are rerun separately unchanged. This harness
adds missing fault types and acceptance/recovery evidence at 20 Hz.
"""
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/tram_odometry'))
from tram_odometry.core import Config,Estimator

# Predeclared evidence criteria, not calibrated on any bag:
# finite, monotone state time, 20 Hz samples, <=1 m/s error, <=10 m relative-distance
# deviation from clean replay, and sustained recovery within 5 s after the fault.
SCENARIOS=[
    ('clean',10.,10.),('front_spike',10.,10.1),('rear_spike',10.,10.1),
    ('front_slip_5mps',10.,15.),('moderate_disagreement',10.,15.),
    ('both_missing',10.,12.),('command_missing',10.,15.),('nan_inf',10.,12.),
    ('bad_command',10.,12.),('rear_missing_long',10.,25.),('both_slip_short',10.,12.),
    ('both_locked_short',10.,11.),('backward_header',10.,11.),
    ('forward_header_spike',10.,10.1),('both_slip_long',10.,20.)]


def simulate(name,start,end,config):
    e=Estimator(config);trace=[];events=[];errors=[];downweighted=0;reject_delta=[];accepted_after={k:0 for k in ['front','rear']}
    for tick in range(801):
        t=tick*.05;truth=8+2*math.sin(t*.1);a=.2*math.cos(t*.1)
        command=round(max(-15,min(15,(a+.05)*(1+truth/12)/1.8*15 if a>=0 else (a+.05)/1.87*15)))
        e.predict(1000+t)
        inputs=[('command',float(command))]
        if tick%2==0:inputs += [('front',truth/config.wheel_scale),('rear',truth/config.wheel_scale)]
        for kind,original in inputs:
            value=original;stamp=1000+t;fault=start<=t<end
            if fault:
                if name in ('front_spike','front_slip_5mps') and kind=='front':value+=5/config.wheel_scale
                if name=='rear_spike' and kind=='rear':value+=5/config.wheel_scale
                if name=='moderate_disagreement' and kind=='front':value+=.48/config.wheel_scale
                if name=='both_missing' and kind!='command':continue
                if name=='command_missing' and kind=='command':continue
                if name=='nan_inf' and kind!='command':value=math.nan if kind=='front' else math.inf
                if name=='bad_command' and kind=='command':value=16.
                if name=='rear_missing_long' and kind=='rear':continue
                if name in ('both_slip_short','both_slip_long') and kind!='command':value+=5/config.wheel_scale
                if name=='both_locked_short' and kind!='command':value=0.
                if name=='backward_header':stamp-=2.
                if name=='forward_header_spike' and kind=='front':stamp+=60.
            before=e.v;pv=e.pv;rej=e.rejected
            try:accepted=e.feed(kind,stamp,value)
            except Exception as exc:errors.append(repr(exc));continue
            if kind!='command' and not accepted and e.rejected>rej and stamp<=1000+t:
                reject_delta.append(abs(e.v-before))
            inferred_r=None
            if kind!='command' and accepted and 0<e.pv<pv and stamp==e.t:
                inferred_r=e.pv*pv/(pv-e.pv)
                if inferred_r>8*config.wheel_sigma**2:downweighted+=1
            if accepted and kind in accepted_after and t>=end:accepted_after[kind]+=1
            if fault or (end<=t<end+1):
                events.append(dict(time=t,kind=kind,accepted=accepted,finite_input=math.isfinite(value),velocity_before=before,velocity_after=e.v,
                    rejected=e.rejected,effective_measurement_variance=inferred_r))
        snap=e.snapshot();trace.append([t,truth,e.v,e.s,e.t,e.pv,e.ps,int(bool(snap['slip'])),int(snap['status']=='model_only'),int(snap['status']=='ok')])
    return np.array(trace),dict(exceptions=errors,rejected=e.rejected,out_of_order=e.out_of_order,
        downweighted_observed=downweighted,max_velocity_change_on_rejected_input_mps=max(reject_delta,default=None),accepted_after_fault=accepted_after,events=events)


def rerun_original(output):
    """Execute unchanged existing stress script in a temporary output layout."""
    import os
    import shutil
    import subprocess
    import tempfile
    with tempfile.TemporaryDirectory(prefix='robustness_existing_') as temp:
        stage=Path(temp)
        (stage/'tools').mkdir();(stage/'results').mkdir();(stage/'src/tram_odometry/config').mkdir(parents=True)
        shutil.copyfile(ROOT/'tools/stress_test.py',stage/'tools/stress_test.py')
        shutil.copyfile(ROOT/'src/tram_odometry/config/model.json',stage/'src/tram_odometry/config/model.json')
        env=dict(os.environ,PYTHONPATH=str(ROOT/'tools'),PYTHONNOUSERSITE='1')
        with (output/'existing_stress.log').open('w') as log:
            subprocess.run([sys.executable,str(stage/'tools/stress_test.py')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        shutil.copyfile(stage/'results/stress_tests.json',output/'existing_stress_tests.json')


def main():
    output=ROOT/'results/robustness_eval';output.mkdir(parents=True,exist_ok=True)
    rerun_original(output)
    c=Config.from_dict(json.loads((ROOT/'src/tram_odometry/config/model.json').read_text()))
    baseline,_=simulate('clean',10,10,c);rows=[];traces={};event_logs={}
    for name,start,end in SCENARIOS:
        trace,log=simulate(name,start,end,c);traces[name]=trace;event_logs[name]=log.pop('events')
        error=abs(trace[:,2]-trace[:,1]);deviation=abs(trace[:,2]-baseline[:,2]);distance=abs(trace[:,3]-baseline[:,3])
        finite=bool(np.isfinite(trace).all());recovery=None
        good=(deviation<=.2)&(trace[:,9]==1)
        for i in np.flatnonzero(trace[:,0]>=end):
            if i+21<=len(trace) and good[i:i+21].all():recovery=float(trace[i,0]-end);break
        drift=float(distance.max());max_error=float(error.max());continuous=bool(np.all(np.diff(trace[:,4])>0))
        criteria=dict(finite=finite,no_exception=not log['exceptions'],state_time_advances_each_tick=continuous,
            max_velocity_error_le_1_mps=max_error<=1,max_distance_deviation_le_10m=drift<=10,
            recovery_within_5s=recovery is not None and recovery<=5,
            sane_wheels_accepted_after_fault=all(v>0 for v in log['accepted_after_fault'].values()))
        row=dict(scenario=name,start_s=start,end_s=end,status='PASS' if all(criteria.values()) else 'FAIL',
            finite=finite,output_samples=len(trace),sample_hz=20.,state_time_advances_each_tick=continuous,
            max_velocity_error_mps=max_error,max_velocity_deviation_from_clean_mps=float(deviation.max()),
            max_relative_distance_deviation_m=drift,slip_flag_activated=bool(trace[:,7].any()),model_only_activated=bool(trace[:,8].any()),
            recovery_time_s=recovery,final_status_ok=bool(trace[-1,9]),criteria=criteria,**log)
        rows.append(row)
    # Direct paired update proves accepted but downweighted measurement influence.
    import copy
    e=Estimator(c)
    for i in range(100):
        t=1000+i*.1;e.predict(t);e.feed('front',t,8/c.wheel_scale);e.feed('rear',t,8/c.wheel_scale)
    e.predict(1010.);normal=copy.deepcopy(e);disagree=copy.deepcopy(e)
    disagree.wheels['front']=(1010.,8.8)
    before=normal.v;pv=normal.pv
    normal_ok=normal.feed('rear',1010.,8.05/c.wheel_scale);down_ok=disagree.feed('rear',1010.,8.05/c.wheel_scale)
    trust=dict(normal_accepted=normal_ok,disagreement_accepted=down_ok,normal_velocity_update=normal.v-before,
        downweighted_velocity_update=disagree.v-before,update_magnitude_ratio=abs(disagree.v-before)/abs(normal.v-before),
        nominal_R=c.wheel_sigma**2,disagreement_R=9*c.wheel_sigma**2,
        explanation='Identical predicted state and rear measurement; only stored opposing wheel differs. Code increases R ninefold.')
    old=json.loads((output/'existing_stress_tests.json').read_text())
    summary=dict(test_utc=datetime.now(timezone.utc).isoformat(),scenario_count=len(rows),passed=sum(r['status']=='PASS' for r in rows),
        failed=[r['scenario'] for r in rows if r['status']=='FAIL'],criteria='Declared before running: finite, no exceptions, state time advances at every 20Hz tick, error <=1m/s, distance deviation <=10m, recovery <=5s sustained for 1s, sane wheels accepted.',
        existing_stress=dict(scenarios=len({r['scenario'] for r in old}),methods=2,all_finite=all(r['finite'] for r in old),note='Existing script asserts finiteness only; it does not define accuracy/recovery pass limits.'),
        trust_reduction=trust,scenarios=rows,
        limitations=['Synthetic independent sinusoidal speed, no labeled real slip.','Distance is longitudinal and compared with clean replay, not geometric map position.',
          'Direct core simulation: invalid float values bypass ROS wire validation; bad command=16 fits int8 but violates application range.',
          'No claim of full ROS coverage for every scenario. Core forward timestamps are intentionally adversarial.',
          'Downweight count inferred from observed covariance update; not an internal diagnostic counter.'])
    (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    with (output/'scenario_metrics.csv').open('w',newline='') as f:
        flat=[{k:v for k,v in r.items() if k not in ('criteria','exceptions','accepted_after_fault')} for r in rows]
        w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
    (output/'acceptance_events.json').write_text(json.dumps(event_logs,indent=2,allow_nan=False)+'\n')
    np.savez_compressed(output/'traces.npz',**traces)
    text=[f"Synthetic cases: {len(rows)}; passed {summary['passed']}; failed {summary['failed']}",summary['criteria']]
    for r in rows:text.append(f"{r['scenario']}: {r['status']}; max speed error={r['max_velocity_error_mps']:.4f} m/s; distance deviation={r['max_relative_distance_deviation_m']:.3f} m; recovery={r['recovery_time_s']} s; rejected={r['rejected']}; downweighted={r['downweighted_observed']}")
    text+=['Trust reduction: '+json.dumps(trust),'Real labeled slip validation: NOT VERIFIED. Slip ratio / adhesion coefficient: not implemented.']
    (output/'summary.txt').write_text('\n'.join(text)+'\n');print('\n'.join(text))
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for name in ['front_slip_5mps','both_missing','both_slip_long','forward_header_spike']:
        trace=traces[name];axes[0].plot(trace[:,0],trace[:,2]-trace[:,1],label=name);axes[1].plot(trace[:,0],trace[:,3]-baseline[:,3],label=name);axes[2].plot(trace[:,0],trace[:,7],label=name)
    for ax in axes:ax.grid();ax.legend(fontsize=8)
    axes[0].set_ylabel('Speed error (m/s)');axes[1].set_ylabel('Distance vs clean (m)');axes[2].set_ylabel('Slip suspicion');axes[2].set_xlabel('Synthetic time (s)')
    fig.suptitle('Synthetic robustness: failures included, not real-slip validation');fig.tight_layout();fig.savefig(output/'scenario_plots.png');plt.close(fig)

if __name__=='__main__':main()
