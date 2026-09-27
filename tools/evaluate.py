"""Arrival-ordered causal replay of the exact runtime estimator; GNSS used after replay."""
import argparse
import csv
import heapq
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/tram_odometry'))
from tram_odometry.core import Config, Estimator, SourceClock
from tram_odometry.position import PositionTracker
from bag_io import load_cache
from calibrate import nearest, sorted_stream


class Baseline:
    def __init__(self,scale):
        self.scale=scale;self.t=None;self.v=0.;self.s=0.;self.wheels={};self.last={};self.initialized=False
    def feed(self,k,t,v):
        if not math.isfinite(v) or t<=self.last.get(k,-math.inf):return
        self.last[k]=t
        if k!='command':
            self.wheels[k]=(t,max(0.,min(40.,v*self.scale)));self.initialized=True
        self.predict(max(t,self.t if self.t is not None else t))
    def predict(self,t):
        if self.t is not None:self.s+=self.v*max(0.,t-self.t)
        self.t=t
        fresh=[v for stamp,v in self.wheels.values() if 0<=t-stamp<.5]
        if fresh:self.v=sum(fresh)/len(fresh)
    def snapshot(self):
        return dict(t=self.t,velocity=self.v,distance=self.s,velocity_variance=0.,distance_variance=0.,status='ok',slip=[])


def replay(data,config,baseline=False,fault=None,position=None):
    model=Baseline(1/3.6) if baseline else Estimator(config)
    source=SourceClock();streams=[]
    end=max(data[k][-1,1] for k in ['front','rear','command'] if len(data[k]))
    keys=['front','rear','command']
    if position is not None:keys+=['master_fix','rover_fix','master_vel','rover_vel']
    def stream(a,i):
        for j in np.argsort(a[:,1],kind='stable'):
            yield float(a[j,1]),i,float(a[j,0]),tuple(map(float,a[j,2:]))
    for i,key in enumerate(keys):
        if len(data[key]):streams.append(stream(data[key],i))
    events=iter(heapq.merge(*streams));event=next(events,None)
    rows=[];latencies=[];tick=0.
    while tick<=end:
        started=time.perf_counter()
        while event is not None and event[0]<=tick:
            receipt,i,stamp,values=event;kind=keys[i]
            if i>=3:
                if model.t is not None:position.feed_gnss(kind,stamp,values,model.snapshot())
                event=next(events,None);continue
            value=values[0]
            if fault:value=fault(kind,stamp,value)
            if value is not None:
                before=time.perf_counter()
                model.feed(kind,stamp,value)
                latencies.append(time.perf_counter()-before)
                if math.isfinite(value) and math.isfinite(stamp) and ((kind=='command' and abs(value)<=15) or (kind!='command' and abs(value*config.wheel_scale)<=40)):
                    source.observe(stamp,receipt)
                if position is not None and model.t is not None:position.begin(model.t)
            event=next(events,None)
        now=source.now(tick)
        if now is not None:
            model.predict(max(model.t if model.t is not None else now,now))
            if model.initialized:
                snap=model.snapshot()
                row=[snap['t'],snap['velocity'],snap['distance'],snap['velocity_variance'],snap['distance_variance'],int(snap['status']!='ok'),int(bool(snap['slip']))]
                if position is not None:
                    pose=position.pose(snap);row.extend([pose['x'],pose['y'],pose['z'],pose['yaw']])
                rows.append(row)
        latencies.append(time.perf_counter()-started)
        tick+=.05
    return np.asarray(rows),np.asarray(latencies),model.snapshot()


def evaluate_reference(data,out):
    if not len(out) or len(data['master_vel'])<2:return None
    g=sorted_stream(data['master_vel']);t=g[:,0];ref=np.hypot(g[:,2],g[:,3])
    j=np.searchsorted(out[:,0],t).clip(0,len(out)-1);k=(j-1).clip(0)
    j=np.where(abs(out[k,0]-t)<abs(out[j,0]-t),k,j)
    valid=(abs(out[j,0]-t)<=.05)&np.isfinite(ref)&(ref<40)
    rv,rok=nearest(data['rover_vel'],t,.15)
    good=valid.copy()
    if rv.shape[1]>=2: good &= rok&(abs(np.hypot(rv[:,0],rv[:,1])-ref)<.5)
    else: good &= False
    e=out[j,1]-ref
    # Integrate only reference intervals with valid timestamps, no long GNSS gaps.
    dref=.5*(ref[1:]+ref[:-1])*np.diff(t)
    dest=np.diff(out[j,2])
    interval=valid[1:]&valid[:-1]&(np.diff(t)>0)&(np.diff(t)<.25)
    clean_interval=good[1:]&good[:-1]&(np.diff(t)>0)&(np.diff(t)<.25)
    speed=lambda m:dict(samples=int(m.sum()),rmse=float(np.sqrt(np.mean(e[m]**2))) if m.any() else None,
                        mae=float(np.mean(abs(e[m]))) if m.any() else None,bias=float(np.mean(e[m])) if m.any() else None)
    distance=float(dref[interval].sum());drift=float((dest-dref)[interval].sum())
    clean_distance=float(dref[clean_interval].sum());clean_drift=float((dest-dref)[clean_interval].sum())
    result=dict(raw=speed(valid),gnss_agree=speed(good),reference_samples=len(t),
                match_coverage=float(valid.mean()),reference_distance_m=distance,
                integrated_distance_error_m=drift,relative_distance_drift_pct=100*drift/distance if distance>10 else None,
                clean_reference_distance_m=clean_distance,clean_integrated_distance_error_m=clean_drift,
                reference_duration_s=float(np.diff(t)[interval].sum()))
    return result


def main():
    ap=argparse.ArgumentParser();ap.add_argument('cache',type=Path);ap.add_argument('--splits',type=Path,required=True)
    ap.add_argument('--config',type=Path,required=True);ap.add_argument('--split',choices=['train','dev','test','all'],default='dev')
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--save-traces',action='store_true')
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    config=Config.from_dict(json.loads(args.config.read_text()));entries=json.loads(args.splits.read_text())
    rows=[];timing=[];beg=time.perf_counter()
    for e in entries:
        if args.split!='all' and e['split']!=args.split:continue
        z=load_cache(args.cache/(e['bag']+'.npz'))
        for base in [False,True]:
            tracker=PositionTracker() if not base else None
            out,latency,status=replay(z,config,base,position=tracker)
            if not base:timing.extend(latency[::10])
            metrics=evaluate_reference(z,out)
            if metrics and tracker is not None and tracker.frame is not None:
                fixes=sorted_stream(z['master_fix'])
                if len(fixes):
                    predicted=np.column_stack([out[:,0],out[:,0],out[:,7:10]])
                    pp,ok=nearest(predicted,fixes[:,0],.05)
                    truth=np.array([tracker.frame.enu(*v) for v in fixes[:,2:5]])
                    ok &= (fixes[:,5]>=0)&(fixes[:,0]>tracker.start+3)&np.isfinite(truth).all(axis=1)
                    ee=np.linalg.norm(pp-truth,axis=1)
                    if ok.any():
                        metrics['position_without_map']=dict(samples=int(ok.sum()),rmse_3d_m=float(np.sqrt(np.mean(ee[ok]**2))),
                                                            final_error_3d_m=float(ee[ok][-1]),gnss_init_messages=tracker.gnss_used,
                                                            mode='Heading held after first 3 s; cannot observe subsequent turns')
            row=dict(bag=e['bag'],split=e['split'],method='wheel_mean' if base else 'model_observer',outputs=len(out),
                     final_status=status,metrics=metrics)
            rows.append(row)
            if args.save_traces and not base:
                np.savez_compressed(args.output/(e['bag']+'.npz'),output=out)
        print(e['bag'],rows[-2]['metrics']['raw']['rmse'] if rows[-2]['metrics'] else 'no reference',flush=True)
    summary={}
    for method in ['model_observer','wheel_mean']:
        rr=[r for r in rows if r['method']==method and r['metrics']]
        summary[method]={}
        for quality in ['raw','gnss_agree']:
            vv=[r['metrics'][quality] for r in rr if r['metrics'][quality]['samples']]
            n=sum(v['samples'] for v in vv)
            summary[method][quality]=dict(samples=n,rmse=(sum(v['samples']*v['rmse']**2 for v in vv)/n)**.5 if n else None,
                mae=sum(v['samples']*v['mae'] for v in vv)/n if n else None,
                bias=sum(v['samples']*v['bias'] for v in vv)/n if n else None)
        drifts=[abs(r['metrics']['relative_distance_drift_pct']) for r in rr if r['metrics']['relative_distance_drift_pct'] is not None]
        summary[method].update(bags=len(rr),median_abs_distance_drift_pct=float(np.median(drifts)) if drifts else None,
                              max_abs_distance_drift_pct=float(max(drifts)) if drifts else None)
    summary['timing']=dict(measured='Core feed call / tick batch on this host; not ROS transport latency',
                           p50_us=float(np.percentile(timing,50)*1e6),p95_us=float(np.percentile(timing,95)*1e6),
                           p99_us=float(np.percentile(timing,99)*1e6),max_us=float(np.max(timing)*1e6),
                           wall_s=time.perf_counter()-beg)
    result=dict(split=args.split,summary=summary,bags=rows)
    (args.output/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    flat=[]
    for r in rows:
        if r['metrics']:
            m=r['metrics'];flat.append(dict(bag=r['bag'],split=r['split'],method=r['method'],rmse_mps=m['raw']['rmse'],mae_mps=m['raw']['mae'],bias_mps=m['raw']['bias'],gnss_agree_rmse=m['gnss_agree']['rmse'],match_coverage=m['match_coverage'],distance_error_m=m['integrated_distance_error_m'],distance_drift_pct=m['relative_distance_drift_pct']))
    if flat:
        with open(args.output/'metrics.csv','w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
