"""Create nonoverlapping bag splits and fit an identifiable acceleration model."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.optimize import lsq_linear
from scipy.ndimage import gaussian_filter1d
from bag_io import load_cache


def sorted_stream(a):
    if not len(a): return a
    a=a[np.argsort(a[:,0],kind='stable')]
    return a[np.r_[True,np.diff(a[:,0])>0]]


def nearest(a,t,tolerance=.15):
    a=sorted_stream(a)
    if not len(a): return np.full((len(t),1),np.nan),np.zeros(len(t),bool)
    j=np.searchsorted(a[:,0],t).clip(0,len(a)-1)
    k=(j-1).clip(0)
    j=np.where(abs(a[k,0]-t)<abs(a[j,0]-t),k,j)
    return a[j,2:],abs(a[j,0]-t)<=tolerance


def features(u,v):
    p=np.maximum(u,0)/15.; b=np.maximum(-u,0)/15.
    return np.stack([p/(1+v/12),p*p/(1+v/12),-b,-b*b,
                     -np.tanh(v/.3),-v/15,-v*v/225],axis=-1)


def make_splits(data,cache):
    groups={}
    for db in sorted(data.rglob('*.db3')):
        h=hashlib.sha256(db.read_bytes()).hexdigest()
        groups.setdefault(h,[]).append(db.parent.name)
    entries=[]
    for h,names in groups.items():
        bag=names[0]; z=load_cache(cache/(bag+'.npz'))
        offset=int(z['start_ns'])*1e-9
        arrays=[z[k] for k in ['front','rear','command'] if len(z[k])]
        start=offset+min(a[:,0].min() for a in arrays)
        end=offset+max(a[:,0].max() for a in arrays)
        entries.append(dict(bag=bag,aliases=names[1:],sha256=h,vehicle=bag.split('_')[0],
                            start=start,end=end,has_reference=len(z['master_vel'])>=10))
    # Merge overlapping intervals per vehicle before chronological splitting.
    for veh in sorted({e['vehicle'] for e in entries}):
        temporal=[]
        for entry in sorted([e for e in entries if e['vehicle']==veh],key=lambda e:e['start']):
            if temporal and entry['start']<=max(e['end'] for e in temporal[-1]):
                temporal[-1].append(entry)
            else: temporal.append([entry])
        count=len(temporal)
        for i,group in enumerate(temporal):
            split='train' if i<int(.6*count) else 'dev' if i<int(.8*count) else 'test'
            for entry in group: entry.update(split=split,temporal_group=f'{veh}_{i:03d}')
    return sorted(entries,key=lambda e:(e['vehicle'],e['start']))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('data',type=Path);ap.add_argument('cache',type=Path)
    ap.add_argument('--output',type=Path,default=Path('results'));ap.add_argument('--config',type=Path,required=True)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    entries=make_splits(args.data,args.cache)
    (args.output/'splits.json').write_text(json.dumps(entries,indent=2))
    ratios=[];train=[]
    for e in entries:
        if e['split']!='train' or not e['has_reference']: continue
        z=load_cache(args.cache/(e['bag']+'.npz'));g=sorted_stream(z['master_vel'])
        t=g[:,0];v=np.hypot(g[:,2],g[:,3])
        f,okf=nearest(z['front'],t);r,okr=nearest(z['rear'],t);rv,okg=nearest(z['rover_vel'],t)
        if rv.shape[1]<2: continue
        q=okf&okr&okg&np.isfinite(v)&(v>3)&(v<25)&(abs(np.hypot(rv[:,0],rv[:,1])-v)<.3)
        wheels=(f[:,0]+r[:,0])/2
        q &= (wheels>1)&(abs(f[:,0]-r[:,0])<1.8)
        if q.sum()>50: ratios.append((e['bag'],float(np.median(v[q]/wheels[q]))))
        train.append((e,z))
    scale=float(np.median([r[1] for r in ratios]))
    matrices=[];targets=[];weights=[]
    for e,z in train:
        g=sorted_stream(z['master_vel']);t=np.arange(max(5,g[0,0]),g[-1,0]-.5,.1)
        if len(t)<20: continue
        gv,ok=nearest(g,t,.06);rv,okr=nearest(z['rover_vel'],t,.15)
        if rv.shape[1]<2:continue
        v=np.hypot(gv[:,0],gv[:,1]);smooth=gaussian_filter1d(v,2.0);a=np.gradient(smooth,.1)
        cmd,oku=nearest(z['command'],t-.25,.1)
        f,okf=nearest(z['front'],t);r,okre=nearest(z['rear'],t)
        mask=ok&okr&oku&okf&okre&(v>.4)&(v<25)&(abs(a)<2.5)
        mask &= abs(np.hypot(rv[:,0],rv[:,1])-v)<.3
        mask &= abs((f[:,0]+r[:,0])*scale/2-v)<.3
        mask &= abs(f[:,0]-r[:,0])*scale<.2
        # Erode validity to prevent derivative kernels spanning GNSS gaps.
        from scipy.ndimage import minimum_filter1d
        mask=minimum_filter1d(mask.astype(int),size=13).astype(bool)
        if mask.sum()<20:continue
        matrices.append(features(cmd[:,0],smooth)[mask]);targets.append(a[mask])
        weights.append(np.full(mask.sum(),1/np.sqrt(mask.sum())))
    X=np.concatenate(matrices);y=np.concatenate(targets);w0=np.concatenate(weights)
    w=w0.copy();coef=np.zeros(7)
    for _ in range(5):
        coef=lsq_linear(X*w[:,None],y*w,bounds=(0,[5,5,5,5,.8,2,2])).x
        residual=y-X@coef
        w=w0*np.sqrt(np.minimum(1,.15/np.maximum(abs(residual),1e-12)))
    config=dict(wheel_scale=scale,coefficients=coef.tolist(),drive_tau=.25,
                wheel_sigma=.025,acceleration_noise=.7,slip_threshold=.65,
                wheel_difference_threshold=.45,max_acceleration=3.0,
                stale_timeout=.5,command_timeout=.5,coast_after=2.0,
                out_of_order_tolerance=.25,stop_velocity=.025)
    args.config.parent.mkdir(parents=True,exist_ok=True);args.config.write_text(json.dumps(config,indent=2))
    report=dict(unique_bags=len(entries),duplicate_copies=sum(len(e['aliases']) for e in entries),
                splits={s:dict(bags=sum(e['split']==s for e in entries),reference_bags=sum(e['split']==s and e['has_reference'] for e in entries)) for s in ['train','dev','test']},
                wheel_scale=scale,nominal_kmh_scale=1/3.6,per_bag_scale=ratios,
                coefficient_order=['traction_linear','traction_quadratic','brake_linear','brake_quadratic','rolling','linear_drag','quadratic_drag'],
                coefficients=coef.tolist(),training_acceleration_samples=len(y),
                acceleration_training_rmse=float(np.sqrt(np.mean((X@coef-y)**2))),
                fit_note='Only train bags used. Offline smoothed GNSS targets; no GNSS in online velocity estimator.')
    (args.output/'calibration.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='per_bag_scale'},indent=2))

if __name__=='__main__':main()
