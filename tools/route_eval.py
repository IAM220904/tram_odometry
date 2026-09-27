"""Position evaluation: NO MAP (heading held) vs TRAIN-built path map.

GNSS enters the replay only through PositionTracker's 3 s initialization window;
afterwards master fixes are used only as reference. Errors are horizontal (2D);
along/cross-track are the error projected on the reference travel direction.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/tram_odometry'))
from tram_odometry.core import Config
from tram_odometry.position import PathMap, PositionTracker
from bag_io import load_cache
from calibrate import nearest, sorted_stream
from evaluate import replay


def map_residual(path_map,xy,direction):
    """Cross-track distance of reference fixes to the nearest same-direction node."""
    out=np.full(len(xy),np.nan)
    for i,((x,y),(hx,hy)) in enumerate(zip(xy,direction)):
        if not np.isfinite(hx):continue
        best=math.inf
        for n in _near(path_map,x,y,8.):
            if n[3]*hx+n[4]*hy<.7:continue
            best=min(best,math.hypot(x-n[0],y-n[1]))
        out[i]=best
    return out


def _near(m,x,y,r):
    cx=math.floor(x/m.cell);cy=math.floor(y/m.cell);k=int(math.ceil(r/m.cell))
    for i in range(cx-k,cx+k+1):
        for j in range(cy-k,cy+k+1):
            for idx in m.grid.get((i,j),()):yield m.nodes[idx]


def reference(tracker,z):
    f=sorted_stream(z['master_fix']);f=f[(f[:,5]>=0)&np.isfinite(f[:,2:5]).all(1)]
    truth=np.array([tracker.frame.enu(*v) for v in f[:,2:5]]);t=f[:,0]
    lo=np.searchsorted(t,t-1.);hi=np.searchsorted(t,t+1.,side='right')-1
    d=truth[hi,:2]-truth[lo,:2];n=np.linalg.norm(d,axis=1)
    direction=np.where((n>=1.5)[:,None],d/np.maximum(n,1e-9)[:,None],np.nan)
    # Carry the last moving direction through stops.
    for i in range(1,len(direction)):
        if not np.isfinite(direction[i,0]):direction[i]=direction[i-1]
    return t,truth,direction


def bag_metrics(out,t,truth,direction,start):
    pp,ok=nearest(np.column_stack([out[:,0],out[:,0],out[:,7:10]]),t,.05)
    ok&=(t>start+3)
    if ok.sum()<10:return None
    e=pp[ok,:2]-truth[ok,:2];d=direction[ok];err=np.linalg.norm(e,axis=1)
    has=np.isfinite(d[:,0]);along=(e[has]*d[has]).sum(1);cross=e[has,0]*-d[has,1]+e[has,1]*d[has,0]
    travelled=float(np.linalg.norm(np.diff(truth[ok,:2],axis=0),axis=1).sum())
    stat=lambda x:dict(mean=float(np.mean(x)),rmse=float(np.sqrt(np.mean(x*x))),max_abs=float(np.max(abs(x)))) if len(x) else dict(mean=None,rmse=None,max_abs=None)
    return dict(samples=int(ok.sum()),initial_error_m=float(err[0]),final_error_m=float(err[-1]),
                rmse_m=float(np.sqrt(np.mean(err**2))),mae_m=float(np.mean(err)),max_error_m=float(err.max()),
                travelled_m=travelled,final_drift_pct=100*float(err[-1])/travelled if travelled>50 else None,
                along=stat(along),cross=stat(cross),final_3d_error_m=float(np.linalg.norm(pp[ok][-1]-truth[ok][-1])))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('cache',type=Path);ap.add_argument('--splits',type=Path,required=True)
    ap.add_argument('--config',type=Path,required=True);ap.add_argument('--path-map',type=Path,required=True)
    ap.add_argument('--split',choices=['dev','test'],required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    config=Config.from_dict(json.loads(args.config.read_text()));pm=PathMap.load(args.path_map)
    meta=json.loads(args.path_map.read_text())['metadata']
    entries=[e for e in json.loads(args.splits.read_text()) if e['split']==args.split]
    assert not set(meta['train_bags'])&{e['bag'] for e in entries},'map built from evaluated bags'
    rows=[];traces={}
    for e in entries:
        z=load_cache(args.cache/(e['bag']+'.npz'))
        row=dict(bag=e['bag'],split=e['split'])
        if len(z['master_fix'])<20 or len(z['master_vel'])<2:
            rows.append(dict(row,status='no GNSS reference'));continue
        runs={}
        for method,pmap in [('no_map',None),('path_map',pm)]:
            tr=PositionTracker(origin=tuple(pm.origin),path_map=pmap)
            out,_,_=replay(z,config,position=tr)
            if tr.start is None or not tr.fix_seen:runs[method]=None;continue
            t,truth,direction=reference(tr,z)
            runs[method]=bag_metrics(out,t,truth,direction,tr.start)
            if pmap is not None and runs[method]:
                runs[method]['off_map_m']=tr.follower.off_map_m if tr.follower else None
                traces[e['bag']]=(out,t,truth,tr.start)
                if args.split=='dev':
                    res=map_residual(pm,truth[:,:2],direction)
                    moving=np.isfinite(direction[:,0])&(t>tr.start+3)
                    runs[method]['map_residual_rmse_m']=float(np.sqrt(np.nanmean(res[moving]**2)))
                    runs[method]['map_coverage_pct']=float(100*np.mean(res[moving]<=3.))
        if not runs['path_map']:
            rows.append(dict(row,status='no initial GNSS fix in window'));continue
        row['status']='evaluated'
        for method,m in runs.items():
            for k,v in m.items():
                if isinstance(v,dict):
                    for kk,vv in v.items():row[f'{method}_{k}_{kk}']=vv
                else:row[f'{method}_{k}']=v
        rows.append(row);print(e['bag'],'no_map final',round(row['no_map_final_error_m'],1),'path_map final',round(row['path_map_final_error_m'],1),'rmse',round(row['path_map_rmse_m'],1),flush=True)
    ev=[r for r in rows if r['status']=='evaluated']
    agg={}
    for method in ['no_map','path_map']:
        med=lambda k:float(np.median([r[f'{method}_{k}'] for r in ev if r.get(f'{method}_{k}') is not None]))
        agg[method]={k:med(k) for k in ['initial_error_m','final_error_m','rmse_m','mae_m','max_error_m','final_drift_pct',
                                         'along_mean','along_rmse','along_max_abs','cross_mean','cross_rmse','cross_max_abs']}
        ev_m=[r for r in ev if r.get(f'{method}_cross_rmse') is not None];n=sum(r[f'{method}_samples'] for r in ev_m)
        agg[method]['pooled_rmse_m']=float(math.sqrt(sum(r[f'{method}_samples']*r[f'{method}_rmse_m']**2 for r in ev_m)/n))
        agg[method]['pooled_cross_rmse_m']=float(math.sqrt(sum(r[f'{method}_samples']*r[f'{method}_cross_rmse']**2 for r in ev_m)/n))
        agg[method]['pooled_along_rmse_m']=float(math.sqrt(sum(r[f'{method}_samples']*r[f'{method}_along_rmse']**2 for r in ev_m)/n))
        agg[method]['max_final_error_m']=float(max(r[f'{method}_final_error_m'] for r in ev))
    if args.split=='dev':
        agg['path_map']['map_coverage_pct_median']=float(np.nanmedian([r['path_map_map_coverage_pct'] for r in ev]))
        agg['path_map']['map_coverage_pct_min']=float(np.nanmin([r['path_map_map_coverage_pct'] for r in ev]))
        agg['path_map']['map_residual_rmse_m_median']=float(np.nanmedian([r['path_map_map_residual_rmse_m'] for r in ev]))
    summary=dict(split=args.split,bags_in_split=len(entries),bags_evaluated=len(ev),aggregate_median_over_bags=agg,
                 failures=dict(path_map_final_error_over_50m=[r['bag'] for r in ev if r['path_map_final_error_m']>50],
                               path_map_max_error_over_100m=[r['bag'] for r in ev if r['path_map_max_error_m']>100]),
                 not_evaluated=[dict(bag=r['bag'],status=r['status']) for r in rows if r['status']!='evaluated'])
    (args.output/f'{args.split}_summary.json').write_text(json.dumps(summary,indent=2))
    keys=sorted({k for r in rows for k in r},key=lambda k:(k not in('bag','split','status'),k))
    with open(args.output/f'{args.split}_metrics.csv','w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
    print(json.dumps(summary,indent=1))
    plots(args,pm,traces,ev)


def plots(args,pm,traces,ev):
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    nodes=np.array(pm.nodes)
    if args.split=='dev':
        fig,ax=plt.subplots(figsize=(14,6))
        ax.scatter(nodes[:,0],nodes[:,1],s=.5,c='#8d99ae',label='train path map nodes')
        for b,(out,t,truth,start) in traces.items():ax.plot(truth[:,0],truth[:,1],lw=.6)
        ax.set_aspect('equal');ax.set_title('Train-only path map with DEV GNSS tracks overlaid');ax.grid(alpha=.3);ax.legend()
        fig.tight_layout();fig.savefig(args.output/'dev_route_overlay.png',dpi=110);plt.close(fig)
        fig,ax=plt.subplots(figsize=(14,6))
        ax.quiver(nodes[::4,0],nodes[::4,1],nodes[::4,3],nodes[::4,4],angles='xy',scale=60,width=.0012,color='#264653')
        ax.set_aspect('equal');ax.set_title(f'Train-only directed path map ({len(nodes)} nodes, {pm.cell:g} m cells), ENU m');ax.grid(alpha=.3)
        fig.tight_layout();fig.savefig(args.output/'route_map.png',dpi=110);plt.close(fig)
        return
    order=sorted(ev,key=lambda r:r['path_map_rmse_m']);bag=order[len(order)//2]['bag']
    out,t,truth,start=traces[bag]
    fig,ax=plt.subplots(figsize=(14,6))
    ax.scatter(nodes[:,0],nodes[:,1],s=.3,c='#dddddd',label='train path map')
    ax.plot(truth[:,0],truth[:,1],'k',lw=1.5,label='test GNSS reference')
    ax.plot(out[:,7],out[:,8],'#d62828',lw=.9,label='estimate with path map')
    ax.set_aspect('equal');ax.grid(alpha=.3);ax.legend();ax.set_title(f'Held-out test bag {bag} (median path-map RMSE)')
    fig.tight_layout();fig.savefig(args.output/'test_trajectory_comparison.png',dpi=110);plt.close(fig)
    for kind in ['along','cross']:
        fig,ax=plt.subplots(figsize=(13,4))
        for b,(out,t,truth,start) in traces.items():
            tr=PositionTracker(origin=tuple(pm.origin))  # frame only
            _,_,direction=reference(tr,load_cache(args.cache/(b+'.npz')))
            pp,ok=nearest(np.column_stack([out[:,0],out[:,0],out[:,7:10]]),t,.05);ok&=t>start+3;ok&=np.isfinite(direction[:,0])
            if not ok.any():continue
            e=pp[ok,:2]-truth[ok,:2];d=direction[ok]
            v=(e*d).sum(1) if kind=='along' else e[:,0]*-d[:,1]+e[:,1]*d[:,0]
            ax.plot(t[ok]-t[ok][0],v,lw=.6)
        ax.set_title(f'{kind}-track error, all held-out test bags, path map');ax.set_xlabel('s after GNSS window');ax.set_ylabel('m');ax.grid(alpha=.3)
        fig.tight_layout();fig.savefig(args.output/f'{kind}_track_error.png',dpi=110);plt.close(fig)


if __name__=='__main__':main()
