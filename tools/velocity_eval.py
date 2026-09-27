"""Velocity-accuracy evidence: overall and per-motion-regime error against GNSS.

Reuses the causal replay and the sample matching of evaluate.py, so the overall
numbers equal results/test/metrics.json. GNSS is used only after replay.

Regimes are labelled from the GNSS reference itself (not from the estimate):
  stop          reference speed < STOP_V
  acceleration  reference acceleration > +ACC_A
  braking       reference acceleration < -ACC_A
  steady        otherwise (moving, |a| <= ACC_A)
Reference acceleration is a centred difference of GNSS speed over +-HALF_WINDOW s;
samples whose window is not covered by GNSS (gap) are labelled 'unclassified'.
"""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/tram_odometry'))
from tram_odometry.core import Config
from bag_io import load_cache
from calibrate import nearest, sorted_stream
from evaluate import replay

STOP_V=.2
ACC_A=.15
HALF_WINDOW=.5
REGIMES=['acceleration','braking','steady','stop']


def matched(data,out):
    """Same matching as evaluate.evaluate_reference 'raw' metric."""
    g=sorted_stream(data['master_vel']);t=g[:,0];ref=np.hypot(g[:,2],g[:,3])
    j=np.searchsorted(out[:,0],t).clip(0,len(out)-1);k=(j-1).clip(0)
    j=np.where(abs(out[k,0]-t)<abs(out[j,0]-t),k,j)
    valid=(abs(out[j,0]-t)<=.05)&np.isfinite(ref)&(ref<40)
    # Secondary check only (same as evaluate 'gnss_agree'): rover speed within 0.5 m/s.
    rv,rok=nearest(data['rover_vel'],t,.15)
    agree=valid&rok&(abs(np.hypot(rv[:,0],rv[:,1])-ref)<.5) if rv.shape[1]>=2 else np.zeros(len(t),bool)
    return t,ref,out[j,1],valid,agree


def reference_acceleration(t,ref):
    lo=np.searchsorted(t,t-HALF_WINDOW);hi=np.searchsorted(t,t+HALF_WINDOW,side='right')-1
    span=t[hi]-t[lo]
    a=np.where(span>0,(ref[hi]-ref[lo])/np.where(span>0,span,1.),np.nan)
    # Require the window to be nearly fully covered by GNSS without gaps.
    ok=span>=1.6*HALF_WINDOW
    gaps=np.r_[0.,np.diff(t)>.25].cumsum()
    ok&=gaps[hi]==gaps[lo]
    return np.where(ok,a,np.nan)


def regime(ref,acc):
    label=np.full(len(ref),'unclassified',dtype=object)
    moving=ref>=STOP_V;known=np.isfinite(acc)
    label[ref<STOP_V]='stop'
    label[moving&known&(acc>ACC_A)]='acceleration'
    label[moving&known&(acc<-ACC_A)]='braking'
    label[moving&known&(abs(np.nan_to_num(acc))<=ACC_A)]='steady'
    return label


def stats(e):
    if not len(e):return dict(samples=0,rmse=None,mae=None,bias=None,max_abs=None,p95_abs=None)
    return dict(samples=int(len(e)),rmse=float(np.sqrt(np.mean(e*e))),mae=float(np.mean(abs(e))),
                bias=float(np.mean(e)),max_abs=float(np.max(abs(e))),p95_abs=float(np.percentile(abs(e),95)))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('cache',type=Path);ap.add_argument('--splits',type=Path,required=True)
    ap.add_argument('--config',type=Path,required=True);ap.add_argument('--split',default='test')
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--plot-bag',default=None)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    config=Config.from_dict(json.loads(args.config.read_text()))
    entries=[e for e in json.loads(args.splits.read_text()) if e['split']==args.split]
    pooled={m:{'e':[],'r':[],'est':[],'label':[],'agree':[]} for m in ['model_observer','wheel_mean']}
    per_bag=[];traces={}
    for e in entries:
        z=load_cache(args.cache/(e['bag']+'.npz'))
        if len(z['master_vel'])<2:
            per_bag.append(dict(bag=e['bag'],status='no GNSS reference'));continue
        row=dict(bag=e['bag'],status='evaluated')
        for method,base in [('model_observer',False),('wheel_mean',True)]:
            out,_,_=replay(z,config,base)
            t,ref,est,valid,agree=matched(z,out)
            label=regime(ref,reference_acceleration(t,ref))
            err=est-ref
            p=pooled[method];p['e'].append(err[valid]);p['r'].append(ref[valid]);p['est'].append(est[valid]);p['label'].append(label[valid]);p['agree'].append(agree[valid])
            s=stats(err[valid]);row.update({f'{method}_{k}':v for k,v in s.items()})
            if method=='model_observer':traces[e['bag']]=(t[valid],ref[valid],est[valid],label[valid],out)
        per_bag.append(row);print(e['bag'],row['model_observer_rmse'],flush=True)
    summary=dict(split=args.split,bags_in_split=len(entries),
                 bags_evaluated=sum(r['status']=='evaluated' for r in per_bag),
                 regime_definition=dict(stop_speed_mps=STOP_V,acceleration_threshold_mps2=ACC_A,
                                        acceleration_window_s=2*HALF_WINDOW,source='GNSS master velocity (reference only)'),
                 methods={})
    regime_rows=[]
    for method,p in pooled.items():
        e=np.concatenate(p['e']);r=np.concatenate(p['r']);est=np.concatenate(p['est']);lab=np.concatenate(p['label'])
        ag=np.concatenate(p['agree'])
        m=dict(overall=stats(e),overall_gnss_agree=stats(e[ag]))
        for name in REGIMES+['unclassified']:
            m[name]=stats(e[lab==name]);m[name+'_gnss_agree']=stats(e[(lab==name)&ag])
            regime_rows.append(dict(method=method,regime=name,reference='raw',**m[name]))
            regime_rows.append(dict(method=method,regime=name,reference='gnss_agree',**m[name+'_gnss_agree']))
        stop=lab=='stop'
        m['stop_extra']=dict(fraction_estimate_above_0p1_mps=float(np.mean(est[stop]>.1)) if stop.any() else None,
                             mean_estimate_mps=float(np.mean(est[stop])) if stop.any() else None,
                             fraction_estimate_exactly_zero=float(np.mean(est[stop]==0)) if stop.any() else None)
        summary['methods'][method]=m
    (args.output/'summary.json').write_text(json.dumps(dict(summary=summary,per_bag=per_bag),indent=2,allow_nan=False))
    with open(args.output/'regime_metrics.csv','w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(regime_rows[0]));w.writeheader();w.writerows(regime_rows)
    with open(args.output/'per_bag_metrics.csv','w',newline='') as f:
        keys=sorted({k for r in per_bag for k in r},key=lambda k:(k!='bag',k!='status',k))
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(per_bag)
    lines=[f"Velocity accuracy, split={args.split}: {summary['bags_evaluated']}/{len(entries)} bags with GNSS reference",
           'Error = estimate - GNSS master speed [m/s]; positive bias = overestimate.','']
    lines.append(f"{'method':15s} {'regime':13s} {'reference':10s} {'samples':>8s} {'RMSE':>7s} {'MAE':>7s} {'bias':>8s} {'p95|e|':>7s} {'max|e|':>7s}")
    for rr in regime_rows:
        if rr['samples']:lines.append(f"{rr['method']:15s} {rr['regime']:13s} {rr['reference']:10s} {rr['samples']:8d} {rr['rmse']:7.4f} {rr['mae']:7.4f} {rr['bias']:+8.4f} {rr['p95_abs']:7.3f} {rr['max_abs']:7.3f}")
    for method in pooled:
        o=summary['methods'][method]
        for q,key in [('raw','overall'),('gnss_agree','overall_gnss_agree')]:
            lines.append(f"{method:15s} overall       {q:10s} {o[key]['samples']:8d} {o[key]['rmse']:7.4f} {o[key]['mae']:7.4f} {o[key]['bias']:+8.4f} {o[key]['p95_abs']:7.3f} {o[key]['max_abs']:7.3f}")
    se=summary['methods']['model_observer']['stop_extra']
    lines+=['',"reference=raw: every matched GNSS master sample; gnss_agree: rover speed within 0.5 m/s of master (metric filter only).",f"Stops (model_observer): mean estimate {se['mean_estimate_mps']:.4f} m/s, exactly zero {100*se['fraction_estimate_exactly_zero']:.1f}%, >0.1 m/s {100*se['fraction_estimate_above_0p1_mps']:.2f}%"]
    (args.output/'summary.txt').write_text('\n'.join(lines)+'\n');print('\n'.join(lines))
    plot(args,traces,per_bag,regime_rows)


def plot(args,traces,per_bag,regime_rows):
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    ev=sorted([r for r in per_bag if r['status']=='evaluated'],key=lambda r:r['model_observer_rmse'])
    bag=args.plot_bag or ev[len(ev)//2]['bag']
    t,ref,est,lab,out=traces[bag];t0=t[0]
    colors={'acceleration':'#2a9d8f','braking':'#e76f51','steady':'#8d99ae','stop':'#264653','unclassified':'#dddddd'}
    fig,ax=plt.subplots(2,1,figsize=(13,7),sharex=True)
    ax[0].plot(t-t0,ref,lw=1.2,label='GNSS master speed (reference)',color='k')
    ax[0].plot(t-t0,est,lw=.9,label='Estimated velocity (model observer)',color='#d62828',alpha=.85)
    ax[0].set_ylabel('m/s');ax[0].legend(loc='upper right');ax[0].grid(alpha=.3)
    ax[0].set_title(f'Held-out test bag {bag} (median-RMSE bag of the test split)')
    e=est-ref
    for name,c in colors.items():
        m=lab==name
        ax[1].scatter(t[m]-t0,e[m],s=1,color=c,label=name)
    ax[1].axhline(0,color='k',lw=.6);ax[1].set_ylabel('estimate − GNSS, m/s');ax[1].set_xlabel('time from first matched sample, s')
    ax[1].legend(loc='upper right',markerscale=8,ncol=5);ax[1].grid(alpha=.3)
    fig.tight_layout();fig.savefig(args.output/'velocity_comparison.png',dpi=120);plt.close(fig)
    # Zoom on a stop -> acceleration -> braking episode.
    moving=np.flatnonzero(ref>=STOP_V)
    starts=moving[np.r_[True,np.diff(t[moving])>5]]
    s0=t[starts[min(1,len(starts)-1)]]-15
    m=(t>=s0)&(t<=s0+120)
    fig,ax=plt.subplots(2,1,figsize=(13,6),sharex=True)
    ax[0].plot(t[m]-t0,ref[m],color='k',lw=1.4,label='GNSS reference')
    ax[0].plot(t[m]-t0,est[m],color='#d62828',lw=1.,label='Estimate')
    for name,c in colors.items():
        mm=m&(lab==name);ax[0].scatter(t[mm]-t0,np.full(mm.sum(),-.3),s=4,color=c,marker='s',label=name)
    ax[0].set_ylabel('m/s');ax[0].legend(loc='upper right',ncol=3,fontsize=8);ax[0].grid(alpha=.3)
    ax[0].set_title(f'{bag}: zoom on stop / acceleration / braking (regime bar at bottom)')
    ax[1].plot(t[m]-t0,e[m],color='#d62828',lw=.8);ax[1].axhline(0,color='k',lw=.6)
    ax[1].set_ylabel('error, m/s');ax[1].set_xlabel('time, s');ax[1].grid(alpha=.3)
    fig.tight_layout();fig.savefig(args.output/'velocity_zoom_transients.png',dpi=120);plt.close(fig)
    fig,ax=plt.subplots(figsize=(13,4))
    ax.plot(t-t0,e,lw=.5,color='#d62828');ax.axhline(0,color='k',lw=.6)
    ax.set_title(f'{bag}: velocity error over the whole run (RMSE {np.sqrt(np.mean(e*e)):.3f} m/s, bias {np.mean(e):+.3f} m/s)')
    ax.set_ylabel('estimate − GNSS, m/s');ax.set_xlabel('time, s');ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(args.output/'velocity_error.png',dpi=120);plt.close(fig)
    fig,ax=plt.subplots(1,3,figsize=(13,4))
    names=REGIMES+['overall'];x=np.arange(len(REGIMES))
    for i,key in enumerate(['rmse','mae','bias']):
        for k,(method,c) in enumerate([('model_observer','#d62828'),('wheel_mean','#8d99ae')]):
            vals=[next(r[key] for r in regime_rows if r['method']==method and r['regime']==g and r['reference']=='raw') for g in REGIMES]
            ax[i].bar(x+(k-.5)*.38,vals,.38,color=c,label=method)
        ax[i].set_xticks(x);ax[i].set_xticklabels(REGIMES);ax[i].set_title(key.upper()+', m/s');ax[i].grid(alpha=.3,axis='y')
        ax[i].axhline(0,color='k',lw=.6)
    ax[0].legend();fig.suptitle(f'Velocity error by motion regime, pooled over {args.split} split (raw GNSS master reference)')
    fig.tight_layout();fig.savefig(args.output/'regime_metrics.png',dpi=120);plt.close(fig)


if __name__=='__main__':main()
