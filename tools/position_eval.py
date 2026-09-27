"""One held-out bag: distance accuracy versus diagnostic no-map ENU position.

No route is created from GNSS. Uses unchanged evaluate.replay and frozen model.
Only the position tracker's initial three seconds may consume GNSS at runtime.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tram_odometry'))
from bag_io import TOPICS, messages
from calibrate import sorted_stream, nearest
from evaluate import replay, evaluate_reference
from tram_odometry.core import Config
from tram_odometry.position import PositionTracker


def read_bag(bag):
    streams = {k: [] for k in TOPICS.values()}; start = None
    for receipt, stamp, kind, values in messages(bag):
        if start is None:
            start = receipt
        streams[kind].append(((stamp-start)*1e-9, (receipt-start)*1e-9, *values))
    return {k: np.asarray(v, dtype=float) for k, v in streams.items()}, start


def norm_stats(error):
    return dict(samples=len(error), rmse_m=float(np.sqrt(np.mean(error**2))),
                mae_m=float(np.mean(abs(error))), max_m=float(np.max(abs(error))), final_m=float(error[-1]))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', type=Path, default=ROOT / 'dataset/data/30618_d4ba7005')
    ap.add_argument('--output', type=Path, default=ROOT / 'results/position_eval')
    args = ap.parse_args(); output = args.output; output.mkdir(parents=True, exist_ok=True)
    split = next(e for e in json.loads((ROOT/'results/splits.json').read_text()) if e['bag'] == args.bag.name)
    if split['split'] != 'test':
        raise ValueError('This evidence script requires a held-out test bag')
    db = next(args.bag.glob('*.db3')); digest = hashlib.sha256(db.read_bytes()).hexdigest()
    if digest != split['sha256']:
        raise ValueError('Bag hash does not match the frozen split manifest')
    data, start_ns = read_bag(args.bag)
    config_path = ROOT/'src/tram_odometry/config/model.json'
    config = Config.from_dict(json.loads(config_path.read_text()))
    tracker = PositionTracker()
    out, _, state = replay(data, config, position=tracker)
    # Direct guard: removing all later GNSS must leave every replay output unchanged.
    without_late_gnss = dict(data)
    for key in ('master_fix','rover_fix','master_vel','rover_vel'):
        without_late_gnss[key] = data[key][data[key][:,0] <= tracker.start + tracker.init_seconds]
    check, _, _ = replay(without_late_gnss, config, position=PositionTracker())
    quarantine_identical = bool(np.array_equal(out, check))
    if not quarantine_identical:
        raise RuntimeError('Post-initialization GNSS affected replay outputs')
    if tracker.frame is None:
        raise RuntimeError('No valid initial master GNSS origin')
    fix = sorted_stream(data['master_fix'])
    prediction, matched = nearest(np.column_stack([out[:,0],out[:,0],out[:,7:10]]), fix[:,0], .05)
    truth = np.array([tracker.frame.enu(*v) for v in fix[:,2:5]])
    mask = matched & (fix[:,5] >= 0) & (abs(fix[:,2]) <= 90) & (abs(fix[:,3]) <= 180)
    mask &= np.isfinite(truth).all(axis=1) & np.isfinite(prediction).all(axis=1)
    mask &= fix[:,0] > tracker.start + tracker.init_seconds
    t = fix[mask,0]; truth = truth[mask]; prediction = prediction[mask]
    error2 = np.linalg.norm(prediction[:,:2]-truth[:,:2],axis=1)
    error3 = np.linalg.norm(prediction-truth,axis=1)
    g = sorted_stream(data['master_vel']); speed = np.hypot(g[:,2],g[:,3]); dt = np.diff(g[:,0])
    valid_speed = np.isfinite(speed) & (speed < 40)
    covered = valid_speed[1:] & valid_speed[:-1] & (dt > 0) & (dt < .25)
    covered &= (g[:-1,0] >= t[0]) & (g[1:,0] <= t[-1])
    travelled = float(np.sum(.5*(speed[:-1]+speed[1:])*dt*covered))
    duration_covered = float(dt[covered].sum())
    # Reference course is diagnostic only, never fed back to the estimator.
    yaw_pred, yaw_ok = nearest(np.column_stack([out[:,0],out[:,0],out[:,10]]), g[:,0], .05)
    moving = valid_speed & (speed > 1) & yaw_ok & (g[:,0] > tracker.start+3)
    course = np.arctan2(g[:,3],g[:,2])
    heading_error = np.abs(np.arctan2(np.sin(course-yaw_pred[:,0]),np.cos(course-yaw_pred[:,0])))
    early = moving & (g[:,0] < tracker.start+30)
    late = out[:,0] > tracker.start+3
    relative = evaluate_reference(data,out)
    geometric = dict(status='NOT VERIFIED', status_reason='No independent real route; diagnostic GNSS comparison does not establish route/pathgraph capability.',
        metric_type='horizontal Euclidean error in shared ENU, NOT along/cross-track error',
        position_2d=norm_stats(error2),position_3d=norm_stats(error3),
        initial_alignment_error_2d_m=float(error2[0]), initial_alignment_error_3d_m=float(error3[0]),
        initial_alignment_sample_s=float(t[0]), final_matched_sample_s=float(t[-1]),
        final_error_pct_of_reference_distance=float(100*error2[-1]/travelled),
        reference_distance_m=travelled, reference_covered_duration_s=duration_covered,
        reference_time_coverage_pct=100*duration_covered/(t[-1]-t[0]),
        matched_fix_count=len(t), total_fix_count=len(fix), along_track='NOT VERIFIED',cross_track='NOT VERIFIED')
    causes = dict(held_heading_deg=float(np.degrees(tracker.yaw)),
        estimated_heading_range_after_init_deg=float(np.degrees(np.ptp(out[late,10]))),
        moving_reference_heading_error_median_deg=float(np.degrees(np.median(heading_error[moving]))),
        moving_reference_heading_error_max_deg=float(np.degrees(np.max(heading_error[moving]))),
        early_moving_heading_error_median_deg=float(np.degrees(np.median(heading_error[early]))) if early.any() else None,
        reference_endpoint_displacement_2d_m=float(np.linalg.norm(truth[-1,:2]-truth[0,:2])),
        estimate_endpoint_displacement_2d_m=float(np.linalg.norm(prediction[-1,:2]-prediction[0,:2])),
        coordinate_origin_zero_error_m=float(np.linalg.norm(tracker.frame.enu(*tracker.frame.origin))),
        reference_bad_status_count=int(np.sum(fix[:,5] < 0)),
        reference_max_header_gap_s=float(np.max(np.diff(fix[:,0]))),
        interpretation='Compare small distance drift and initial alignment with large later heading differences and fixed estimated heading; no route was supplied. No claim of perfect GNSS ground truth or calibrated antenna lever arm.')
    result = dict(test_utc=datetime.now(timezone.utc).isoformat(),bag=args.bag.name,split='test',bag_sha256=digest,
        route_found=False,route_integrated=False,representation='Integrated longitudinal distance + local ENU with held heading; no runtime route',
        origin_lla=tracker.frame.origin,header_epoch_ns=start_ns,initialization_seconds=3,
        continuous_gnss_dependency=False,post_initialization_gnss_removal_identical_outputs=quarantine_identical,
        gnss_initialization_messages=tracker.gnss_used,model_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
        relative_distance=dict(status='VERIFIED',scope='GNSS-matched continuous intervals of this single full bag, not geometric along-track error',**relative),
        geometric_position=geometric,diagnosis=causes,all_outputs_finite=bool(np.isfinite(out).all()),
        offline_output_count=len(out),offline_last_time_s=float(out[-1,0]),
        limitations=['One complete test bag only; not whole-dataset evaluation.',
            'Initial alignment uses allowed GNSS; first scored sample is after the 3 s initialization window, not an independent initial-pose validation.',
            'Final error refers to last matched valid GNSS sample, not necessarily bag end.',
            'Drift percentage uses covered GNSS speed intervals; gaps are excluded and coverage is reported.',
            '2D MAE means mean Euclidean horizontal error, not coordinate-wise MAE.',
            'No route built from test GNSS; no along-track/cross-track claim.'])
    (output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    lines=[f"Bag: {args.bag.name} (test; full offline replay)",f"Real route found/integrated: NO / NO; origin LLA: {tracker.frame.origin}",
        'A. Relative distance (covered GNSS intervals): '+json.dumps(relative,indent=2),
        'B. No-map geometric position: NOT VERIFIED as route capability; diagnostic GNSS errors below.',json.dumps(geometric,indent=2),
        'Evidence for causes: '+json.dumps(causes,indent=2),'GNSS removal after initialization produces identical outputs: '+str(quarantine_identical)]
    (output/'summary.txt').write_text('\n'.join(lines)+'\n')
    row=dict(bag=args.bag.name,initial_alignment_2d_m=float(error2[0]),**geometric['position_2d'],
        geometric_final_drift_pct=geometric['final_error_pct_of_reference_distance'],
        relative_distance_error_m=relative['integrated_distance_error_m'],relative_distance_drift_pct=relative['relative_distance_drift_pct'],
        reference_distance_m=travelled,reference_time_coverage_pct=geometric['reference_time_coverage_pct'],along_track='NOT VERIFIED',cross_track='NOT VERIFIED')
    with (output/'per_bag_metrics.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(row));w.writeheader();w.writerow(row)
    np.savez_compressed(output/'comparison.npz',time=t,estimate_enu=prediction,reference_enu=truth,error_2d_m=error2,error_3d_m=error3,output=out)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(8,6));ax.plot(truth[:,0],truth[:,1],label='GNSS reference ONLY');ax.plot(prediction[:,0],prediction[:,1],label='Odometry: held heading, NO MAP')
    ax.scatter([truth[0,0]],[truth[0,1]],label='First post-init reference');ax.set(xlabel='East (m)',ylabel='North (m)',title=args.bag.name+' — no-map failure, not route validation');ax.axis('equal');ax.legend();ax.grid();fig.tight_layout();fig.savefig(output/'trajectory_comparison.png');plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
    for i,name in enumerate(['East','North']):
        axes[i].plot(t,truth[:,i],label='GNSS reference ONLY');axes[i].plot(t,prediction[:,i],label='Held-heading odometry');axes[i].set_ylabel(name+' (m)');axes[i].legend();axes[i].grid()
    axes[-1].set_xlabel('Header seconds relative to bag recording start');fig.suptitle('No-map position comparison');fig.tight_layout();fig.savefig(output/'position_comparison.png');plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,4));ax.plot(t,error2);ax.set(xlabel='Header seconds relative to bag recording start',ylabel='Horizontal Euclidean error (m)',title='Diagnostic position error — NOT along/cross-track');ax.grid();fig.tight_layout();fig.savefig(output/'position_error.png');plt.close(fig)
    print('\n'.join(lines))

if __name__=='__main__':main()
