"""One-command jury report built only from saved evidence files.

Reads results/*, never replays TEST, never changes estimator/map/config.
Writes results/FINAL_REPORT.txt, results/FINAL_REPORT.md, results/final_report.json,
two lightweight summary plots, and results/final_demo/ (copies of key plots).
"""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
R=ROOT/'results'
PROTECTED=['src/tram_odometry/config/model.json','src/tram_odometry/config/train_path_map.json',
           'results/splits.json','src/tram_odometry/tram_odometry/core.py','src/tram_odometry/tram_odometry/position.py']
EVIDENCE={
 'velocity_summary':'results/velocity_eval/summary.json','velocity_live':'results/velocity_eval/live_ros_check.json',
 'test_metrics':'results/test/metrics.json','splits':'results/splits.json',
 'route_dev':'results/route_eval/dev_summary.json','route_test':'results/route_eval/test_summary.json',
 'route_test_csv':'results/route_eval/test_metrics.csv','route_audit':'results/route_eval/audit/test_reporting_audit.json',
 'route_dev_audit':'results/route_eval/audit/dev_geometry_summary.json','route_runtime':'results/route_eval/audit/runtime_check.json',
 'robustness':'results/robustness_eval/summary.json','performance':'results/performance/performance.json',
 'path_map':'src/tram_odometry/config/train_path_map.json'}
PLOTS={
 'Velocity':['results/velocity_eval/velocity_comparison.png','results/velocity_eval/velocity_error.png',
             'results/velocity_eval/velocity_zoom_transients.png','results/velocity_eval/regime_metrics.png',
             'results/velocity_eval/live_vs_offline.png'],
 'Position':['results/route_eval/route_map.png','results/route_eval/dev_route_overlay.png',
             'results/route_eval/test_trajectory_comparison.png','results/route_eval/along_track_error.png',
             'results/route_eval/cross_track_error.png'],
 'Robustness':['results/robustness_eval/scenario_plots.png','results/final_demo/robustness_summary.png'],
 'Performance':['results/final_demo/performance_summary.png']}


def sha(p):return hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
def load(key):return json.loads((ROOT/EVIDENCE[key]).read_text())
def f(x,n=4):return 'n/a' if x is None else f'{x:.{n}f}'


def robustness_plot(rob,out):
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    sc=rob['scenarios'];names=[s['scenario'] for s in sc]
    err=[s['max_velocity_error_mps'] for s in sc];dist=[s['max_relative_distance_deviation_m'] for s in sc]
    col=['#2a9d8f' if s['status']=='PASS' else '#d62828' for s in sc]
    fig,ax=plt.subplots(1,2,figsize=(13,5.5))
    for a,vals,lim,title in [(ax[0],err,1.,'Max speed error, m/s (log)'),(ax[1],dist,10.,'Max distance deviation vs clean replay, m (log)')]:
        a.barh(names,[max(v,1e-4) for v in vals],color=col);a.set_xscale('log');a.axvline(lim,color='k',ls='--',lw=1)
        a.set_title(title+f'\n dashed = pre-declared limit {lim:g}');a.invert_yaxis();a.grid(alpha=.3,axis='x')
    ax[1].set_yticklabels([])
    fig.suptitle(f"Synthetic fault scenarios: {rob['scenario_count']-len(rob['failed'])} PASS (green), {len(rob['failed'])} FAIL (red). Not real labelled slip. Values below 1e-4 (incl. exact 0) drawn at 1e-4.")
    fig.tight_layout();fig.savefig(out,dpi=110);plt.close(fig)


def performance_plot(perf,out):
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    rt=perf['rates'];res=perf['resources'];px=perf['latency_proxies']['/result/velocity']['all_inputs_pooled']['delivery_latency_proxy_ms']
    fig,ax=plt.subplots(1,4,figsize=(15,4))
    ax[0].bar(['velocity','position'],[rt['/result/velocity']['hz_count_over_window'],rt['/result/position']['hz_count_over_window']],color='#264653')
    ax[0].axhline(10,color='#d62828',ls='--',label='required ≥10 Hz');ax[0].set_title('Publication rate, Hz');ax[0].legend()
    ax[1].bar(['mean','sampled peak'],[res['effective_cores_mean'],res['effective_cores_sampled_peak']],color='#2a9d8f')
    ax[1].axhline(2,color='#d62828',ls='--',label='limit 2 cores');ax[1].set_title('Node CPU, cores');ax[1].legend()
    ax[2].bar(['mean','max'],[res['rss_mb']['mean'],res['rss_mb']['max']],color='#8d99ae')
    ax[2].axhline(500,color='#d62828',ls='--',label='limit 500 MB');ax[2].set_title('Node RSS, MB');ax[2].legend()
    ax[3].bar(['mean','median','max'],[px['mean'],px['median'],px['max']],color='#e9c46a')
    ax[3].set_title('LATENCY PROXY input→/result/velocity, ms\n(not causal latency; NOT VERIFIED vs 100 ms)',fontsize=9)
    for a in ax:a.grid(alpha=.3,axis='y')
    fig.suptitle(f"Short ROS run, {perf['observation_seconds']:.0f} s observation, bag {Path(perf['bag']).name}")
    fig.tight_layout();fig.savefig(out,dpi=110);plt.close(fig)


def main():
    missing=[p for p in EVIDENCE.values() if not (ROOT/p).exists()]
    if missing:
        print('ERROR: missing evidence files:\n  '+'\n  '.join(missing));sys.exit(2)
    before={p:sha(p) for p in PROTECTED}
    E={k:load(k) for k in EVIDENCE if not k.endswith('_csv')}
    demo=R/'final_demo';demo.mkdir(exist_ok=True)
    if not (demo/'robustness_summary.png').exists():robustness_plot(E['robustness'],demo/'robustness_summary.png')
    if not (demo/'performance_summary.png').exists():performance_plot(E['performance'],demo/'performance_summary.png')
    notes=[]
    # ---- Criterion 1
    vs=E['velocity_summary']['summary'];mo=vs['methods']['model_observer'];ov=mo['overall']
    tm=E['test_metrics']['summary']['model_observer']['raw']
    if tm['samples']!=ov['samples'] or abs(tm['rmse']-ov['rmse'])>1e-9:notes.append('Velocity: results/test/metrics.json disagrees with velocity_eval/summary.json')
    live=E['velocity_live'];perf=E['performance'];rates=perf['rates']
    C1=dict(split='test',bags=vs['bags_evaluated'],bags_in_split=vs['bags_in_split'],matched_samples=ov['samples'],
            rmse=ov['rmse'],mae=ov['mae'],bias=ov['bias'],max_abs=ov['max_abs'],
            filtered_gnss_agree=mo['overall_gnss_agree'],
            regimes={r:{k:mo[r][k] for k in ['samples','rmse','mae','bias']} for r in ['acceleration','braking','steady','stop']},
            stop_gnss_agree={k:mo['stop_gnss_agree'][k] for k in ['samples','rmse','mae','bias']},
            baseline_wheel_mean=vs['methods']['wheel_mean']['overall'],
            live_minus_offline_rmse=live['live_minus_offline']['rmse'],live_minus_offline_max=live['live_minus_offline']['max_abs'],
            live_check_bag=live['bag'],live_check_seconds=live['header_span_s'],
            velocity_rate_hz=rates['/result/velocity']['hz_count_over_window'])
    # ---- Criterion 2
    rt=E['route_test']['aggregate_median_over_bags'];pm=rt['path_map'];nm=rt['no_map'];au=E['route_audit']
    da=E['route_dev_audit'];dv=E['route_dev']['aggregate_median_over_bags']['path_map']
    if abs(E['route_test']['aggregate_median_over_bags']['path_map']['pooled_rmse_m']-au['path_map']['all_bags_pooled_position_rmse_m'])>1e-6:
        notes.append(f"Position pooled RMSE: route_eval/test_summary.json {pm['pooled_rmse_m']:.2f} m vs audit-corrected "
                     f"{au['path_map']['all_bags_pooled_position_rmse_m']:.2f} m (audit recomputed from CSV totals; audit value used)")
    rows=list(csv.DictReader(open(ROOT/EVIDENCE['route_test_csv'])))
    fails30=[r['bag'] for r in rows if r['status']=='evaluated' and float(r['path_map_final_error_m'])>30]
    fails50=E['route_test']['failures']['path_map_final_error_over_50m']
    counts={s:sum(e['split']==s for e in E['splits']) for s in ['train','dev','test']}
    rrt=E['route_runtime']
    C2=dict(split=counts,path_map='src/tram_odometry/config/train_path_map.json',map_source='TRAIN GNSS master fixes only',
            map_train_bags=len(E['path_map']['metadata']['train_bags']),map_nodes=len(E['path_map']['nodes']),
            dev_coverage_pct_pooled_audited=da['pooled_coverage_pct'],dev_lateral_rmse_m_audited=da['covered_lateral_rmse_m'],
            dev_failures=E['route_dev']['failures']['path_map_final_error_over_50m'],
            test_bags=E['route_test']['bags_evaluated'],
            median=dict(final_error_m=pm['final_error_m'],rmse_m=pm['rmse_m'],mae_m=pm['mae_m'],final_drift_pct_unreliable_denominator=pm['final_drift_pct'],
                        along_mean=pm['along_mean'],along_rmse=pm['along_rmse'],along_max=pm['along_max_abs'],
                        cross_mean=pm['cross_mean'],cross_rmse=pm['cross_rmse'],cross_max=pm['cross_max_abs']),
            pooled=dict(position_rmse_m=au['path_map']['all_bags_pooled_position_rmse_m'],along_cross='NOT VERIFIED (weighting bug found by audit)'),
            global_max=dict(final_error_m=pm['max_final_error_m'],along_abs_m=au['path_map']['global_along_max_abs_m'],cross_abs_m=au['path_map']['global_cross_max_abs_m']),
            no_map=dict(median_final_error_m=nm['final_error_m'],median_rmse_m=nm['rmse_m'],pooled_rmse_m=au['no_map']['all_bags_pooled_position_rmse_m'],worst_final_m=nm['max_final_error_m']),
            whole_run_drift='NOT VERIFIED',endpoint_over_covered_speed_distance_median_pct=au['path_map']['median_endpoint_error_over_covered_speed_distance_pct'],
            failures_final_gt_30m=fails30,failures_final_gt_50m=fails50,
            runtime_path_map_verified=dict(status='VERIFIED (short DEV run)' if rrt.get('alive') and rrt.get('finite') else 'NOT VERIFIED',
                                           bag=rrt['bag'],seconds=rrt['seconds'],messages=rrt['position_messages'],last_status=rrt['last_status']))
    # ---- Criterion 3
    rob=E['robustness'];sc={s['scenario']:s for s in rob['scenarios']};rr=rob['runtime_results']
    failed=[dict(scenario=n,max_speed_error_mps=sc[n]['max_velocity_error_mps'],distance_deviation_m=sc[n]['max_relative_distance_deviation_m'],
                 recovery_s=sc[n]['recovery_time_s']) for n in rob['failed']]
    caps=[('Front/rear disagreement detection','YES','front_slip_5mps, moderate_disagreement PASS; slip flag set'),
          ('Model residual detection','PARTIAL','both_slip_short PASS; both_slip_long FAIL (sustained consistent slip reacquired)'),
          ('Jump/outlier rejection','YES','front_spike, rear_spike PASS; rejected inputs change speed by 0.0 m/s'),
          ('Trust reduction/downweighting','YES',f"R x9 on disagreement; update ratio {rob['trust_reduction']['update_magnitude_ratio']:.2f}"),
          ('Model fallback','YES','both_missing PASS; model_only observed in ROS'),
          ('Stale/missing-input handling','YES','both_missing, rear_missing_long, command_missing PASS; command_stale observed in ROS'),
          ('Recovery','PARTIAL','recovery <=0.5 s in passing cases; forward_header_spike never recovers'),
          ('Diagnostics/slip status','YES','/result/diagnostics slip flag, model_only, invalid counts observed in ROS'),
          ('ROS crashes','NONE OBSERVED',f"short real-bag + synthetic ROS runs alive, exit codes {rr['real_bag']['exit_codes']}"),
          ('NaN/Inf outputs','NONE OBSERVED','all 15 cases finite; ROS outputs finite'),
          ('Real labelled slip validation','NOT VERIFIED','no labels supplied'),
          ('Slip-ratio estimation','NO','not implemented'),
          ('Adhesion-coefficient estimation','NO','not implemented')]
    C3=dict(scenarios=rob['scenario_count'],passed=rob['scenario_count']-len(rob['failed']),failed=failed,criteria=rob['criteria'],
            capabilities=[dict(item=a,status=b,evidence=c) for a,b,c in caps],unit_tests=rob['unit_tests_after'])
    # ---- Criterion 4
    res=perf['resources'];prx=perf['latency_proxies']['/result/velocity']['all_inputs_pooled']['delivery_latency_proxy_ms']
    prp=perf['latency_proxies']['/result/position']['all_inputs_pooled']['delivery_latency_proxy_ms'];req=perf['requirements']
    C4=dict(velocity_hz=rates['/result/velocity']['hz_count_over_window'],position_hz=rates['/result/position']['hz_count_over_window'],
            max_interval_ms=max(rates['/result/velocity']['interval_ms']['max'],rates['/result/position']['interval_ms']['max']),
            cpu_percent_mean=res['cpu_percent_mean'],cpu_percent_sampled_peak=res['cpu_percent_sampled_peak'],
            effective_cores_mean=res['effective_cores_mean'],effective_cores_peak=res['effective_cores_sampled_peak'],
            rss_mb_mean=res['rss_mb']['mean'],rss_mb_max=res['rss_mb']['max'],observation_s=perf['observation_seconds'],
            short_run_alive=perf['node_and_player_alive_entire_observation'],build_without_internet=perf['build'],
            latency_proxy_velocity_ms=dict(mean=prx['mean'],median=prx['median'],max=prx['max']),
            latency_proxy_position_ms=dict(mean=prp['mean'],median=prp['median'],max=prp['max']),
            causal_latency_le_100ms=req['causal_input_to_output_latency_le_100_ms'],causal_peak_le_250ms=req['causal_peak_latency_le_250_ms'],
            long_term_stability=req['long_term_memory_stability'])
    # ---- Text
    L=[];bar='='*70
    def H(t):L.extend(['',bar,t,bar])
    L+=['TRAM BACKUP ODOMETRY - HACKATHON TECHNICAL REPORT','Built only from saved evidence files (no TEST replay, no retuning).',
        'Evidence status labels are ours, NOT an official jury score.']
    H('CRITERION 1 - VELOCITY ACCURACY (0-30)')
    L+=['Measures: estimated longitudinal speed vs GNSS master speed on held-out TEST bags;',
        'error = estimate - GNSS; causal replay of the same core used by the ROS node.',
        f"Held-out test bags: {C1['bags']}/{C1['bags_in_split']}    matched samples: {C1['matched_samples']}",
        f"MAIN (all matched samples):  RMSE {f(C1['rmse'])} m/s   MAE {f(C1['mae'])} m/s   bias {C1['bias']:+.4f} m/s   max |e| {f(C1['max_abs'],3)} m/s",
        f"Alternative, labelled FILTERED (rover agrees with master within 0.5 m/s): RMSE {f(C1['filtered_gnss_agree']['rmse'])}  MAE {f(C1['filtered_gnss_agree']['mae'])}  bias {C1['filtered_gnss_agree']['bias']:+.4f}  n={C1['filtered_gnss_agree']['samples']}",
        f"Baseline (mean of wheels /3.6): RMSE {f(C1['baseline_wheel_mean']['rmse'])}  MAE {f(C1['baseline_wheel_mean']['mae'])}",
        '','Regime (from GNSS reference)   samples     RMSE      MAE      bias']
    for r,v in C1['regimes'].items():L.append(f"  {r:28s} {v['samples']:8d}  {f(v['rmse'])}   {f(v['mae'])}   {v['bias']:+.4f}")
    s=C1['stop_gnss_agree'];L.append(f"  {'stop, FILTERED (see above)':28s} {s['samples']:8d}  {f(s['rmse'])}   {f(s['mae'])}   {s['bias']:+.4f}")
    L+=['  (raw stop RMSE is dominated by 27 master-GNSS dropouts to ~0 m/s while rover agrees with the estimate)',
        f"Live ROS /result/velocity vs offline replay: RMSE {f(C1['live_minus_offline_rmse'])} m/s, max {f(C1['live_minus_offline_max'],3)} m/s ({C1['live_check_bag']}, {C1['live_check_seconds']:.0f} s)",
        f"/result/velocity publication rate: {C1['velocity_rate_hz']:.1f} Hz",
        'Plots: '+', '.join(PLOTS['Velocity'])]
    H('CRITERION 2 - POSITION ACCURACY (0-35)')
    L+=['Measures: horizontal position vs GNSS master fix after the 3 s GNSS initialisation window;',
        'along/cross = error components relative to local GNSS travel direction (not graph station error).',
        f"Split (results/splits.json): train {counts['train']} / dev {counts['dev']} / test {counts['test']}",
        f"Path map: {C2['path_map']}  ({C2['map_nodes']} directed nodes; source: {C2['map_source']}, {C2['map_train_bags']} train bags)",
        f"DEV coverage (audited, pooled moving samples within 3 m): {C2['dev_coverage_pct_pooled_audited']:.2f}%",
        f"DEV lateral RMSE (audited, covered samples): {C2['dev_lateral_rmse_m_audited']:.3f} m     DEV failures >50 m: {', '.join(C2['dev_failures']) or 'none'}",
        f"TEST bags evaluated: {C2['test_bags']}",'',
        'MEDIAN PER-BAG (TEST, train map)                     NO MAP (same bags)',
        f"  final position error     {pm['final_error_m']:9.2f} m              {nm['final_error_m']:9.1f} m",
        f"  position RMSE            {pm['rmse_m']:9.2f} m              {nm['rmse_m']:9.1f} m",
        f"  position MAE             {pm['mae_m']:9.2f} m",
        f"  along mean/RMSE/max      {pm['along_mean']:+.2f} / {pm['along_rmse']:.2f} / {pm['along_max_abs']:.2f} m",
        f"  cross mean/RMSE/max      {pm['cross_mean']:+.2f} / {pm['cross_rmse']:.2f} / {pm['cross_max_abs']:.2f} m",
        f"  final drift              {pm['final_drift_pct']:.3f}% of GNSS fix-path length - denominator unreliable; whole-run drift NOT VERIFIED",
        f"                           (alt. endpoint error / covered GNSS-speed distance: {C2['endpoint_over_covered_speed_distance_median_pct']:.3f}%, different coverage)",
        'POOLED (all TEST samples)',
        f"  position RMSE            {C2['pooled']['position_rmse_m']:9.2f} m (audit-corrected)   no map {C2['no_map']['pooled_rmse_m']:.1f} m",
        '  along/cross pooled       NOT VERIFIED',
        'GLOBAL MAXIMUM (TEST)',
        f"  worst final error        {C2['global_max']['final_error_m']:9.2f} m              {C2['no_map']['worst_final_m']:9.1f} m",
        f"  max |along| / |cross|    {C2['global_max']['along_abs_m']:.1f} / {C2['global_max']['cross_abs_m']:.1f} m",
        f"Failure threshold final >30 m: {len(fails30)} bags: {', '.join(fails30)}",
        f"                  final >50 m: {len(fails50)} bags",
        f"Runtime with path_map_file: {C2['runtime_path_map_verified']['status']} - {rrt['bag']}, {rrt['seconds']:.0f} s, {rrt['position_messages']} finite msgs, status '{rrt['last_status']}'",
        'Plots: '+', '.join(PLOTS['Position'])]
    H('CRITERION 3 - SLIP / ANOMALY ROBUSTNESS (0-20)')
    L+=['Measures: behaviour under injected wheel faults on a synthetic analytic speed profile (not real labelled slip).',
        f"Pre-declared pass criteria: {rob['criteria']}",
        f"Synthetic scenarios: {C3['scenarios']}   passed: {C3['passed']}   failed: {len(failed)}"]
    for x in failed:L.append(f"  FAILED {x['scenario']}: max speed error {x['max_speed_error_mps']:.3f} m/s, distance deviation {x['distance_deviation_m']:.1f} m, recovery {'never' if x['recovery_s'] is None else format(x['recovery_s'],'.1f')+' s'}")
    L+=['',f"  {'Capability':34s} {'Status':14s} Evidence"]
    for c in C3['capabilities']:L.append(f"  {c['item']:34s} {c['status']:14s} {c['evidence']}")
    L+=[f"Unit tests (robustness audit): {C3['unit_tests']['passed']} passed, {C3['unit_tests']['failed']} failed",
        'Plots: '+', '.join(PLOTS['Robustness'])]
    H('CRITERION 4 - REAL-TIME PERFORMANCE (0-15)')
    L+=[f"Measures: short ROS 2 run, bag {Path(perf['bag']).name}, node process only.",
        f"/result/velocity rate: {C4['velocity_hz']:.2f} Hz    /result/position rate: {C4['position_hz']:.2f} Hz    max output interval: {C4['max_interval_ms']:.1f} ms",
        f"CPU mean {C4['cpu_percent_mean']:.1f}%  sampled peak {C4['cpu_percent_sampled_peak']:.1f}%  -> effective cores mean {C4['effective_cores_mean']:.3f}, peak {C4['effective_cores_peak']:.3f} (limit 2)",
        f"RSS mean {C4['rss_mb_mean']:.1f} MB, max {C4['rss_mb_max']:.1f} MB (limit 500 MB)",
        f"Observation duration: {C4['observation_s']:.1f} s    short-run alive: {C4['short_run_alive']}",
        f"Build without internet: {'VERIFIED' if C4['build_without_internet'].get('exit_code')==0 else 'NOT VERIFIED'} (colcon exit {C4['build_without_internet'].get('exit_code')}, {C4['build_without_internet'].get('duration_seconds')} s, network-restricted; results/performance/build.log)",
        'LATENCY PROXY (observer receipt of input -> next output; NOT verified causal latency):',
        f"  -> /result/velocity  mean {prx['mean']:.2f} ms  median {prx['median']:.2f} ms  max {prx['max']:.2f} ms",
        f"  -> /result/position  mean {prp['mean']:.2f} ms  median {prp['median']:.2f} ms  max {prp['max']:.2f} ms",
        f"Causal latency <=100 ms: {C4['causal_latency_le_100ms']}",f"Causal peak <=250 ms: {C4['causal_peak_le_250ms']}",
        f"Long-term stability: {C4['long_term_stability']}",'Plots: '+', '.join(PLOTS['Performance'])]
    H('SUMMARY')
    summ=[('Velocity accuracy',f"RMSE {C1['rmse']:.3f} m/s, MAE {C1['mae']:.3f}, bias {C1['bias']:+.3f} (20 test bags)",'strong'),
          ('Position accuracy',f"median final {pm['final_error_m']:.1f} m, pooled RMSE {C2['pooled']['position_rmse_m']:.1f} m; {len(fails30)}/20 bags >30 m",'partial'),
          ('Slip/anomaly robustness',f"{C3['passed']}/{C3['scenarios']} synthetic PASS; no real slip labels",'partial'),
          ('Real-time performance',f"20 Hz, {C4['effective_cores_peak']:.2f} cores peak, {C4['rss_mb_max']:.0f} MB; causal latency NOT VERIFIED",'partial')]
    L+=[f"{'Criterion':26s} {'Result':74s} Evidence status",'-'*112]
    L+=[f"{a:26s} {b:74s} {c}" for a,b,c in summ]
    L+=['','Strongest verified results:',
        f"  - Velocity RMSE {C1['rmse']:.4f} m/s on 20 held-out bags, 16.5% below wheel-mean baseline; live ROS matches offline (RMSE {C1['live_minus_offline_rmse']:.4f} m/s).",
        f"  - Train-only map: median final position error {pm['final_error_m']:.1f} m vs {nm['final_error_m']:.0f} m without map.",
        f"  - 20 Hz output, {C4['effective_cores_peak']:.2f} CPU cores peak, {C4['rss_mb_max']:.0f} MB RSS; no crashes or NaN/Inf observed.",
        'NOT VERIFIED:','  - causal latency <=100 ms and peak <=250 ms','  - long-term stability (only ~45 s runs)',
        '  - whole-run position drift %; pooled along/cross-track metrics','  - real labelled slip; slip ratio / adhesion not implemented',
        '  - organiser permission to use a map built from training GNSS',
        'Known failures/limitations:',f"  - position: {len(fails30)} test bags >30 m final error ({', '.join(fails30)}); dev bag {', '.join(C2['dev_failures'])} never engages the map",
        '  - robustness: '+'; '.join(f"{x['scenario']} ({x['max_speed_error_mps']:.2f} m/s, {x['distance_deviation_m']:.0f} m)" for x in failed),
        '  - velocity: small negative bias (-0.02 m/s); 3 test bags with RMSE >0.1 m/s',
        '  - map fallback heading for stationary starts is applied after follower initialisation (audit finding, not fixed)']
    if notes:L+=['','EVIDENCE INCONSISTENCIES:']+['  - '+n for n in notes]
    L+=['','Evidence documents: docs/VELOCITY_EVAL.md, docs/ROUTE_EVAL.md, docs/ROBUSTNESS_EVAL.md, results/performance/performance.txt']
    # ---- Checks
    allplots=[p for v in PLOTS.values() for p in v];absent=[p for p in allplots if not (ROOT/p).exists()]
    for p in allplots:
        if (ROOT/p).exists() and not p.startswith('results/final_demo/'):shutil.copy2(ROOT/p,demo/(p.split('/')[1]+'__'+Path(p).name))
    after={p:sha(p) for p in PROTECTED}
    checks=dict(protected_files_unchanged=before==after,missing_plots=absent,evidence_inconsistencies=notes)
    L+=['',f"Self-check: protected files unchanged={checks['protected_files_unchanged']}; missing plots={absent or 'none'}"]
    text='\n'.join(L)+'\n'
    (R/'FINAL_REPORT.txt').write_text(text)
    md=['# Tram backup odometry — hackathon technical report','','```text']
    for l in L[1:]:
        if l==bar:continue
        if l.startswith('CRITERION') or l=='SUMMARY':md+=['```','','## '+l,'','```text']
        else:md.append(l)
    md+=['```','','## Plots','']
    for k,v in PLOTS.items():md+=[f'### {k}','']+[f'![{Path(p).name}]({Path(p).relative_to("results")})' for p in v]+['']
    (R/'FINAL_REPORT.md').write_text('\n'.join(md)+'\n')
    (R/'final_report.json').write_text(json.dumps(dict(criterion_1_velocity=C1,criterion_2_position=C2,criterion_3_robustness=C3,
        criterion_4_performance=C4,summary=[dict(criterion=a,result=b,evidence_status=c) for a,b,c in summ],
        plots=PLOTS,checks=checks,protected_sha256=after,note='Evidence status is not an official jury score.'),indent=2,default=str))
    shutil.copy2(R/'FINAL_REPORT.txt',demo/'FINAL_REPORT.txt')
    print(text,end='')
    print(f"Written: results/FINAL_REPORT.txt, results/FINAL_REPORT.md, results/final_report.json, results/final_demo/ ({len(list(demo.iterdir()))} files)")
    if absent or not checks['protected_files_unchanged']:sys.exit(3)


if __name__=='__main__':main()
