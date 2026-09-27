"""Audit existing TRAIN map and DEV coverage without replaying TEST or tuning.

Reconstructs a temporary map from TRAIN only and compares its geometry exactly.
Leaves the delivered map, runtime and original result files untouched.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from bag_io import decode
from calibrate import sorted_stream
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/tram_odometry'))
from tram_odometry.position import LocalFrame


def fixes(bag):
    db=next(bag.glob('*.db3'))
    with sqlite3.connect(db.resolve().as_uri()+'?mode=ro',uri=True) as c:
        start=c.execute('select min(timestamp) from messages').fetchone()[0]
        tid=c.execute("select id from topics where name='/sensing/gnss/master/fix'").fetchone()[0]
        rows=[]
        for receipt,raw in c.execute('select timestamp,data from messages where topic_id=? order by timestamp,id',(tid,)):
            stamp,values=decode(raw,'master_fix');rows.append(((stamp-start)*1e-9,(receipt-start)*1e-9,*values))
    return np.asarray(rows,dtype=float)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,default=ROOT/'results/route_eval/audit')
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    splits=json.loads((ROOT/'results/splits.json').read_text());map_path=ROOT/'src/tram_odometry/config/train_path_map.json'
    m=json.loads(map_path.read_text());nodes=np.array(m['nodes']);frame=LocalFrame(m['origin_lla']);tree=cKDTree(nodes[:,:2])
    with tempfile.TemporaryDirectory(prefix='train_map_audit_') as temp:
        cache=Path(temp);train_extents=[]
        for e in splits:
            if e['split']!='train':continue
            f=fixes(ROOT/'dataset/data'/e['bag']);np.savez_compressed(cache/(e['bag']+'.npz'),master_fix=f)
            if len(f):
                valid=f[(f[:,5]>=0)&np.isfinite(f[:,2:5]).all(1)]
                if len(valid):
                    xy=np.array([frame.enu(*v)[:2] for v in valid[:,2:5]])
                    distance=tree.query(xy)[0]
                    train_extents.append(dict(bag=e['bag'],fixes=len(valid),bounds_xy=[xy.min(0).tolist(),xy.max(0).tolist()],fraction_fixes_within_5m_of_map=float(np.mean(distance<=5))))
        target=cache/'reconstructed.json'
        subprocess.run([sys.executable,str(ROOT/'tools/build_path_map.py'),str(cache),'--splits',str(ROOT/'results/splits.json'),'--output',str(target)],check=True)
        recreated=json.loads(target.read_text())
        identical=m['nodes']==recreated['nodes'] and m['origin_lla']==recreated['origin_lla'] and m['metadata']['train_bags']==recreated['metadata']['train_bags']
    # Proximity components are descriptive only: not rail connectivity or branch edges.
    pairs=tree.query_pairs(5.,output_type='ndarray')
    graph=coo_matrix((np.ones(len(pairs)),(pairs[:,0],pairs[:,1])),shape=(len(nodes),len(nodes)))
    count,labels=connected_components(graph,directed=False)
    geometry=dict(train_reconstruction_exact_match=identical,map_sha256=hashlib.sha256(map_path.read_bytes()).hexdigest(),
        split_counts={s:sum(e['split']==s for e in splits) for s in ['train','dev','test']},
        training_bags_used=len(m['metadata']['train_bags']),node_count=len(nodes),cell_size_m=m['cell_m'],
        nearest_neighbor_spacing_m=dict(median=float(np.median(tree.query(nodes[:,:2],k=2)[0][:,1])),max=float(np.max(tree.query(nodes[:,:2],k=2)[0][:,1]))),
        node_support_bags=dict(min=int(nodes[:,6].min()),median=float(np.median(nodes[:,6])),max=int(nodes[:,6].max())),
        directional_nodes_east=int(np.sum(nodes[:,3]>0)),directional_nodes_west=int(np.sum(nodes[:,3]<0)),
        proximity_components_at_5m=int(count),proximity_component_sizes=sorted(np.bincount(labels).tolist(),reverse=True),
        bounds_xy=[nodes[:,:2].min(0).tolist(),nodes[:,:2].max(0).tolist()],train_bag_extents=train_extents,
        limitations='2 m cells are NOT uniform 2 m point spacing. Directed nodes have no edges; branches and disconnected tracks cannot be certified from proximity components. Low-support terminal/depot tracks are removed by the existing >=3-bag filter.')
    (a.output/'train_provenance.json').write_text(json.dumps(geometry,indent=2)+'\n')
    rows=[];total=covered=0;node_errors=[];cross_errors=[]
    for e in splits:
        if e['split']!='dev':continue
        f=fixes(ROOT/'dataset/data'/e['bag'])
        if not len(f):rows.append(dict(bag=e['bag'],moving_samples=0,status='no fixes'));continue
        f=sorted_stream(f);f=f[(f[:,5]>=0)&np.isfinite(f[:,2:5]).all(1)]
        if len(f)<20:rows.append(dict(bag=e['bag'],moving_samples=0,status='insufficient fixes'));continue
        t=f[:,0];xy=np.array([frame.enu(*v)[:2] for v in f[:,2:5]])
        lo=np.searchsorted(t,t-1);hi=np.searchsorted(t,t+1,side='right')-1
        delta=xy[hi]-xy[lo];norm=np.linalg.norm(delta,axis=1);gaps=np.r_[0,np.cumsum(np.diff(t)>.25)]
        moving=(norm>=1.5)&(t[hi]-t[lo]>=1.6)&(gaps[hi]==gaps[lo])
        pts=xy[moving];direction=delta[moving]/norm[moving,None]
        if not len(pts):rows.append(dict(bag=e['bag'],moving_samples=0,status='no valid moving window'));continue
        dist,idx=tree.query(pts,k=min(64,len(nodes)));dots=np.sum(nodes[idx,3:5]*direction[:,None,:],axis=2)
        eligible=(dots>=.7)&(dist<=8);cost=np.where(eligible,dist,np.inf);j=cost.argmin(1)
        residual=cost[np.arange(len(pts)),j];n=nodes[idx[np.arange(len(pts)),j]];ok=residual<=3
        signed_cross=(pts[:,0]-n[:,0])*(-n[:,4])+(pts[:,1]-n[:,1])*n[:,3]
        total+=len(pts);covered+=int(ok.sum());node_errors.extend(residual[ok].tolist());cross_errors.extend(signed_cross[ok].tolist())
        rows.append(dict(bag=e['bag'],status='measured',moving_samples=len(pts),covered_samples=int(ok.sum()),coverage_pct=100*float(np.mean(ok)),
            covered_nearest_node_rmse_m=float(np.sqrt(np.mean(residual[ok]**2))) if ok.any() else None,
            covered_lateral_rmse_m=float(np.sqrt(np.mean(signed_cross[ok]**2))) if ok.any() else None,
            uncovered_samples=int((~ok).sum()),uncovered_bounds_xy=[pts[~ok].min(0).tolist(),pts[~ok].max(0).tolist()] if (~ok).any() else None))
    summary=dict(scope='DEV geometry only; no estimator replay or tuning; TEST not read',moving_definition='GNSS displacement >=1.5 m over centered +/-1s window, span >=1.6s, no gap >0.25s; directions NOT carried through stops',
        coverage_definition='Within 3 m of same-direction TRAIN node (dot >=0.7), among up to 64 nearest nodes within 8m',
        pooled_moving_samples=total,pooled_covered_samples=covered,pooled_coverage_pct=100*covered/total,
        covered_nearest_node_rmse_m=float(np.sqrt(np.mean(np.square(node_errors)))),
        covered_lateral_rmse_m=float(np.sqrt(np.mean(np.square(cross_errors)))),
        lateral_definition='Signed offset normal to nearest compatible map-node tangent; conditional on coverage, NOT graph station error',
        bags_without_moving_reference=[r['bag'] for r in rows if not r['moving_samples']],per_bag=rows)
    (a.output/'dev_geometry_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    fields=sorted({k for r in rows for k in r})
    with (a.output/'dev_geometry_metrics.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    print('TRAIN exact reconstruction:',identical,'components:',count,'spacing:',geometry['nearest_neighbor_spacing_m'])
    print(json.dumps({k:v for k,v in summary.items() if k!='per_bag'},indent=2))
    if not identical:raise SystemExit('Existing map provenance could not be reproduced')
    audit_saved_test_metrics(a.output)

def audit_saved_test_metrics(output):
    """Postprocess saved scalar metrics; does not open TEST bags or run replay."""
    import math
    import statistics
    with (ROOT/'results/route_eval/test_metrics.csv').open() as f:
        rows=list(csv.DictReader(f))
    reference={r['bag']:r['metrics'] for r in json.loads((ROOT/'results/test/metrics.json').read_text())['bags'] if r['method']=='model_observer'}
    report={}
    for method in ['no_map','path_map']:
        n=sum(int(r[method+'_samples']) for r in rows)
        report[method]={
            'all_bags_pooled_position_rmse_m':math.sqrt(sum(int(r[method+'_samples'])*float(r[method+'_rmse_m'])**2 for r in rows)/n),
            'samples':n,
            'global_along_max_abs_m':max(float(r[method+'_along_max_abs']) for r in rows if r[method+'_along_max_abs']),
            'global_cross_max_abs_m':max(float(r[method+'_cross_max_abs']) for r in rows if r[method+'_cross_max_abs']),
            'median_endpoint_error_over_covered_speed_distance_pct':statistics.median(100*float(r[method+'_final_error_m'])/reference[r['bag']]['reference_distance_m'] for r in rows if reference[r['bag']]['reference_distance_m']>50)}
    for r in rows:
        distance=reference[r['bag']]['reference_distance_m']
        r['reference_speed_covered_distance_m']=distance
        r['reference_speed_integrated_error_m']=reference[r['bag']]['integrated_distance_error_m']
        r['position_error_over_covered_speed_distance_pct']=100*float(r['path_map_final_error_m'])/distance if distance>50 else None
    report['caveats']=[
        'Existing saved metrics only; no TEST replay and no map/runtime changes.',
        'Original travelled_m sums noisy GNSS fixes without outlier/gap rejection; whole-run drift NOT VERIFIED.',
        'Covered GNSS-speed distance is an alternative denominator with different coverage, not verified whole-run drift.',
        'Pooled along/cross metrics use incorrect direction-sample weights in the original evaluator; NOT VERIFIED.',
        'Along/cross are local reference-direction components, not graph arc-length station errors.']
    (output/'test_reporting_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    with (output/'test_metrics_annotated.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

if __name__=='__main__':main()
