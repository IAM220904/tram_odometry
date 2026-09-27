"""Build a directed track-node map from TRAIN-split GNSS master fixes only.

Nodes are 2 m cells x 8 heading sectors; each keeps the mean position and mean
travel direction of all training traversals, and only cells seen in at least
MIN_BAGS different training bags are kept (rejects GNSS noise and one-off
detours). Dev/test bags are never read by this tool.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/tram_odometry'))
from tram_odometry.position import LocalFrame
from bag_io import load_cache
from calibrate import sorted_stream

CELL=2.
SECTORS=8
MIN_BAGS=3
MIN_MOVE=1.5  # metres over the +-1 s direction window


def train_tracks(cache,entries):
    for e in entries:
        if e['split']!='train':continue
        f=load_cache(cache/(e['bag']+'.npz'))['master_fix']
        if not len(f):continue
        f=sorted_stream(f)
        f=f[(f[:,5]>=0)&np.isfinite(f[:,2:5]).all(1)]
        if len(f)>10:yield e['bag'],f


def main():
    ap=argparse.ArgumentParser();ap.add_argument('cache',type=Path);ap.add_argument('--splits',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();entries=json.loads(args.splits.read_text())
    tracks=list(train_tracks(args.cache,entries))
    origin=tuple(float(v) for v in np.median(np.vstack([f[:,2:5] for _,f in tracks]),0))
    frame=LocalFrame(origin)
    cells={};used=[];rejected=0;total=0
    for bag,f in tracks:
        t=f[:,0];p=np.array([frame.enu(*r[2:5]) for r in f])
        # Outlier rejection: implied speed between consecutive fixes above 30 m/s.
        step=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1);dt=np.diff(t)
        ok=np.r_[True,step<=30*np.maximum(dt,.05)+2.]
        t,p=t[ok],p[ok];rejected+=int((~ok).sum());total+=len(ok)
        lo=np.searchsorted(t,t-1.);hi=np.searchsorted(t,t+1.,side='right')-1
        d=p[hi,:2]-p[lo,:2];n=np.linalg.norm(d,axis=1)
        moving=(n>=MIN_MOVE)&(t[hi]-t[lo]<=2.5)
        if not moving.any():continue
        used.append(bag)
        for (x,y,z),(dx,dy),m in zip(p[moving],d[moving],n[moving]):
            tx,ty=dx/m,dy/m;sector=int(((math.atan2(ty,tx)+math.pi)/(2*math.pi)*SECTORS))%SECTORS
            key=(math.floor(x/CELL),math.floor(y/CELL),sector)
            c=cells.setdefault(key,[0.,0.,0.,0.,0.,0,set()])
            c[0]+=x;c[1]+=y;c[2]+=z;c[3]+=tx;c[4]+=ty;c[5]+=1;c[6].add(bag)
    nodes=[]
    for c in cells.values():
        if len(c[6])<MIN_BAGS:continue
        norm=math.hypot(c[3],c[4])
        if norm<.5*c[5]:continue  # inconsistent directions inside the cell
        nodes.append([round(c[0]/c[5],3),round(c[1]/c[5],3),round(c[2]/c[5],3),round(c[3]/norm,5),round(c[4]/norm,5),c[5],len(c[6])])
    result=dict(type='directed_track_nodes',origin_lla=list(origin),cell_m=CELL,
                node_fields=['east_m','north_m','up_m','dir_east','dir_north','samples','train_bags'],nodes=nodes,
                metadata=dict(generated_from='TRAIN split GNSS master fixes only (results/splits.json split=="train")',
                              train_bags=used,train_bags_without_moving_gnss=[e['bag'] for e in entries if e['split']=='train' and e['bag'] not in used],
                              method=f'{CELL:g} m cells x {SECTORS} heading sectors; mean position and unit travel direction; '
                                     f'direction from +-1 s fix difference >= {MIN_MOVE} m; keep cells seen in >= {MIN_BAGS} train bags; '
                                     'fixes with implied speed > 30 m/s rejected',
                              frame='local ENU (east, north, up) metres about origin_lla (WGS84 lat, lon, alt); origin = median of train fixes',
                              point_spacing_m=CELL,fixes=total,outlier_fixes_rejected=rejected,
                              generated_utc=datetime.now(timezone.utc).isoformat(timespec='seconds')))
    args.output.write_text(json.dumps(result,separators=(',',':')))
    print(f'{len(nodes)} nodes from {len(used)} train bags; {rejected}/{total} fixes rejected; origin {origin}')


if __name__=='__main__':main()
