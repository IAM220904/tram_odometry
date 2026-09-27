"""Extract timestamp-preserving caches and dataset audit. Python + NumPy only."""
import argparse
import csv
import json
import zipfile
from pathlib import Path
import numpy as np
from bag_io import TOPICS, messages


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('data', type=Path)
    ap.add_argument('--cache', type=Path, default=Path('cache'))
    ap.add_argument('--output', type=Path, default=Path('results'))
    args = ap.parse_args()
    args.cache.mkdir(parents=True, exist_ok=True)
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for idx, db in enumerate(sorted(args.data.rglob('*.db3'))):
        target = args.cache / (db.parent.name + '.npz')
        if not target.exists() or not zipfile.is_zipfile(target):
            streams = {k: [] for k in TOPICS.values()}
            start = None
            for receipt, stamp, kind, values in messages(db):
                if start is None:
                    start = receipt
                streams[kind].append(((stamp-start)*1e-9, (receipt-start)*1e-9, *values))
            arrays = {k: np.asarray(v, dtype=float) for k, v in streams.items()}
            temp = target.with_suffix('.tmp.npz')
            np.savez_compressed(temp, start_ns=np.array(start, dtype=np.int64), **arrays)
            temp.replace(target)
        with np.load(target) as arrays:
            for key in TOPICS.values():
                a = arrays[key]
                if not len(a):
                    continue
                t, receipt, val = a[:, 0], a[:, 1], a[:, 2:]
                dt = np.diff(t)
                lag = receipt - t
                row = dict(bag=db.parent.name, vehicle=db.parent.name.split('_')[0], topic=key,
                           count=len(t), duration_s=float(receipt[-1]-receipt[0]),
                           header_start_s=t[0], header_end_s=t[-1],
                           header_rate_hz=float(1/np.median(dt[dt>0])) if np.any(dt>0) else 0.,
                           receipt_rate_hz=float((len(t)-1)/(receipt[-1]-receipt[0])) if receipt[-1]>receipt[0] else 0.,
                           header_backwards=int(np.sum(dt<0)), header_duplicates=int(np.sum(dt==0)),
                           lag_min_s=float(np.min(lag)), lag_median_s=float(np.median(lag)),lag_max_s=float(np.max(lag)),
                           max_gap_s=float(np.max(dt)) if len(dt) else 0.,
                           nonfinite=int(np.sum(~np.isfinite(val))), min_value=float(np.nanmin(val[:,0])),max_value=float(np.nanmax(val[:,0])))
                rows.append(row)
        if (idx+1)%20==0: print('Audited',idx+1,flush=True)
    with open(args.output/'dataset_audit.csv','w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    summary = {'bags': len({r['bag'] for r in rows}), 'messages':sum(r['count'] for r in rows),
               'topics':{k:{'count':sum(r['count'] for r in rows if r['topic']==k),
                            'bags_with_messages':sum(1 for r in rows if r['topic']==k),
                            'median_header_hz':float(np.median([r['header_rate_hz'] for r in rows if r['topic']==k])),
                            'header_backwards':sum(r['header_backwards'] for r in rows if r['topic']==k),
                            'median_receipt_minus_header_s':float(np.median([r['lag_median_s'] for r in rows if r['topic']==k]))}
                         for k in TOPICS.values()}}
    (args.output/'dataset_summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))

if __name__=='__main__': main()
