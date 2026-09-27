"""Synthetic faults on an independent analytic speed trajectory, not slip labels."""
import json
import math
from pathlib import Path
import sys
import numpy as np
from evaluate import replay
from tram_odometry.core import Config


def main():
    root=Path(__file__).resolve().parents[1]
    c=Config.from_dict(json.loads((root/'src/tram_odometry/config/model.json').read_text()))
    rng=np.random.default_rng(20260926)
    t=np.arange(0,100,.1);u_t=np.arange(0,100,.05)
    velocity=lambda t:8+2*np.sin(t*.1)
    a=.2*np.cos(u_t*.1);v=velocity(u_t)
    command=np.where(a>=0,(a+.05)*(1+v/12)/1.8*15,(a+.05)/1.87*15)
    command=np.rint(command).clip(-15,15)
    data={'front':np.column_stack([t,t+.025,(velocity(t)+rng.normal(0,.015,len(t)))/c.wheel_scale]),
          'rear':np.column_stack([t,t+.035,(velocity(t)+rng.normal(0,.015,len(t)))/c.wheel_scale]),
          'command':np.column_stack([u_t,u_t+.001,command])}
    scenarios={
        'clean':lambda k,t,v:v,
        'front_slip_5mps_5s':lambda k,t,v:v+5/c.wheel_scale if k=='front' and 30<=t<35 else v,
        'both_slip_5mps_2s':lambda k,t,v:v+5/c.wheel_scale if k!='command' and 30<=t<32 else v,
        'both_locked_1s':lambda k,t,v:0. if k!='command' and 30<=t<31 else v,
        'both_missing_2s':lambda k,t,v:None if k!='command' and 30<=t<32 else v,
        'rear_missing_10s':lambda k,t,v:None if k=='rear' and 30<=t<40 else v,
        'nan_and_inf':lambda k,t,v:float('nan') if k=='front' and 30<=t<32 else float('inf') if k=='rear' and 30<=t<32 else v,
    }
    results=[]
    for name,fault in scenarios.items():
        for base in [False,True]:
            out,lat,status=replay(data,c,base,fault)
            e=out[:,1]-velocity(out[:,0]);mask=(out[:,0]>=30)&(out[:,0]<45)
            results.append(dict(scenario=name,method='wheel_mean' if base else 'model_observer',
                                rmse_mps=float(np.sqrt(np.mean(e[mask]**2))),max_error_mps=float(np.max(abs(e[mask]))),
                                finite=bool(np.isfinite(out).all()),rejected=status.get('rejected',0)))
    path=root/'results/stress_tests.json';path.write_text(json.dumps(results,indent=2))
    print(json.dumps(results,indent=2))
    if not all(r['finite'] for r in results):raise SystemExit('Nonfinite result')

if __name__=='__main__':main()
