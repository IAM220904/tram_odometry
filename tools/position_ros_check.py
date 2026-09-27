"""45-second held-out rosbag check of /result/position; no route/GNSS fitting."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from tram_vehicle_msgs.msg import VelocitySensor, DriverControllerCommand

ROOT=Path(__file__).resolve().parents[1]

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag',type=Path,default=ROOT/'dataset/data/30618_d4ba7005')
    ap.add_argument('--output',type=Path,default=ROOT/'results/position_eval')
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    splits=json.loads((ROOT/'results/splits.json').read_text())
    assert any(x['bag']==args.bag.name and x['split']=='test' for x in splits)
    os.environ.setdefault('ROS_LOG_DIR',str(args.output.resolve()/'ros_logs'))
    rows=[];diagnostics=[];counts={};processes=[];logs=[]
    started=datetime.now(timezone.utc).isoformat()
    rclpy.init();node=rclpy.create_node('position_output_check')
    def position(msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,msg.twist.twist.linear.x,*msg.pose.covariance,*msg.twist.covariance]
        rows.append(dict(receipt=time.monotonic(),stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            stamp_valid=msg.header.stamp.sec>0 and 0<=msg.header.stamp.nanosec<1000000000,
            frame_id=msg.header.frame_id,child_frame_id=msg.child_frame_id,x=p.x,y=p.y,z=p.z,
            velocity=msg.twist.twist.linear.x,finite=all(math.isfinite(x) for x in values),
            quaternion_norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)))
    def diag(msg):
        for status in msg.status:
            diagnostics.append(dict(message=status.message,values={x.key:x.value for x in status.values}))
    subs=[node.create_subscription(Odometry,'/result/position',position,qos_profile_sensor_data),
          node.create_subscription(DiagnosticArray,'/result/diagnostics',diag,qos_profile_sensor_data)]
    for suffix,cls in [('front_bogie_velocity',VelocitySensor),('rear_bogie_velocity',VelocitySensor),('driver_position_cmd',DriverControllerCommand)]:
        topic='/vehicle/'+suffix;counts[topic]=0
        def count(msg,t=topic):counts[t]+=1
        subs.append(node.create_subscription(cls,topic,count,qos_profile_sensor_data))
    commands=[['ros2','launch','tram_odometry','odometry.launch.py','use_sim_time:=true'],
              ['ros2','bag','play',str(args.bag.resolve()),'--clock','100','--rate','1.0']]
    def start(i):
        log=(args.output/('live_launch.log' if i==0 else 'live_bag.log')).open('w');logs.append(log)
        proc=subprocess.Popen(commands[i],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc)
    try:
        start(0);deadline=time.monotonic()+3
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        start(1);begin=time.monotonic();deadline=begin+45;alive=True
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            alive &= all(p.poll() is None for p in processes)
        elapsed=time.monotonic()-begin
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=3)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
    if len(rows)<2:raise RuntimeError('Too few position outputs; inspect logs')
    after=[r for r in rows if r['receipt']>=begin+5]
    pairs=list(zip(after,after[1:]))
    steps=[math.dist((a['x'],a['y'],a['z']),(b['x'],b['y'],b['z'])) for a,b in pairs]
    dt=[b['stamp']-a['stamp'] for a,b in pairs]
    moving=[i for i,(a,b) in enumerate(pairs) if min(a['velocity'],b['velocity'])>.2]
    checks=dict(published=True,valid_timestamps=all(r['stamp_valid'] for r in rows),
        increasing_timestamps=all(b['stamp']>a['stamp'] for a,b in zip(rows,rows[1:])),
        populated_frames=all(r['frame_id']=='odom' and r['child_frame_id']=='base_link' for r in rows),
        finite_pose_twist_covariance=all(r['finite'] for r in rows),
        normalized_orientation=all(abs(r['quaternion_norm']-1)<1e-6 for r in rows),
        motion_observed=bool(moving),position_changes_while_moving=bool(moving) and all(steps[i]>1e-6 for i in moving),
        no_large_post_warmup_jumps=all(s<=40*d+1 for s,d in zip(steps,dt)),
        no_post_warmup_timestamp_gap_over_250ms=max(dt)<=.25,
        node_and_player_alive=bool(alive),inputs_received=all(v>0 for v in counts.values()),
        gnss_initialization_locked=any(d['values'].get('gnss_locked')=='True' for d in diagnostics))
    result=dict(test_utc=started,bag=args.bag.name,split='test',commands=commands,duration_s=elapsed,
        output_count=len(rows),receipt_rate_hz=(len(rows)-1)/(rows[-1]['receipt']-rows[0]['receipt']),checks=checks,
        status='VERIFIED' if all(checks.values()) else 'FAILED',
        max_post_warmup_step_m=max(steps),max_post_warmup_header_dt_s=max(dt),
        moving_intervals=len(moving),first=rows[0],last=rows[-1],input_counts=counts,
        continuity_definition='After 5 s wall warmup: step <= 40 m/s * Header dt + 1 m tolerance for estimator correction; max dt <= 0.25 s; finite outputs. Not an accuracy claim.',
        process_exit_codes=[p.returncode for p in processes],scope='Short ROS output sanity check only; accuracy is full-bag offline evaluation.')
    (args.output/'live_ros_check.json').write_text(json.dumps(result,indent=2)+'\n')
    (args.output/'live_diagnostics.json').write_text(json.dumps(diagnostics,indent=2)+'\n')
    with (args.output/'live_position.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
