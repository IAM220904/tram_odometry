"""Short real DEV-bag and synthetic ROS robustness checks, preserving the bag."""
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
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tram_vehicle_msgs.msg import VelocitySensor,DriverControllerCommand
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/robustness_eval'


def main():
    OUT.mkdir(parents=True,exist_ok=True);os.environ['ROS_LOG_DIR']=str(OUT/'ros_logs')
    scale=json.loads((ROOT/'src/tram_odometry/config/model.json').read_text())['wheel_scale']
    rclpy.init();node=rclpy.create_node('robustness_observer');phase='real_bag';data={};beg=0.
    def output(topic,msg):
        values=[msg.velocity] if topic=='velocity' else [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,msg.twist.twist.linear.x]
        data[phase][topic].append(dict(receipt=time.monotonic()-beg,stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,values=values,finite=all(math.isfinite(v) for v in values)))
    def diagnostic(msg):
        for status in msg.status:
            fields={v.key:v.value for v in status.values};finite=True
            for v in fields.values():
                try:finite &= math.isfinite(float(v))
                except ValueError:pass
            data[phase]['diagnostics'].append(dict(receipt=time.monotonic()-beg,message=status.message,fields=fields,finite=finite))
    subs=[node.create_subscription(VelocitySensor,'/result/velocity',lambda m:output('velocity',m),qos_profile_sensor_data),
          node.create_subscription(Odometry,'/result/position',lambda m:output('position',m),qos_profile_sensor_data),
          node.create_subscription(DiagnosticArray,'/result/diagnostics',diagnostic,qos_profile_sensor_data)]
    pubs={k:node.create_publisher(DriverControllerCommand if k=='command' else VelocitySensor,t,10) for k,t in [('front','/vehicle/front_bogie_velocity'),('rear','/vehicle/rear_bogie_velocity'),('command','/vehicle/driver_position_cmd')]}
    clock=node.create_publisher(Clock,'/clock',10)
    logs=[];processes=[];results={}
    def start(label,cmd):
        log=(OUT/(label+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    def stop():
        for p in reversed(processes):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=4)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
        codes=[p.returncode for p in processes];processes.clear()
        for f in logs:f.close()
        logs.clear();return codes
    def spin(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    try:
        for phase in ['real_bag','synthetic_ros']:
            data[phase]={'velocity':[],'position':[],'diagnostics':[]};beg=time.monotonic()
            cmd=['ros2','launch','tram_odometry','odometry.launch.py','use_sim_time:=true']
            if phase=='synthetic_ros':cmd+=['allow_initial_gnss:=false']
            launch=start(phase+'_launch',cmd);spin(3);beg=time.monotonic()
            if phase=='real_bag':
                start('real_bag_player',['ros2','bag','play',str(ROOT/'dataset/data/30618_af7496f0'),'--clock','100']);spin(20)
            else:
                for i in range(361):
                    t=i*.05;stamp=1000+t-(50 if t>=14 else 0);sec=int(stamp);ns=round((stamp-sec)*1e9)
                    c=Clock();c.clock.sec=sec;c.clock.nanosec=ns;clock.publish(c)
                    for kind,pub in pubs.items():
                        if kind!='command' and i%2:continue
                        if 7<=t<9:continue
                        msg=DriverControllerCommand() if kind=='command' else VelocitySensor()
                        msg.header.stamp.sec=sec;msg.header.stamp.nanosec=ns
                        if kind=='command':msg.position=0
                        else:msg.velocity=math.nan if 9<=t<11 else (13 if kind=='front' and 4<=t<7 else 8)/scale
                        pub.publish(msg)
                    deadline=beg+t+.05
                    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
                spin(.3)
            alive=all(p.poll() is None for p in processes);codes=stop()
            d=data[phase];diag=d['diagnostics']
            result=dict(outputs_velocity=len(d['velocity']),outputs_position=len(d['position']),diagnostics=len(diag),
                finite_outputs=all(r['finite'] for key in ['velocity','position'] for r in d[key]),finite_numeric_diagnostics=all(r['finite'] for r in diag),
                alive_at_end=alive,exit_codes=codes,
                max_velocity_receipt_gap_s=max((b['receipt']-a['receipt'] for a,b in zip(d['velocity'],d['velocity'][1:])),default=None))
            if phase=='synthetic_ros':
                result.update(slip_flag_observed=any(r['fields'].get('slip') not in (None,'[]') for r in diag),
                    model_only_observed=any(r['message'].split(';',1)[0].strip()=='model_only' for r in diag),
                    command_stale_observed=any(r['fields'].get('command_stale')=='True' for r in diag),
                    invalid_input_count_observed=max((int(r['fields'].get('invalid_inputs',0)) for r in diag),default=0),
                    diagnostics_after_clock_reset=any(r['receipt']>=14.5 and float(r['fields']['t'])<1000 for r in diag),
                    recovered_status_before_reset=any(12<=r['receipt']<14 and r['message'].split(';',1)[0].strip()=='ok' and r['fields'].get('slip')=='[]' for r in diag),
                    final_status=diag[-1]['message'].split(';',1)[0].strip() if diag else None)
            results[phase]=result
    finally:
        stop();node.destroy_node();rclpy.shutdown()
    report=dict(test_utc=datetime.now(timezone.utc).isoformat(),real_bag='30618_af7496f0',split='dev',results=results,
        synthetic_schedule='0..4 normal 8m/s; 4..7 front +5m/s; 7..9 all vehicle inputs missing; 9..11 wheel NaN; 11..18 normal. At 14s simulation clock rolls back 50s.',
        limitations='Synthetic direct publishers, not a modified bag. No forward Header spike or prolonged simultaneous slip injected into ROS; those are tested in the core harness. No labeled real slip validation.')
    (OUT/'runtime_robustness.json').write_text(json.dumps(report,indent=2)+'\n')
    (OUT/'runtime_samples.json').write_text(json.dumps(data,indent=2)+'\n')
    (OUT/'diagnostics_sample.txt').write_text('\n'.join(json.dumps(r) for p in data.values() for r in p['diagnostics'])+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
