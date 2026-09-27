import json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import qos_profile_sensor_data
root=Path('/home/iam220904/rezerv_odometr/tram_odometry_solution');out=root/'results/route_eval/audit'
os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
rclpy.init();node=rclpy.create_node('frozen_route_check');poses=[];status=[];processes=[];logs=[]
node.create_subscription(Odometry,'/result/position',lambda m:poses.append([m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]),qos_profile_sensor_data)
node.create_subscription(DiagnosticArray,'/result/diagnostics',lambda m:status.extend(s.message for s in m.status),qos_profile_sensor_data)
commands=[['ros2','launch','tram_odometry','odometry.launch.py','use_sim_time:=true','path_map_file:='+str(root/'src/tram_odometry/config/train_path_map.json')],['ros2','bag','play',str(root/'dataset/data/30618_af7496f0'),'--clock','100']]
try:
    for i,cmd in enumerate(commands):
        log=(out/('runtime_launch.log' if i==0 else 'runtime_bag.log')).open('w');logs.append(log)
        processes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        if i==0:
            until=time.monotonic()+3
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
    begin=time.monotonic()
    while time.monotonic()-begin<20:rclpy.spin_once(node,timeout_sec=.05)
    import math
    result=dict(bag='30618_af7496f0',split='dev',seconds=time.monotonic()-begin,commands=commands,position_messages=len(poses),finite=all(math.isfinite(v) for p in poses for v in p),path_map_diagnostics=sum('path_map' in s for s in status),last_status=status[-1] if status else None,alive=all(p.poll() is None for p in processes))
finally:
    for p in reversed(processes):
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
    for f in logs:f.close()
    node.destroy_node();rclpy.shutdown()
result['exit_codes']=[p.returncode for p in processes]
(out/'runtime_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
