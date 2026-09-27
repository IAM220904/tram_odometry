"""Short ROS/DDS observation; does not modify or instrument the estimator.

Run after sourcing Humble and this workspace. Outputs are overwritten on rerun.
Latency is a peer-observed proxy, not proof of estimator input acceptance.
"""
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shlex
import signal
import statistics
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from tram_vehicle_msgs.msg import VelocitySensor, DriverControllerCommand

ROOT = Path(__file__).resolve().parents[1]
INPUTS = ['/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity', '/vehicle/driver_position_cmd']
OUTPUTS = ['/result/velocity', '/result/position']


def stats(values):
    return dict(samples=len(values), mean=statistics.mean(values), median=statistics.median(values), max=max(values), min=min(values)) if values else dict(samples=0)


def proc_sample(pid):
    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    return {'monotonic': time.monotonic(), 'cpu_seconds': (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK'),
            'rss_mb': int(fields[21]) * os.sysconf('SC_PAGE_SIZE') / 1e6}


def find_node(parent):
    # Only descendants of the launch started by this script.
    descendants = {parent}
    for _ in range(5):
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields = path.read_text().rsplit(')', 1)[1].split()
                pid = int(path.parent.name)
                if int(fields[1]) in descendants:
                    descendants.add(pid)
                    cmd = (path.parent / 'cmdline').read_bytes().replace(b'\0', b' ')
                    if b'/lib/tram_odometry/odometry_node' in cmd:
                        return pid
            except (OSError, ValueError):
                pass
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', type=Path, default=ROOT / 'dataset/data/30618_af7496f0')
    ap.add_argument('--output', type=Path, default=ROOT / 'results/performance')
    args = ap.parse_args()
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_LOG_DIR', str(out / 'ros_logs'))
    launch_cmd = ['ros2', 'launch', 'tram_odometry', 'odometry.launch.py', 'use_sim_time:=true']
    bag_cmd = ['ros2', 'bag', 'play', str(args.bag.resolve()), '--clock', '100', '--rate', '1.0']
    started_utc = datetime.now(timezone.utc).isoformat()
    rclpy.init(); node = rclpy.create_node('performance_observer')
    records = defaultdict(list); resources = []; processes = []; logs = []
    def receive(topic, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        value = msg.velocity if isinstance(msg, VelocitySensor) else msg.position if isinstance(msg, DriverControllerCommand) else [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.twist.twist.linear.x]
        finite = all(math.isfinite(v) for v in (value if isinstance(value, list) else [value]))
        records[topic].append({'receipt': now, 'header': stamp, 'finite': finite})
    subscriptions = [node.create_subscription(DriverControllerCommand if t == INPUTS[2] else Odometry if t == OUTPUTS[1] else VelocitySensor,
                     t, lambda msg, topic=t: receive(topic, msg), qos_profile_sensor_data) for t in INPUTS + OUTPUTS]
    def start(name, cmd):
        log = (out / (name + '.log')).open('w'); logs.append(log)
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc); return proc
    report = None
    try:
        launch = start('launch', launch_cmd)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        pid = find_node(launch.pid)
        if pid is None:
            raise RuntimeError('Odometry process not found; inspect launch.log')
        player = start('bag', bag_cmd)
        begin = time.monotonic(); deadline = begin + 45; next_sample = begin
        alive = True
        print('Observing 45 seconds; publication/latency window is seconds 5..25.', flush=True)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if now >= next_sample:
                try:
                    resources.append(proc_sample(pid))
                except OSError:
                    alive = False
                alive &= launch.poll() is None and player.poll() is None
                next_sample = now + .5
        end = time.monotonic(); lo = begin + 5; hi = begin + 25
        rates = {}; proxies = {}
        for topic in OUTPUTS:
            window = [r for r in records[topic] if lo <= r['receipt'] < hi]
            intervals = [(b['receipt'] - a['receipt']) * 1000 for a, b in zip(window, window[1:])]
            all_gaps = [b['receipt'] - a['receipt'] for a, b in zip(records[topic], records[topic][1:])]
            bins = [sum(begin + i * 5 <= r['receipt'] < begin + (i + 1) * 5 for r in records[topic]) for i in range(9)]
            rates[topic] = {'count_20s': len(window), 'hz_count_over_window': len(window) / 20,
                'hz_interarrival': 1000 / statistics.mean(intervals) if intervals else None,
                'interval_ms': stats(intervals), 'count_entire_run': len(records[topic]), 'counts_per_5s': bins,
                'max_gap_entire_run_s': max(all_gaps) if all_gaps else None}
            proxies[topic] = {}
            pooled_latencies = []; pooled_headers = []; pooled_unmatched = 0
            for source in INPUTS:
                # Candidate: next observed output whose Header covers this input Header.
                # This does NOT prove that the estimator accepted/used that input.
                latencies = []; stamp_deltas = []; unmatched = 0
                for inp in records[source]:
                    if not lo <= inp['receipt'] < hi:
                        continue
                    candidate = next((r for r in records[topic] if r['receipt'] >= inp['receipt'] and r['header'] >= inp['header']), None)
                    if candidate is None:
                        unmatched += 1
                    else:
                        latencies.append((candidate['receipt'] - inp['receipt']) * 1000)
                        stamp_deltas.append((candidate['header'] - inp['header']) * 1000)
                proxies[topic][source] = {'delivery_latency_proxy_ms': stats(latencies), 'header_delta_ms': stats(stamp_deltas), 'unmatched': unmatched}
                pooled_latencies.extend(latencies); pooled_headers.extend(stamp_deltas); pooled_unmatched += unmatched
            proxies[topic]['all_inputs_pooled'] = {'delivery_latency_proxy_ms': stats(pooled_latencies), 'header_delta_ms': stats(pooled_headers), 'unmatched': pooled_unmatched}
        cpu = [100 * (b['cpu_seconds'] - a['cpu_seconds']) / (b['monotonic'] - a['monotonic']) for a, b in zip(resources, resources[1:])]
        cpu_mean = 100 * (resources[-1]['cpu_seconds'] - resources[0]['cpu_seconds']) / (resources[-1]['monotonic'] - resources[0]['monotonic'])
        rss = [r['rss_mb'] for r in resources]
        first_rss = statistics.mean(r['rss_mb'] for r in resources if r['monotonic'] < begin + 5)
        last_rss = statistics.mean(r['rss_mb'] for r in resources if r['monotonic'] >= begin + 40)
        stable = alive and all(all(v > 0 for v in rates[t]['counts_per_5s']) and all(r['finite'] for r in records[t]) for t in OUTPUTS)
        requirements = {
            'velocity_rate_ge_10_hz': 'VERIFIED' if rates[OUTPUTS[0]]['hz_count_over_window'] >= 10 else 'FAILED',
            'position_rate_ge_10_hz': 'VERIFIED' if rates[OUTPUTS[1]]['hz_count_over_window'] >= 10 else 'FAILED',
            'recommended_observed_rate_20_to_50_hz': 'VERIFIED' if all(20 <= rates[t]['hz_count_over_window'] <= 50 for t in OUTPUTS) else 'FAILED',
            'causal_input_to_output_latency_le_100_ms': 'NOT VERIFIED',
            'causal_peak_latency_le_250_ms': 'NOT VERIFIED',
            'node_cpu_le_2_cores_sampled': 'VERIFIED' if max(cpu) <= 200 else 'FAILED',
            'node_rss_le_0_5_gb_sampled': 'VERIFIED' if max(rss) <= 500 else 'FAILED',
            'short_run_alive_finite_continuous_outputs': 'VERIFIED' if stable else 'FAILED',
            'long_term_memory_stability': 'NOT VERIFIED',
        }
        report = {'test_utc': started_utc, 'bag': str(args.bag.resolve()), 'launch_command': shlex.join(launch_cmd),
            'bag_command': shlex.join(bag_cmd), 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
            'observation_seconds': end - begin, 'measurement_window_seconds': 20, 'warmup_seconds': 5,
            'rates': rates, 'latency_proxies': proxies,
            'latency_method': 'Observer monotonic receipt of each input to next received output with output Header >= input Header. All inputs, including potentially rejected ones, are included. Per-source values are reported separately.',
            'latency_limitations': 'No accepted-input ID or publication timestamp is exposed. DDS scheduling differs between observer and estimator; the candidate output may not include the input. Output Header is an extrapolated source clock, not wall publication time. Neither proxy nor Header delta proves causal latency compliance. No bag-recording age is treated as processing latency.',
            'resources': {'scope': 'Odometry node PID only, including its threads; excludes bag player, launch and observer. Decimal MB. /proc sampled about every 0.5 s; between-sample peaks unverified.',
                'pid': pid, 'cpu_percent_mean': cpu_mean, 'cpu_percent_sampled_peak': max(cpu), 'effective_cores_mean': cpu_mean / 100,
                'effective_cores_sampled_peak': max(cpu) / 100, 'rss_mb': stats(rss), 'first_5s_mean_rss_mb': first_rss,
                'last_5s_mean_rss_mb': last_rss, 'rss_growth_mb': last_rss - first_rss},
            'node_and_player_alive_entire_observation': bool(alive), 'requirements': requirements,
            'input_counts': {t: len(records[t]) for t in INPUTS}}
        (out / 'samples.json').write_text(json.dumps({'topic_receipts': dict(records), 'proc_samples': resources}, indent=2) + '\n')
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM); proc.wait(timeout=3)
        for log in logs:
            log.close()
        node.destroy_node(); rclpy.shutdown()
    report['process_exit_codes'] = [p.returncode for p in processes]
    build_log = ROOT / 'log/latest_build/logger_all.log'
    build_text = build_log.read_text() if build_log.exists() else ''
    build_ok = "run_until_complete finished with '0'" in build_text
    report['build'] = {'command': 'colcon build --symlink-install',
                       'evidence': str(build_log.resolve()), 'successful_log_found': build_ok,
                       'scope': 'Existing build log; this observer does not rebuild. Network isolation must be established by the build invocation.'}
    requirements['colcon_build_success'] = 'VERIFIED' if build_ok else 'NOT VERIFIED'
    (out / 'performance.json').write_text(json.dumps(report, indent=2) + '\n')
    lines = [f"Test UTC: {report['test_utc']}", f"Bag: {report['bag']}", f"Launch: {report['launch_command']}", f"Playback: {report['bag_command']}",
             f"Observation: {report['observation_seconds']:.3f} s; rates/proxies: 20 s after 5 s warmup."]
    for topic, value in rates.items():
        lines.append(f"{topic}: {value['hz_count_over_window']:.3f} Hz; mean-interval rate {value['hz_interarrival']:.3f} Hz; max interval {value['interval_ms']['max']:.3f} ms")
    for topic, sources in proxies.items():
        for source, value in sources.items():
            st = value['delivery_latency_proxy_ms']
            lines.append(f"PROXY {source} -> {topic}: {json.dumps(st)} ms; unmatched={value['unmatched']}")
    lines += [report['latency_method'], report['latency_limitations'], json.dumps(report['resources'], indent=2),
              'Short-run alive: ' + str(alive), 'Build: ' + json.dumps(report['build']), 'Requirements (scope: this short run only):', json.dumps(requirements, indent=2)]
    (out / 'performance.txt').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines), flush=True)


if __name__ == '__main__':
    main()
