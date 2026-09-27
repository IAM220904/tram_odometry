"""ROS 2 Humble adapter. ROS integration must be verified on the target host."""
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from tram_vehicle_msgs.msg import VelocitySensor, DriverControllerCommand
from .core import Config, Estimator, SourceClock
from .position import PathMap, PositionTracker, Route


class OdometryNode(Node):
    def __init__(self):
        super().__init__('tram_odometry')
        default_model=str(Path(get_package_share_directory('tram_odometry'))/'config/model.json')
        defaults={'model_file':default_model,'wheel_scale':0.,'publish_hz':20.,'frame_id':'odom',
                  'child_frame_id':'base_link','allow_initial_gnss':True,'initialization_seconds':3.,
                  'initial_yaw':0.,'initial_x':0.,'initial_y':0.,'initial_z':0.,
                  'configured_origin':False,'origin_latitude':0.,'origin_longitude':0.,'origin_altitude':0.,
                  'route_file':'','path_map_file':'','route_start_s':-1.,'route_direction':1,'dual_gnss_heading':True}
        for name,value in defaults.items():self.declare_parameter(name,value)
        self.settings={name:self.get_parameter(name).value for name in defaults}
        self.config=Config.from_dict(json.loads(Path(self.settings['model_file']).read_text()))
        if self.settings['wheel_scale']>0:
            data=dict(vars(self.config));data['wheel_scale']=self.settings['wheel_scale'];self.config=Config.from_dict(data)
        if not 10<=self.settings['publish_hz']<=100:raise ValueError('publish_hz must be 10..100')
        self.route=Route.load(self.settings['route_file']) if self.settings['route_file'] else None
        self.path_map=PathMap.load(self.settings['path_map_file']) if self.settings['path_map_file'] else None
        self.last_arrival=None;self.last_published=None;self.last_diagnostic=-math.inf
        self.compute_max_us=0.;self.reset_state()
        qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        self.velocity_pub=self.create_publisher(VelocitySensor,'/result/velocity',10)
        self.position_pub=self.create_publisher(Odometry,'/result/position',10)
        self.diagnostic_pub=self.create_publisher(DiagnosticArray,'/result/diagnostics',10)
        self.subscriptions_keep=[]
        for kind,name in [('front','front_bogie_velocity'),('rear','rear_bogie_velocity'),('command','driver_position_cmd')]:
            cls=DriverControllerCommand if kind=='command' else VelocitySensor
            self.subscriptions_keep.append(self.create_subscription(cls,'/vehicle/'+name,lambda msg,k=kind:self.input_callback(k,msg),qos))
        if self.settings['allow_initial_gnss']:
            for receiver in ['master','rover']:
                self.subscriptions_keep.append(self.create_subscription(NavSatFix,f'/sensing/gnss/{receiver}/fix',lambda msg,k=receiver+'_fix':self.gnss_callback(k,msg),qos))
                self.subscriptions_keep.append(self.create_subscription(TwistStamped,f'/sensing/gnss/{receiver}/vel',lambda msg,k=receiver+'_vel':self.gnss_callback(k,msg),qos))
        # A steady timer still runs during bag pause; arrival_clock below freezes state.
        self.steady_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.timer=self.create_timer(1./self.settings['publish_hz'],self.publish_tick,clock=self.steady_clock)
        self.get_logger().info(f'Wheel scale={self.config.wheel_scale:.9f}; GNSS init={self.settings["initialization_seconds"]} s; route={bool(self.route)}; path_map={bool(self.path_map)}')
        if self.route is None and self.path_map is None:self.get_logger().warning('No route: direction held after initialization; turns are unobservable from scalar inputs.')

    def reset_state(self):
        self.estimator=Estimator(self.config);self.source_clock=SourceClock()
        # A previous episode's clock must not suppress diagnostics after rewind.
        self.last_diagnostic=-math.inf;self.invalid_inputs=0
        s=self.settings;origin=(s['origin_latitude'],s['origin_longitude'],s['origin_altitude']) if s['configured_origin'] else None
        self.position=PositionTracker(init_seconds=s['initialization_seconds'],allow_gnss=s['allow_initial_gnss'],
             initial_yaw=s['initial_yaw'],initial_xyz=(s['initial_x'],s['initial_y'],s['initial_z']),origin=origin,
             route=self.route,route_s=None if s['route_start_s']<0 else s['route_start_s'],route_direction=s['route_direction'],dual_heading=s['dual_gnss_heading'],path_map=self.path_map)
        self.last_published=None

    def arrival_clock(self):
        now=self.get_clock().now().nanoseconds*1e-9 if self.get_parameter('use_sim_time').value else time.monotonic()
        if self.last_arrival is not None and now<self.last_arrival-.5:
            self.reset_state();self.get_logger().warning('Clock moved backwards: starting new odometry episode')
        self.last_arrival=now
        return now

    @staticmethod
    def header_time(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def input_callback(self,kind,msg):
        arrival=self.arrival_clock();t=self.header_time(msg)
        value=float(msg.position if kind=='command' else msg.velocity)
        if t<=0 or not math.isfinite(value):
            self.invalid_inputs+=1
            return
        started=time.perf_counter()
        self.estimator.feed(kind,t,value)
        if self.estimator.last_seen.get(kind)==t:
            self.source_clock.observe(t,arrival);self.position.begin(self.estimator.t)
        self.compute_max_us=max(self.compute_max_us,(time.perf_counter()-started)*1e6)

    def gnss_callback(self,kind,msg):
        if self.estimator.t is None or self.position.locked:return
        if kind.endswith('_fix'):values=(msg.latitude,msg.longitude,msg.altitude,float(msg.status.status))
        else:values=(msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z)
        self.position.feed_gnss(kind,self.header_time(msg),values,self.estimator.snapshot())

    @staticmethod
    def set_stamp(header,t):
        ns=round(t*1e9);header.stamp.sec=ns//1000000000;header.stamp.nanosec=ns%1000000000

    def publish_tick(self):
        arrival=self.arrival_clock();t=self.source_clock.now(arrival)
        if t is None or not self.estimator.initialized:return
        t=max(t,self.estimator.t)
        if self.last_published is not None and t<=self.last_published+1e-6:return
        self.estimator.predict(t);state=self.estimator.snapshot();pose=self.position.pose(state)
        velocity=VelocitySensor();self.set_stamp(velocity.header,t)
        velocity.header.frame_id=self.settings['child_frame_id'];velocity.velocity=float(state['velocity'])
        odom=Odometry();self.set_stamp(odom.header,t);odom.header.frame_id=self.settings['frame_id'];odom.child_frame_id=self.settings['child_frame_id']
        odom.pose.pose.position.x=pose['x'];odom.pose.pose.position.y=pose['y'];odom.pose.pose.position.z=pose['z']
        odom.pose.pose.orientation.z=math.sin(pose['yaw']/2);odom.pose.pose.orientation.w=math.cos(pose['yaw']/2)
        odom.twist.twist.linear.x=float(state['velocity'])
        c=math.cos(pose['yaw']);s=math.sin(pose['yaw']);along=pose['longitudinal_variance'];cross=pose['cross_variance']
        odom.pose.covariance[0]=c*c*along+s*s*cross;odom.pose.covariance[7]=s*s*along+c*c*cross
        odom.pose.covariance[1]=odom.pose.covariance[6]=c*s*(along-cross)
        odom.pose.covariance[14]=1e6;odom.pose.covariance[21]=odom.pose.covariance[28]=1e6;odom.pose.covariance[35]=pose['yaw_variance']
        odom.twist.covariance[0]=float(state['velocity_variance'])
        for i in [7,14,21,28,35]:odom.twist.covariance[i]=1e6
        self.velocity_pub.publish(velocity);self.position_pub.publish(odom);self.last_published=t
        if arrival-self.last_diagnostic>=1.:
            self.last_diagnostic=arrival
            diag=DiagnosticArray();diag.header=odom.header
            item=DiagnosticStatus();item.name='tram_odometry';item.hardware_id='tram'
            item.level=DiagnosticStatus.OK if state['status']=='ok' and pose['status'] in ('known_route','path_map') else DiagnosticStatus.WARN
            item.message=state['status']+'; '+pose['status']
            fields={**state,**pose,'compute_max_us':self.compute_max_us,'input_age_s':t-max(self.estimator.last_seen.values()),
                    'invalid_inputs':self.invalid_inputs,'command_stale':t-self.estimator.command_t>self.config.command_timeout}
            item.values=[KeyValue(key=str(k),value=str(v)) for k,v in fields.items()]
            diag.status=[item];self.diagnostic_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=OdometryNode();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
