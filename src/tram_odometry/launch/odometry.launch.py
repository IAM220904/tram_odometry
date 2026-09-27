from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('route_file',default_value=''),
        DeclareLaunchArgument('path_map_file',default_value=''),
        DeclareLaunchArgument('route_start_s',default_value='-1.0'),
        DeclareLaunchArgument('route_direction',default_value='1'),
        DeclareLaunchArgument('allow_initial_gnss',default_value='true'),
        Node(package='tram_odometry',executable='odometry_node',name='tram_odometry',output='screen',
             parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool),
                          'route_file':ParameterValue(LaunchConfiguration('route_file'),value_type=str),
                          'path_map_file':ParameterValue(LaunchConfiguration('path_map_file'),value_type=str),
                          'route_start_s':ParameterValue(LaunchConfiguration('route_start_s'),value_type=float),
                          'route_direction':ParameterValue(LaunchConfiguration('route_direction'),value_type=int),
                          'allow_initial_gnss':ParameterValue(LaunchConfiguration('allow_initial_gnss'),value_type=bool)}])])
