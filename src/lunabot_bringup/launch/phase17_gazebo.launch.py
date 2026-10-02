"""Phase 17 stage 1: Gazebo world and rover only."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,SetEnvironmentVariable,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
 worlds=PathJoinSubstitution([FindPackageShare('lunabot_gazebo'),'worlds']);world=PathJoinSubstitution([worlds,'lunar_world.sdf']);model=PathJoinSubstitution([FindPackageShare('lunabot_gazebo'),'models','lunabot_v4','model.sdf'])
 gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'),'launch','gz_sim.launch.py'])),launch_arguments={'gz_args':['-r ',world]}.items())
 spawn=TimerAction(period=LaunchConfiguration('spawn_delay'),actions=[Node(package='ros_gz_sim',executable='create',arguments=['-file',model,'-name','lunabot_v4','-x','0','-y','0','-z','1.0'],output='screen')])
 return LaunchDescription([DeclareLaunchArgument('spawn_delay',default_value='3.0'),SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',worlds),SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',worlds),gazebo,spawn])
