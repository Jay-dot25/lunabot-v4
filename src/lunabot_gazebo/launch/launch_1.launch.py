"""Single entry point for the LunaBot Phase 1 simulation."""
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _gazebo(context):
    world = PathJoinSubstitution([
        FindPackageShare('lunabot_gazebo'), 'worlds', 'lunar_world.sdf'
    ]).perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')
    command = ['ign', 'gazebo', '-r']
    if headless:
        command.append('-s')
    command.append(world)
    gazebo = ExecuteProcess(cmd=command, output='screen', sigterm_timeout='10')
    stop_launch_if_gazebo_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))],
        )
    )
    return [gazebo, stop_launch_if_gazebo_exits]


def generate_launch_description():
    models_dir = PathJoinSubstitution([
        FindPackageShare('lunabot_gazebo'), 'models'
    ])
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/lunabot/gz_cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
    ]
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo server only when true.'),
        DeclareLaunchArgument('obstacle_threshold', default_value='1.5',
                              description='Forward obstacle threshold in metres.'),
        DeclareLaunchArgument('obstacle_forward_half_angle_deg', default_value='15.0',
                              description='Half-width of the front scan sector in degrees.'),
        DeclareLaunchArgument('obstacle_stale_timeout_sec', default_value='0.75',
                              description='Sensor stale timeout in seconds.'),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models_dir),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_dir),
        OpaqueFunction(function=_gazebo),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='lunabot_gz_bridge', arguments=bridge_args, output='screen',
             parameters=[{'use_sim_time': True}], respawn=True, respawn_delay=2.0),
        Node(package='lunabot_tools', executable='command_gate',
             name='lunabot_command_gate', output='screen', respawn=True,
             respawn_delay=2.0),
        Node(package='lunabot_tools', executable='obstacle_monitor',
             name='lunabot_obstacle_monitor', output='screen', respawn=True,
             respawn_delay=2.0, parameters=[
                 {'use_sim_time': True},
                 {'threshold': ParameterValue(LaunchConfiguration('obstacle_threshold'), value_type=float)},
                 {'forward_half_angle_deg': ParameterValue(LaunchConfiguration('obstacle_forward_half_angle_deg'), value_type=float)},
                 {'stale_timeout_sec': ParameterValue(LaunchConfiguration('obstacle_stale_timeout_sec'), value_type=float)},
             ]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='camera_link_tf', output='screen',
             arguments=['--x', '0.28', '--y', '0', '--z', '0.32', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='lidar_link_tf', output='screen',
             arguments=['--x', '0', '--y', '0', '--z', '0.34', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'lidar_link']),
    ])
