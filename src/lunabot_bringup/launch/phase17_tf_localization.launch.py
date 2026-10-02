"""Phase 17 stage 3: bridges plus an explicit connected TF/localization tree."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def static_tf(name, parent, child, xyz, rpy=(0.0, 0.0, 0.0)):
    x, y, z = xyz
    roll, pitch, yaw = rpy
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                   '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                   '--frame-id', parent, '--child-frame-id', child],
        output='screen')


def generate_launch_description():
    bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('lunabot_bringup'), 'launch',
            'phase17_bridges.launch.py'])))

    # Gazebo sensor frame names are scoped. These transforms represent the
    # neutral mast pose defined by model.sdf; mast actuation is not used here.
    transforms = [
        static_tf('map_to_odom', 'map', 'odom', (0.0, 0.0, 0.0)),
        static_tf('chassis_to_rgb', 'chassis',
                  'lunabot_v4/sensor_head/rgb_camera', (0.32, 0.0, 0.85)),
        static_tf('chassis_to_depth', 'chassis',
                  'lunabot_v4/sensor_head/depth_camera', (0.32, 0.0, 0.81)),
        static_tf('chassis_to_lidar', 'chassis',
                  'lunabot_v4/sensor_head/lidar', (0.20, 0.0, 0.94),
                  (0.0, 0.5, 0.0)),
        static_tf('chassis_to_imu', 'chassis',
                  'lunabot_v4/imu_link/imu', (0.0, 0.0, 0.10)),
    ]

    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('lunabot_localization'),
                                  'config', 'ekf.yaml']),
            {'odom0': '/lunabot/odom', 'publish_tf': True,
             'use_sim_time': True},
        ])
    return LaunchDescription([bridges, *transforms, ekf])
