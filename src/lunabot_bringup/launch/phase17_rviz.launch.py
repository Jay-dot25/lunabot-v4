"""Phase 17 stage 4: validated TF/localization plus minimal RViz."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tf_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('lunabot_bringup'), 'launch',
            'phase17_tf_localization.launch.py'])))
    rviz = Node(
        package='rviz2', executable='rviz2', name='phase17_rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('lunabot_bringup'), 'rviz',
            'phase17_minimal.rviz'])],
        parameters=[{'use_sim_time': True}], output='screen')
    # Let Gazebo spawn the rover and establish simulated time / TF before RViz
    # creates message filters. This avoids caching pre-spawn sensor timestamps.
    delayed_rviz = TimerAction(period=8.0, actions=[rviz])
    return LaunchDescription([tf_localization, delayed_rviz])
