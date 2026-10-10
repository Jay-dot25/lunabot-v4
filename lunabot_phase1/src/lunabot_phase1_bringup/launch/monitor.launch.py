"""Launch only the diagnostics-only obstacle monitor for an existing /scan."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    parameters = PathJoinSubstitution([
        FindPackageShare("lunabot_phase1_bringup"),
        "config",
        "obstacle_monitor.yaml",
    ])
    return LaunchDescription([
        Node(
            package="lunabot_phase1_tools",
            executable="obstacle_monitor",
            name="obstacle_monitor",
            parameters=[parameters, {"use_sim_time": True}],
            output="screen",
        )
    ])
