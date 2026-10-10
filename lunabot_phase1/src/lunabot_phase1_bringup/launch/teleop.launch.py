"""Optional keyboard teleop launch; run from a terminal with a real TTY."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lunabot_phase1_tools",
            executable="keyboard_teleop",
            name="keyboard_teleop",
            emulate_tty=True,
            output="screen",
        )
    ])
