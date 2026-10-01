"""Phase 2 package-foundation smoke launch; it starts no rover processes."""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        LogInfo(msg="LunaBot ROS 2 package foundation is discoverable."),
    ])
