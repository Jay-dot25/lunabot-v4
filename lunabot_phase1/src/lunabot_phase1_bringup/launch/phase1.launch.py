"""Launch the Lunar Base Camp world, rover, bridges, TF and Phase 1 tools."""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _static_transform(name, parent, child, xyz, rpy=(0.0, 0.0, 0.0)):
    x, y, z = xyz
    roll, pitch, yaw = rpy
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(x), "--y", str(y), "--z", str(z),
            "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
            "--frame-id", parent, "--child-frame-id", child,
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare("lunabot_phase1_simulation"),
        "worlds",
        "lunar_base_camp.sdf",
    ])
    model_resources = PathJoinSubstitution([
        FindPackageShare("lunabot_phase1_description"), "models"
    ])
    bridge_config = PathJoinSubstitution([
        FindPackageShare("lunabot_phase1_bringup"), "config", "bridge.yaml"
    ])
    camera_bridge_config = PathJoinSubstitution([
        FindPackageShare("lunabot_phase1_bringup"), "config", "camera_bridge.yaml"
    ])
    lidar_bridge_config = PathJoinSubstitution([
        FindPackageShare("lunabot_phase1_bringup"), "config", "lidar_bridge.yaml"
    ])
    guard_config = PathJoinSubstitution([
        FindPackageShare("lunabot_phase1_bringup"), "config", "command_guard.yaml"
    ])
    monitor_config = PathJoinSubstitution([
        FindPackageShare("lunabot_phase1_bringup"), "config", "obstacle_monitor.yaml"
    ])

    # Fortress resolves the world's model:// include through this resource path.
    resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            model_resources,
            os.pathsep,
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "3", world],
        output="screen",
        emulate_tty=True,
    )

    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="lunabot_phase1_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )
    camera_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="lunabot_phase1_camera_bridge",
        parameters=[{
            "config_file": camera_bridge_config,
            "override_frame_id": "camera_optical_frame",
        }],
        output="screen",
    )
    lidar_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="lunabot_phase1_lidar_bridge",
        parameters=[{
            "config_file": lidar_bridge_config,
            "override_frame_id": "lidar_link",
        }],
        output="screen",
    )
    command_guard = Node(
        package="lunabot_phase1_tools",
        executable="command_guard",
        name="command_guard",
        parameters=[guard_config, {"use_sim_time": False}],
        output="screen",
    )
    obstacle_monitor = Node(
        package="lunabot_phase1_tools",
        executable="obstacle_monitor",
        name="obstacle_monitor",
        parameters=[monitor_config, {"use_sim_time": True}],
        output="screen",
    )

    # Exactly one dynamic odom -> base_footprint transform comes from DiffDrive.
    # These static transforms are the rigid sensor/chassis extrinsics.
    transforms = [
        _static_transform(
            "base_footprint_to_base_link", "base_footprint", "base_link",
            (0.0, 0.0, 0.28),
        ),
        _static_transform(
            "base_link_to_camera", "base_link", "camera_link",
            (0.43, 0.0, 0.28),
        ),
        _static_transform(
            "camera_to_optical", "camera_link", "camera_optical_frame",
            (0.0, 0.0, 0.0), (-1.57079632679, 0.0, -1.57079632679),
        ),
        _static_transform(
            "base_link_to_lidar", "base_link", "lidar_link",
            (0.06, 0.0, 0.20),
        ),
    ]

    return LaunchDescription([
        resource_path,
        gazebo,
        bridge,
        camera_bridge,
        lidar_bridge,
        command_guard,
        obstacle_monitor,
        *transforms,
    ])
