"""Launch an isolated Phase 2 terrain-capture world, bridges and safe teleop guard.

This does not alter or start the standalone Phase 1 launch and does not start
model inference. Use scene01, scene02, or scene03 for distinct capture layouts.
"""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SCENES = {"scene01", "scene02", "scene03"}


def _static_transform(name: str, parent: str, child: str, xyz: tuple[float, float, float],
                      rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Node:
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


def _setup_capture(context):
    scene = LaunchConfiguration("scene").perform(context).strip().lower()
    if scene not in SCENES:
        raise RuntimeError(f"scene must be one of {', '.join(sorted(SCENES))}; got {scene!r}")

    phase2_share = Path(get_package_share_directory("lunabot_phase2_perception"))
    bringup_share = Path(get_package_share_directory("lunabot_phase1_bringup"))
    description_share = Path(get_package_share_directory("lunabot_phase1_description"))
    world = phase2_share / "worlds" / f"eyes_of_luna_capture_{scene}.sdf"
    if not world.is_file():
        raise RuntimeError(f"capture world is missing from the install: {world}")

    model_paths = [
        str(description_share / "models"),
        str(phase2_share / "models"),
    ]
    previous_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    if previous_path:
        model_paths.append(previous_path)
    resource_path = os.pathsep.join(model_paths)

    bridge_config = bringup_share / "config" / "bridge.yaml"
    camera_bridge_config = bringup_share / "config" / "camera_bridge.yaml"
    guard_config = bringup_share / "config" / "command_guard.yaml"

    return [
        SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=resource_path),
        ExecuteProcess(
            cmd=["ign", "gazebo", "-r", "-v", "3", str(world)],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="ros_ign_bridge",
            executable="parameter_bridge",
            name="eyes_of_luna_capture_bridge",
            parameters=[{"config_file": str(bridge_config)}],
            output="screen",
        ),
        Node(
            package="ros_ign_bridge",
            executable="parameter_bridge",
            name="eyes_of_luna_capture_camera_bridge",
            parameters=[
                {"config_file": str(camera_bridge_config)},
                {"override_frame_id": "camera_optical_frame"},
            ],
            output="screen",
        ),
        Node(
            package="lunabot_phase1_tools",
            executable="command_guard",
            name="eyes_of_luna_capture_command_guard",
            parameters=[str(guard_config), {"use_sim_time": False}],
            output="screen",
        ),
        _static_transform(
            "capture_base_footprint_to_base_link", "base_footprint", "base_link",
            (0.0, 0.0, 0.28),
        ),
        _static_transform(
            "capture_base_link_to_camera", "base_link", "camera_link",
            (0.43, 0.0, 0.28),
        ),
        _static_transform(
            "capture_camera_to_optical", "camera_link", "camera_optical_frame",
            (0.0, 0.0, 0.0), (-1.57079632679, 0.0, -1.57079632679),
        ),
        _static_transform(
            "capture_base_link_to_lidar", "base_link", "lidar_link",
            (0.50, 0.0, 0.20),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene", default_value="scene01",
            description="Independent simulated terrain layout: scene01, scene02, or scene03",
        ),
        OpaqueFunction(function=_setup_capture),
    ])
