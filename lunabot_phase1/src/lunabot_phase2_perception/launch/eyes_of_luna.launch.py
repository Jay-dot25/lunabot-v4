"""Start Phase 2 perception against the already-running Phase 1 camera."""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_ml_path() -> str:
    for parent in Path(get_package_share_directory("lunabot_phase2_perception")).resolve().parents:
        candidate = parent / "ml"
        if (candidate / "lunabot_ml").is_dir():
            return str(candidate)
    return os.environ.get("LUNABOT_ML_PATH", "")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_bundle_path",
            default_value=os.environ.get("EYES_OF_LUNA_BUNDLE", ""),
            description="Path to training output deployment.json; required for inference.",
        ),
        DeclareLaunchArgument(
            "ml_path",
            default_value=_find_ml_path(),
            description="Path to the repository ml directory containing lunabot_ml.",
        ),
        DeclareLaunchArgument("device", default_value="auto"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
    Node(
        package="lunabot_phase2_perception",
        executable="eyes_of_luna_node",
        name="eyes_of_luna_perception",
        output="screen",
        parameters=[{
            "model_bundle_path": LaunchConfiguration("model_bundle_path"),
            "ml_path": LaunchConfiguration("ml_path"),
            "device": LaunchConfiguration("device"),
            "image_topic": LaunchConfiguration("image_topic"),
        }],
    ),
    ])
