#!/usr/bin/env python3
"""Composable ROS 2 Python launch file for the full LunaBot V4 stack (Phase A-L)."""

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REPO_ROOT = Path(__file__).resolve().parent.parent


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    enable_rviz = LaunchConfiguration("enable_rviz", default="true")

    scripts_dir = REPO_ROOT / "scripts"
    rviz_config = REPO_ROOT / "rviz" / "phase_l.rviz"

    nodes = [
        ExecuteProcess(
            cmd=["python3", str(scripts_dir / "control_odometry.py"),
                 "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(scripts_dir / "odometry_monitor.py"),
                 "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(scripts_dir / "terrain_segmentation.py"),
                 "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(scripts_dir / "semantic_terrain_mapper.py"),
                 "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(scripts_dir / "obstacle_detector.py"),
                 "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(scripts_dir / "terrain_cost_mapper.py"),
                 "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(scripts_dir / "terrain_aware_planner.py"),
                 "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(scripts_dir / "terrain_path_follower.py"),
                 "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(scripts_dir / "dynamic_replan_monitor.py"),
                 "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(scripts_dir / "phase_k_evaluator.py"),
                 "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(scripts_dir / "phase_l_mission.py"),
                 "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="lunabot_rviz2",
            arguments=["-d", str(rviz_config)],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(enable_rviz),
            output="screen",
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        *nodes,
    ])
