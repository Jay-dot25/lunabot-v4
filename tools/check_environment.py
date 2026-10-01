#!/usr/bin/env python3
"""Report LunaBot development/runtime capabilities without changing the host."""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def command_version(command: str, args: list[str]) -> dict:
    path = shutil.which(command)
    result = {"available": path is not None, "path": path, "version": None}
    if path:
        try:
            proc = subprocess.run(
                [path, *args], capture_output=True, text=True, timeout=10,
                check=False,
            )
            text = (proc.stdout or proc.stderr).strip().splitlines()
            result["version"] = text[0] if text else None
            result["version_exit_code"] = proc.returncode
        except (OSError, subprocess.TimeoutExpired) as exc:
            result["version_error"] = str(exc)
    return result


def inspect_environment() -> dict:
    ros_distro = os.environ.get("ROS_DISTRO")
    ros_setup = Path("/opt/ros/humble/setup.bash")
    tools = {
        "python": command_version("python3", ["--version"]),
        "git": command_version("git", ["--version"]),
        "bash": command_version("bash", ["--version"]),
        "ros2": command_version("ros2", ["--help"]),
        "colcon": command_version("colcon", ["version-check"]),
        "gazebo_gz": command_version("gz", ["--version"]),
        "gazebo_ign": command_version("ign", ["--version"]),
        "rviz2": command_version("rviz2", ["--help"]),
        "shellcheck": command_version("shellcheck", ["--version"]),
        "xmllint": command_version("xmllint", ["--version"]),
    }
    runtime_ready = bool(
        tools["ros2"]["available"]
        and (tools["gazebo_gz"]["available"] or tools["gazebo_ign"]["available"])
    )
    return {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "repository": str(ROOT),
        "platform": platform.platform(),
        "python_runtime": sys.version.splitlines()[0],
        "ros_distro_environment": ros_distro,
        "ros_humble_setup_exists": ros_setup.is_file(),
        "runtime_ready": runtime_ready,
        "tools": tools,
        "notes": (
            [] if runtime_ready else [
                "ROS 2/Gazebo runtime validation is unavailable in this environment; "
                "static validation remains available."
            ]
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="emit JSON")
    parser.add_argument("--output", type=Path, help="also write JSON to this path")
    args = parser.parse_args()
    report = inspect_environment()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print("LunaBot environment capability report")
        print(f"  Python: {report['python_runtime']}")
        for name, value in report["tools"].items():
            state = "available" if value["available"] else "missing"
            detail = f" ({value['path']})" if value["path"] else ""
            print(f"  {name}: {state}{detail}")
        print(f"  ROS/Gazebo runtime ready: {'yes' if report['runtime_ready'] else 'no'}")
        for note in report["notes"]:
            print(f"  NOTE: {note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
