#!/bin/bash
# Collect real Phase B evidence on a workstation with ROS 2 + Gazebo.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
EVIDENCE=1 DEMO=1 HEADLESS=1 bash "$REPO_DIR/scripts/launch-b.sh"
rc=$?
echo "Phase B evidence collector exit code: $rc"
exit "$rc"
