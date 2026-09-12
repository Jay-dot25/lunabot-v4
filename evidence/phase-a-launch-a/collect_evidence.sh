#!/bin/bash
# ============================================================
#  LunaBot V4 - Phase A - runtime evidence collector
#  Run on the workstation with ROS 2 Humble + Gazebo Sim:
#
#      bash evidence/phase-a-launch-a/collect_evidence.sh
#
#  Starts the full Phase A stack headless in DEMO+EVIDENCE mode
#  (automated forward + turn drive test), records all runtime
#  evidence into evidence/phase-a-launch-a/, and shuts down
#  cleanly. Exits with the launch's overall result code.
# ============================================================
set -u
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO_DIR"

echo "Collecting Phase A runtime evidence..."
echo "  (Gazebo + bridge + demo drive + evidence capture)"
echo ""
EVIDENCE=1 DEMO=1 HEADLESS=1 bash scripts/launch-a.sh
rc=$?
echo ""
echo "Evidence directory: $REPO_DIR/evidence/phase-a-launch-a"
exit $rc
