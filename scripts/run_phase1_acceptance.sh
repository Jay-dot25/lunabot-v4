#!/usr/bin/env bash
# ============================================================
# LunaBot - Phase 1 - Base Camp acceptance test
#
# Prerequisite: the simulation must already be running, e.g.
#     scripts/run_phase1_sim.sh --no-teleop
#
# Runs the automated Phase 1 pass-gate checks and writes evidence
# to evidence/phase1/. All extra arguments are forwarded to the
# test, e.g. --timeout 90, --output-dir my/dir
# ============================================================

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

ROS_DISTRO="${ROS_DISTRO:-humble}"
TEST_PATH="$REPO_DIR/src/lunabot_tests/lunabot_tests/phase1_base_camp.py"

if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    echo "ERROR: /opt/ros/$ROS_DISTRO/setup.bash not found."
    exit 1
fi

if [ -f "$REPO_DIR/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$REPO_DIR/install/setup.bash"
fi

echo "============================================================"
echo " LunaBot - Phase 1 - Base Camp acceptance test"
echo "============================================================"
echo ""

python3 "$TEST_PATH" --output-dir "$REPO_DIR/evidence/phase1" "$@"
RESULT=$?

echo ""
if [ $RESULT -eq 0 ]; then
    echo "RESULT: PASS - evidence written to evidence/phase1/"
else
    echo "RESULT: FAIL - see evidence/phase1/ for details"
fi
exit $RESULT
