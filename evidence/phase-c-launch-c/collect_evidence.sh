#!/usr/bin/env bash
# Run the complete Phase C runtime/evidence gate from any working directory.
set -euo pipefail
SCRIPT="$(readlink -f "${BASH_SOURCE[0]}")"
ROOT="$(cd "$(dirname "$SCRIPT")/../.." && pwd)"

cd "$ROOT"
source /opt/ros/humble/setup.bash
HEADLESS="${HEADLESS:-1}" DEMO=1 EVIDENCE=1 "$ROOT/scripts/launch-c.sh"
