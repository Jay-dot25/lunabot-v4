#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
if [[ ${1:-} == --execute ]]; then
  shift
  exec python3 tools/run_experiments.py config/experiments.json --output "${EXPERIMENT_OUTPUT:-evidence/experiments}" --execute --resume "$@"
fi
exec python3 tools/run_experiments.py config/experiments.json --output "${EXPERIMENT_OUTPUT:-evidence/experiments}" "$@"
