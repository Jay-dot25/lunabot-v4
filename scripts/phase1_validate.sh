#!/usr/bin/env bash
# Phase 1 validation. Modes: static (offline), build, runtime, all.
set -uo pipefail
REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
MODE="${1:-static}"
BLOCKED=0
FAIL=0

blocked() { echo "BLOCKED: $*"; BLOCKED=1; }
fail() { echo "FAIL: $*"; FAIL=1; }
pass() { echo "PASS: $*"; }

run_static() {
  echo '== Static and offline tests =='
  if python3 - <<'PY'
import ast, pathlib, xml.etree.ElementTree as ET
root = pathlib.Path('.')
xml_files = list(root.glob('src/**/*.xml')) + list(root.glob('src/**/*.sdf'))
for path in xml_files:
    ET.parse(path)
    print(f'XML PASS {path}')
for path in root.glob('src/**/*.py'):
    ast.parse(path.read_text(), filename=str(path))
    print(f'PYTHON SYNTAX PASS {path}')
required = [
 'src/lunabot_gazebo/package.xml', 'src/lunabot_gazebo/CMakeLists.txt',
 'src/lunabot_gazebo/worlds/lunar_world.sdf',
 'src/lunabot_gazebo/models/lunabot_v4/model.sdf',
 'src/lunabot_gazebo/models/lunabot_v4/model.config',
 'src/lunabot_gazebo/launch/launch_1.launch.py',
 'src/lunabot_tools/package.xml', 'src/lunabot_tools/setup.py',
]
missing = [f for f in required if not pathlib.Path(f).is_file()]
if missing:
    raise SystemExit('Missing required files: ' + ', '.join(missing))
print(f'PASS required files ({len(required)}) and XML/Python syntax')
PY
  then pass 'XML/SDF well-formed, Python syntax, package structure'; else fail 'static parser / structure checks'; fi
  if python3 -m unittest discover -s src/lunabot_tools/test -p 'test_*.py' -v; then
    pass 'obstacle algorithm and command clamp unit tests';
  else fail 'offline unit tests'; fi
  if grep -R -I -nE '(/home/[^ ]+|/root/[^ ]+|~/lunabot)' src/lunabot_gazebo src/lunabot_tools --exclude='*.pyc'; then
    fail 'developer-specific absolute path found in package source';
  else pass 'no developer-specific resource path in packages'; fi
}

run_build() {
  echo '== ROS build =='
  if [[ -z "${ROS_DISTRO:-}" ]]; then
    blocked 'ROS_DISTRO is unset; source the supported ROS environment first.'; return
  fi
  for tool in ros2 colcon rosdep; do
    if ! command -v "$tool" >/dev/null 2>&1; then blocked "$tool is not installed/available"; return; fi
  done
  local -a rosdep_skip_args=()
  if ros2 pkg prefix ament_python >/dev/null 2>&1; then
    echo 'ament_python is present in the sourced ROS underlay; skip its unresolved rosdep key.'
    rosdep_skip_args+=(--skip-keys=ament_python)
  fi
  if rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y "${rosdep_skip_args[@]}" && colcon build --symlink-install; then
    # shellcheck disable=SC1091
    source install/setup.bash
    pass "rosdep + colcon build for $ROS_DISTRO"
  else fail 'dependency resolution or colcon build'; fi
}

run_runtime() {
  echo '== Live simulation acceptance probe =='
  if ! command -v ros2 >/dev/null 2>&1; then blocked 'ROS 2 is unavailable; launch the simulation and source the workspace.'; return; fi
  if ! ros2 node list 2>/dev/null | grep -q 'lunabot_gz_bridge'; then
    blocked 'Phase 1 launch is not detected; start it in another terminal first.'; return
  fi
  if ros2 run lunabot_tools runtime_probe; then pass 'live sensor, odometry, motion, watchdog, and obstacle probe';
  else fail 'live runtime probe'; fi
}

case "$MODE" in
  static) run_static ;;
  build) run_build ;;
  runtime) run_runtime ;;
  all) run_static; run_build; run_runtime ;;
  *) echo "Usage: $0 [static|build|runtime|all]" >&2; exit 64 ;;
esac
if (( FAIL )); then exit 1; fi
if (( BLOCKED )); then exit 2; fi
