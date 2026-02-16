#!/usr/bin/env bash
set -euo pipefail

source_ros_setup() {
  if [[ -n "${ROS_SETUP_BASH:-}" ]] && [[ -f "${ROS_SETUP_BASH}" ]]; then
    source "${ROS_SETUP_BASH}"
    return 0
  fi
  if [[ -f /opt/ros/noetic/setup.bash ]]; then
    source /opt/ros/noetic/setup.bash
    return 0
  fi
  if [[ -f /opt/ros/melodic/setup.bash ]]; then
    source /opt/ros/melodic/setup.bash
    return 0
  fi
  echo "[build_robot_ws] ROS setup not found under /opt/ros/{noetic,melodic}." >&2
  return 1
}

strip_workspace_overlay() {
  local root="$1"
  local var value entry filtered
  for var in CMAKE_PREFIX_PATH ROS_PACKAGE_PATH; do
    value="${!var:-}"
    filtered=""
    IFS=':' read -r -a _entries <<< "${value}"
    for entry in "${_entries[@]}"; do
      [[ -z "${entry}" ]] && continue
      case "${entry}" in
        "${root}"|${root}/*) ;;
        *) filtered="${filtered:+${filtered}:}${entry}" ;;
      esac
    done
    export "${var}=${filtered}"
  done
  unset _CATKIN_SETUP_DIR CATKIN_ENV_HOOK_WORKSPACE CATKIN_SHELL
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)/ws_robot"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

strip_workspace_overlay "${ROOT_DIR}"
source_ros_setup
cd "${WS_DIR}"

if command -v catkin >/dev/null 2>&1; then
  catkin build
elif command -v catkin_make >/dev/null 2>&1; then
  catkin_make
else
  echo "[build_robot_ws] Neither catkin nor catkin_make is available in PATH." >&2
  exit 1
fi

echo "[build_robot_ws] Build complete: ${WS_DIR}"
