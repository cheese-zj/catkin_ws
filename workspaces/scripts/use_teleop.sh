#!/usr/bin/env bash

if [[ -n "${ZSH_VERSION:-}" ]]; then
  # shellcheck disable=SC2296
  _THIS_FILE="${(%):-%N}"
elif [[ -n "${BASH_VERSION:-}" ]]; then
  _THIS_FILE="${BASH_SOURCE[0]}"
else
  _THIS_FILE="$0"
fi

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

_source_ros_setup() {
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
  echo "[use_teleop] ROS setup not found under /opt/ros/{noetic,melodic}." >&2
  return 1
}

_SCRIPT_DIR="$(cd "$(dirname "${_THIS_FILE}")" && pwd)"
_WS_DIR="$(cd "${_SCRIPT_DIR}/.." && pwd)/ws_teleop"
_ROOT_DIR="$(cd "${_SCRIPT_DIR}/../.." && pwd)"

strip_workspace_overlay "${_ROOT_DIR}"
_source_ros_setup || { return 1 2>/dev/null || exit 1; }

if [[ -f "${_WS_DIR}/devel/setup.bash" ]]; then
  source "${_WS_DIR}/devel/setup.bash"
else
  echo "[use_teleop] Workspace not built yet: ${_WS_DIR}" >&2
  echo "[use_teleop] Run: ${_SCRIPT_DIR}/build_teleop_ws.sh" >&2
  { return 1 2>/dev/null || exit 1; }
fi

export ZENO_ROLE_WORKSPACE=teleop
echo "[use_teleop] Active workspace: ${_WS_DIR}"
