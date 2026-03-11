#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   set_teleop_mode.sh teleop   # enable both teleop arms + free-move (gravity-comp style)
#   set_teleop_mode.sh follow   # enable both teleop arms + follow mode
#
# Default mode is "teleop".

MODE="${1:-teleop}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${MODE}" != "teleop" && "${MODE}" != "follow" ]]; then
  echo "[set_teleop_mode] Invalid mode: ${MODE}" >&2
  echo "Usage: $0 [teleop|follow]" >&2
  exit 1
fi

have_piper_enable_type() {
  python3 -c 'import piper_msgs.srv' >/dev/null 2>&1
}

ensure_piper_enable_type() {
  if have_piper_enable_type; then
    return 0
  fi

  # Prefer teleop role workspace; fall back to robot role if needed.
  if [[ -f "${SCRIPT_DIR}/use_teleop.sh" ]]; then
    # shellcheck disable=SC1091
    source "${SCRIPT_DIR}/use_teleop.sh" >/dev/null 2>&1 || true
  fi
  if have_piper_enable_type; then
    return 0
  fi

  if [[ -f "${SCRIPT_DIR}/use_robot.sh" ]]; then
    # shellcheck disable=SC1091
    source "${SCRIPT_DIR}/use_robot.sh" >/dev/null 2>&1 || true
  fi
  if have_piper_enable_type; then
    return 0
  fi

  cat >&2 <<EOF
[set_teleop_mode] Missing ROS service type piper_msgs/Enable in current shell.
[set_teleop_mode] Source a role workspace and try again:

  source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
  # or
  source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh

[set_teleop_mode] If sourcing fails, build first:
  /home/jameszhao2004/catkin_ws/workspaces/scripts/build_teleop_ws.sh
EOF
  return 1
}

wait_for_service() {
  local svc="$1"
  local timeout_s="${2:-10}"
  local start_ts
  start_ts="$(date +%s)"
  while ! rosservice list 2>/dev/null | grep -qx "${svc}"; do
    if (( "$(date +%s)" - start_ts >= timeout_s )); then
      echo "[set_teleop_mode] Timeout waiting for service: ${svc}" >&2
      return 1
    fi
    sleep 0.2
  done
}

ensure_piper_enable_type

echo "[set_teleop_mode] Waiting for teleop enable services..."
wait_for_service "/teleop/arm_left/enable_srv" 15
wait_for_service "/teleop/arm_right/enable_srv" 15

echo "[set_teleop_mode] Enabling left teleop arm..."
rosservice call /teleop/arm_left/enable_srv "enable_request: true"
echo "[set_teleop_mode] Enabling right teleop arm..."
rosservice call /teleop/arm_right/enable_srv "enable_request: true"

if [[ "${MODE}" == "teleop" ]]; then
  echo "[set_teleop_mode] Setting slave_follow_flag=false (teleop/free mode)..."
  rostopic pub --once /conrft_robot/slave_follow_flag std_msgs/Bool "data: false"
else
  echo "[set_teleop_mode] Setting slave_follow_flag=true (follow mode)..."
  rostopic pub --once /conrft_robot/slave_follow_flag std_msgs/Bool "data: true"
fi

echo "[set_teleop_mode] Done."
