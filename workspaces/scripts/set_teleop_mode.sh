#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   set_teleop_mode.sh teleop   # enable both teleop arms + free-move (gravity-comp style)
#   set_teleop_mode.sh follow   # enable both teleop arms + follow mode
#
# Default mode is "teleop".

MODE="${1:-teleop}"

if [[ "${MODE}" != "teleop" && "${MODE}" != "follow" ]]; then
  echo "[set_teleop_mode] Invalid mode: ${MODE}" >&2
  echo "Usage: $0 [teleop|follow]" >&2
  exit 1
fi

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
