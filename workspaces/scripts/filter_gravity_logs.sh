#!/usr/bin/env bash

set -euo pipefail

# Usage:
#   bash workspaces/scripts/filter_gravity_logs.sh
#   bash workspaces/scripts/filter_gravity_logs.sh "MIT torque clamped|Clipped ARM joints"
#
# Optional env vars:
#   ROSOUT_TOPIC=/rosout_agg
#
# The default pattern focuses on gravity/limit diagnostics we are currently using.

TOPIC="${ROSOUT_TOPIC:-/rosout_agg}"
PATTERN="${1:-Gravity torque pushes toward|MIT torque clamped on joint|Clipped ARM joints to URDF limits|Torque jump on joint|Joint step clamped on joint|Applied angle unwrap}"

echo "[filter_gravity_logs] topic: ${TOPIC}"
echo "[filter_gravity_logs] pattern: ${PATTERN}"
echo "[filter_gravity_logs] press Ctrl-C to stop"

rostopic echo "${TOPIC}" | grep --line-buffered -E "${PATTERN}"
