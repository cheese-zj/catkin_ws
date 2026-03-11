#!/usr/bin/env bash
set -euo pipefail

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
elif [[ -f /opt/ros/melodic/setup.bash ]]; then
  source /opt/ros/melodic/setup.bash
else
  echo "[rebuild_robot_realsense_rsusb] ROS setup not found under /opt/ros/{noetic,melodic}." >&2
  exit 1
fi

RSUSB_PREFIX="${RSUSB_PREFIX:-$HOME/.local/librealsense-2.50.0-rsusb}"
RSUSB_CMAKE_DIR="$RSUSB_PREFIX/lib/cmake/realsense2"
WS_DIR="/home/jameszhao2004/catkin_ws/workspaces/ws_robot"

if [[ ! -f "$RSUSB_CMAKE_DIR/realsense2Config.cmake" ]]; then
  echo "[rebuild_robot_realsense_rsusb] Missing RSUSB SDK at $RSUSB_CMAKE_DIR" >&2
  echo "[rebuild_robot_realsense_rsusb] Build it first with build_librealsense_rsusb_250.sh" >&2
  exit 1
fi

export realsense2_DIR="$RSUSB_CMAKE_DIR"
export CMAKE_PREFIX_PATH="$RSUSB_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$RSUSB_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$WS_DIR"

if ! command -v catkin >/dev/null 2>&1; then
  echo "[rebuild_robot_realsense_rsusb] catkin is required for this partial rebuild." >&2
  exit 1
fi

catkin clean -y realsense2_camera realsense2_description || true
catkin build realsense2_camera realsense2_description \
  --no-deps \
  --force-cmake \
  --cmake-args -Drealsense2_DIR="$RSUSB_CMAKE_DIR"

PLUGIN_LIB="$WS_DIR/devel/lib/librealsense2_camera.so"

if [[ -f "$PLUGIN_LIB" ]]; then
  echo
  echo "[rebuild_robot_realsense_rsusb] Linked dependencies for $PLUGIN_LIB:"
  ldd "$PLUGIN_LIB" | grep librealsense || true
  echo
  echo "[rebuild_robot_realsense_rsusb] ELF search path for $PLUGIN_LIB:"
  readelf -d "$PLUGIN_LIB" | grep -E 'RPATH|RUNPATH' || true
fi

cat <<EOF

Rebuild complete.

Use the RSUSB runtime with:
  source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot_rsusb.sh
EOF
