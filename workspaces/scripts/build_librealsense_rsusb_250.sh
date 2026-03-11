#!/usr/bin/env bash
set -euo pipefail

SRC_DIR="${SRC_DIR:-$HOME/librealsense-2.50.0-rsusb}"
BUILD_DIR="${BUILD_DIR:-$SRC_DIR/build-rsusb}"
INSTALL_DIR="${INSTALL_DIR:-$HOME/.local/librealsense-2.50.0-rsusb}"
TAG="${TAG:-v2.50.0}"

if [[ ! -f /usr/include/libusb-1.0/libusb.h ]]; then
  cat >&2 <<'EOF'
Missing libusb development headers.

Install them first:
  sudo apt-get update
  sudo apt-get install libusb-1.0-0-dev
EOF
  exit 1
fi

if [[ ! -d "$SRC_DIR/.git" ]]; then
  git clone --branch "$TAG" --depth 1 \
    https://github.com/IntelRealSense/librealsense.git \
    "$SRC_DIR"
fi

cmake -S "$SRC_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
  -DFORCE_RSUSB_BACKEND=true \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false \
  -DBUILD_PYTHON_BINDINGS=false \
  -DLIBUSB_INC=/usr/include/libusb-1.0 \
  -DLIBUSB_LIB=/usr/lib/x86_64-linux-gnu/libusb-1.0.so

cmake --build "$BUILD_DIR" --target install -j"$(nproc)"

cat <<EOF

RSUSB build installed to:
  $INSTALL_DIR

Use it in a robot shell with:
  source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot_rsusb.sh
EOF
