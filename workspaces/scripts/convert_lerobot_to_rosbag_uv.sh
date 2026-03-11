#!/usr/bin/env bash
set -euo pipefail

cd /home/jameszhao2004/catkin_ws

if ! command -v uv >/dev/null 2>&1; then
  python3 -m pip install --user uv
  export PATH="$HOME/.local/bin:$PATH"
fi

uv venv /home/jameszhao2004/catkin_ws/.venv_lerobot_to_rosbag \
  --python "$(command -v python3)"
source /home/jameszhao2004/catkin_ws/.venv_lerobot_to_rosbag/bin/activate

uv pip install -U numpy pyarrow opencv-python-headless av rosbags pyyaml

python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/lerobot_to_rosbag.py "$@"
