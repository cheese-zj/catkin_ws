#!/usr/bin/env bash
set -euo pipefail

cd /home/jameszhao2004/catkin_ws

python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/lerobot_to_rosbag.py "$@"
