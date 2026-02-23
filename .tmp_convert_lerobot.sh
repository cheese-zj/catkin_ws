set -euo pipefail
source /opt/ros/noetic/setup.bash
cd /home/jameszhao2004/catkin_ws

python3 -m pip install --user numpy pandas pyarrow opencv-python

python3 workspaces/scripts/rosbag_to_lerobot_v21.py \
  --session-dir data/rosbags/act_20260218_134343 \
  --output-root data/lerobot \
  --dataset-id act_20260218_134343_v21 \
  --task "bimanual teleop"
