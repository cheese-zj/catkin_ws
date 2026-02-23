#!/usr/bin/env bash
set -euo pipefail
cd /home/jameszhao2004/catkin_ws

SESSION_DIR="data/rosbags/act_20260218_134343"
OUTPUT_ROOT="data/lerobot"
DATASET_ID="act_20260218_134343_v21"
TASK_TEXT="bimanual teleop"
FPS="30"
MIN_FRAMES="32"
CONVERT_TO_V30="true"
V30_PUSH_TO_HUB="false"
V30_FORCE_CONVERSION="false"

usage() {
  cat <<'EOF'
Usage:
  bash workspaces/scripts/convert_lerobot_uv.sh [options]

Options:
  --session-dir <path>          Rosbag session dir with episode_*/episode.bag
  --output-root <path>          Parent output dir for dataset folder
  --dataset-id <name>           Dataset folder name
  --task <text>                 Task text saved in metadata
  --fps <number>                Target sampling fps (default: 30)
  --min-frames <int>            Minimum valid synced frames per episode (default: 32)
  --no-v30                      Keep output at v2.1 format only
  --v30-push-to-hub <true|false>
                                Push converted v3.0 dataset to hub (default: false)
  --v30-force-conversion        Forward --force-conversion to LeRobot converter (hub-oriented)
  --help                        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session-dir)
      SESSION_DIR="$2"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --dataset-id)
      DATASET_ID="$2"
      shift 2
      ;;
    --task)
      TASK_TEXT="$2"
      shift 2
      ;;
    --fps)
      FPS="$2"
      shift 2
      ;;
    --min-frames)
      MIN_FRAMES="$2"
      shift 2
      ;;
    --no-v30)
      CONVERT_TO_V30="false"
      shift
      ;;
    --v30-push-to-hub)
      V30_PUSH_TO_HUB="$2"
      shift 2
      ;;
    --v30-force-conversion)
      V30_FORCE_CONVERSION="true"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ROS setup file not found: /opt/ros/noetic/setup.bash" >&2
  echo "Run this script in your ROS Noetic environment." >&2
  exit 1
fi
source /opt/ros/noetic/setup.bash

if ! command -v uv >/dev/null 2>&1; then
  python3 -m pip install --user uv
  export PATH="$HOME/.local/bin:$PATH"
fi

uv venv /home/jameszhao2004/catkin_ws/.venv_convert_ros \
  --python "$(command -v python3)" \
  --system-site-packages
source /home/jameszhao2004/catkin_ws/.venv_convert_ros/bin/activate

uv pip install -U numpy pandas pyarrow opencv-python "lerobot==0.4.3"
python3 -c "import rosbag,numpy,pandas,cv2,pyarrow,lerobot; print('deps ok, lerobot', lerobot.__version__)"

CMD=(
  python3 workspaces/scripts/rosbag_to_lerobot_v21.py
  --session-dir "$SESSION_DIR"
  --output-root "$OUTPUT_ROOT"
  --dataset-id "$DATASET_ID"
  --task "$TASK_TEXT"
  --fps "$FPS"
  --min-frames "$MIN_FRAMES"
)

if [[ "$CONVERT_TO_V30" == "true" ]]; then
  CMD+=(--convert-to-v30 --v30-push-to-hub "$V30_PUSH_TO_HUB")
  if [[ "$V30_FORCE_CONVERSION" == "true" ]]; then
    CMD+=(--v30-force-conversion)
  fi
fi

"${CMD[@]}"

python3 - "$OUTPUT_ROOT" "$DATASET_ID" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1]).expanduser().resolve()
dataset_id = sys.argv[2]
info_path = root / dataset_id / "meta" / "info.json"

if not info_path.is_file():
    raise SystemExit(f"missing metadata file: {info_path}")

info = json.loads(info_path.read_text(encoding="utf-8"))
version = info.get("codebase_version", "unknown")
episodes = info.get("total_episodes", "unknown")
frames = info.get("total_frames", "unknown")
print(f"dataset={dataset_id} version={version} episodes={episodes} frames={frames}")
PY
