# Zeno Teleop Launch Runbook

This is the lab runbook for launching robot-side and teleop-side stacks with
role-isolated Catkin workspaces.

For 3-arm intervention sessions (with `arm_opp` + `opp_master_switch.py`), use:

- `/home/jameszhao2004/catkin_ws/workspaces/RUNBOOK_3ARM_INTERVENTION.md`

## 1) Lab CAN Naming

Use these interface names in this lab:

- Robot left arm: `can_sl`
- Robot right arm: `can_sr`
- Teleop left arm: `can_ml`
- Teleop right arm: `can_mr`

Check current CAN interface mapping:

```bash
bash /home/jameszhao2004/catkin_ws/find_all_can_port.sh
```

## 2) Workspace Rules

- Robot role must use `workspaces/ws_robot`.
- Teleop role must use `workspaces/ws_teleop`.
- Never source both role workspaces in the same shell.
- Always start from a fresh terminal for each role.

Role source scripts:

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
```

## 3) One-Time Build

Install required ROS system package first (`dm_hw` depends on `serial`):

```bash
sudo apt-get update
sudo apt-get install -y ros-noetic-serial
```

If you use Docker alias `ros1`, add `ros-noetic-serial` to the `ros1-piper`
Dockerfile package list and rebuild the image.

Build each isolated workspace once (or after code changes):

```bash
/home/jameszhao2004/catkin_ws/workspaces/scripts/build_robot_ws.sh
/home/jameszhao2004/catkin_ws/workspaces/scripts/build_teleop_ws.sh
```

## 4) Daily Launch Sequence

Open two terminals.

Model default in current integration:

- `robot_description` uses `piper_x_description/urdf/piper_x_description.urdf`.
- Gravity compensation defaults to `piper_x_description/urdf/piper_x_description.urdf`.

Terminal A (robot side):

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  enable_cameras:=false enable_rviz:=false enable_handeye_tf:=false enable_fisheye:=false
```

Terminal B (teleop side):

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
roslaunch teleop_setup start_teleop_all.launch \
  left_can_port:=can_ml right_can_port:=can_mr
```

Optional teleop args:

- Disable paddle/haptics: `enable_paddle:=false`
- Disable gravity compensation: `enable_gravity_compensation:=false`
- Stable mode (recommended for bounce suppression): `gravity_mix_mode:=additive master_slave_enable:=false`
- Use legacy gravity model: `gravity_urdf_package:=piper_description gravity_urdf_relpath:=urdf/piper_description.urdf`
- Legacy gravity scaling (J1-3 at 1/4): `gravity_joint_scale:="[0.25, 0.25, 0.25, 1.0, 1.0, 1.0]"`
- J3 sign fix (both arms): `gravity_joint_scale_left:="[1.0, 1.0, -1.0, 1.0, 1.0, 1.0]" gravity_joint_scale_right:="[1.0, 1.0, -1.0, 1.0, 1.0, 1.0]"`

Optional model fallback (robot + teleop):

- Use legacy `robot_description`: `robot_description_file:=$(find piper_description)/urdf/piper_description.xacro`

## 5) Camera Launch Variant

If camera stack is needed, use robot launch with cameras enabled:

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  enable_cameras:=true enable_rviz:=true enable_handeye_tf:=true enable_fisheye:=false \
  camera_left_usb_port:=2-2.2.1.4 camera_right_usb_port:=2-2.2.1.2 camera_top_usb_port:=2-2.2.1.3
```

Camera behavior in current config:

- Left/right wrist cameras: Infra1 stream enabled.
- Top camera: RGB color stream enabled.
- Depth/pointcloud disabled.
- Lab port mapping:
  left wrist D405 -> `2-2.2.1.4`, right wrist D405 -> `2-2.2.1.2`, top D435 -> `2-2.2.1.3`.

Check USB port mapping first:

```bash
bash /home/jameszhao2004/catkin_ws/find_all_camera_port.sh
```

## 6) Quick Health Checks

Run these after launch:

```bash
rosparam get /piper_ctrl_left_node/can_port
rosparam get /piper_ctrl_right_node/can_port
rosparam get /piper_teleop_left_node/can_port
rosparam get /piper_teleop_right_node/can_port
```

Expected values:

- Robot: `can_sl`, `can_sr`
- Teleop: `can_ml`, `can_mr`

Check gripper reverse setting (should be off for this lab):

```bash
rosparam get /piper_teleop_left_node/gripper_reverse
rosparam get /piper_teleop_right_node/gripper_reverse
```

Expected values:

- `false`
- `false`

Confirm role package resolution:

```bash
rospack find piper_ctrl
rospack find piper_x_description
```

Expected:

- In robot terminal: path under `workspaces/ws_robot/src/...`
- In teleop terminal: path under `workspaces/ws_teleop/src/...`

## 7) Known Issues and Fixes

Duplicate-package error during build:

- Symptom: `Multiple packages found with the same name ...`
- Fix: use a fresh shell and source only one role script.

Wrong CAN socket error:

- Symptom: `CAN socket can_left does not exist`
- Fix: pass explicit launch args listed in Section 4 and re-check params.

No ROS master:

- Symptom: `ERROR: Unable to communicate with master!`
- Fix: make sure one `roslaunch` is running, or start `roscore`.

Missing fisheye package:

- Symptom: `Resource not found: v4l2_cam_launch`
- Fix: keep `enable_fisheye:=false` (default in current robot launch).

Missing serial package:

- Symptom: `Could not find a package configuration file provided by "serial"`
- Fix: install `ros-noetic-serial`, then rebuild:
  `sudo apt-get update && sudo apt-get install -y ros-noetic-serial`

RealSense USB busy:

- Symptom: `RS2_USB_STATUS_BUSY` or `failed to set power state`
- Fix: close competing camera processes, replug camera if needed, relaunch.

## 8) Important Config Files

- Robot main launch:
  `src/zeno-wholebody-intervation/zeno-wholebody-robot/robot_side/robot_setup/launch/start_robot_all.launch`
- Teleop main launch:
  `src/zeno-wholebody-intervation/zeno-wholebody-teleop/teleop_side/teleop_setup/launch/start_teleop_all.launch`
- Robot/teleop parameter map (robot copy):
  `src/zeno-wholebody-intervation/zeno-wholebody-robot/common/piper_ctrl/config/piper_dual.yaml`
- Robot/teleop parameter map (teleop copy):
  `src/zeno-wholebody-intervation/zeno-wholebody-teleop/common/piper_ctrl/config/piper_dual.yaml`

## 9) ACT Keyboard Recording (SPACE/Q)

After launching robot + teleop (Section 4), open a third terminal and run:

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard.py
```

This uses the default profile (`workspaces/config/rosbag_profiles/act_rgb_profile.yaml`) and is the recommended command for this setup.

Optional example:

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard.py \
  --prefix act --notes "pick_place_trial"
```

Keep debug logs (default is auto-clean):

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard.py \
  --keep-debug-logs
```

Keyboard controls:

- `SPACE`: start/stop current episode
- `Q`: safe exit (stops recording first and writes metadata)

Default behavior:

- one bag per episode
- session directory: `<prefix>_<YYYYMMDD>_<HHMMSS>`
- episode numbering: `episode_001`, `episode_002`, ... (auto-increment, no overwrite)
- compression: `--lz4`
- split: disabled by default; enable with `--split --split-size-mb 512`
- output root: `/home/jameszhao2004/catkin_ws/data/rosbags`
- topic profile: `workspaces/config/rosbag_profiles/act_rgb_profile.yaml`
- debug logs: `rosbag_record.log` and `hz_*.log` are auto-cleaned by default
  (use `--keep-debug-logs` for troubleshooting)

Per-episode metadata:

- file: `metadata.json`
- includes start/end time, stop reason, recorded/missing optional topics, bag stats, hz stats, key rosparam snapshot

## 10) Gripper Data Definition (For Recording)

There is no standalone gripper state topic. Gripper data is in `JointState` index `6`:

- `position[6]`: gripper opening position
- `effort[6]`: gripper force/effort feedback

Primary observation topics:

- `/robot/arm_left/joint_states_single`
- `/robot/arm_right/joint_states_single`
- `/teleop/arm_left/joint_states_single`
- `/teleop/arm_right/joint_states_single`

Notes:

- ACT profile records RGB from 3 cameras + dual-arm joint/gripper/pose + core context topics.
- `/tf` and `/tf_static` are excluded.
- paddle-related topics are excluded (not used in this lab).

## 11) ACT Checkpoint Online Inference (Publish-Only)

New script:

- `workspaces/scripts/run_act_checkpoint_ros.py`

Design constraints (important):

- Publish policy commands only to robot-side topics:
  - `/robot/arm_left/vla_joint_cmd`
  - `/robot/arm_right/vla_joint_cmd`
- No `joint_cmd_mux_select` service calls
- No `/conrft_robot/slave_follow_flag` publishing
- No teleop/follow mode switching

### 11.1 Preconditions

1. Robot + teleop stacks are already running.
2. Inference environment is available (recommended: Python>=3.10 venv/uv venv inside ROS1 Docker).
3. Since **February 23, 2026**, training and canonical policy storage are owned by `/home/jameszhao2004/training_codebase`.
4. Sync policy aliases before loading checkpoint shortcuts:
   `bash /home/jameszhao2004/training_codebase/pipeline/scripts/trainctl.sh policy-sync`

### 11.2 Start the policy publisher

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
source /home/jameszhao2004/catkin_ws/.venv_train_act/bin/activate

python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_act_checkpoint_ros.py \
  --checkpoint-dir /home/jameszhao2004/catkin_ws/policies/by_run/<run_name>/best \
  --device cuda \
  --rate 20 \
  --temporal-ensemble-coeff 0.01 \
  --guard-profile medium
```

Optional flags:

- `--disable-gripper`: disable policy gripper control
- `--robot-left-topic` / `--robot-right-topic`: override state input topics
- `--top-camera-topic` / `--left-wrist-topic` / `--right-wrist-topic`: override image input topics
- `--out-left-topic` / `--out-right-topic`: override publish target topics

### 11.3 External switching flow (owned outside this node)

After the publisher starts, robot takeover is decided by external mux switching:

Switch robot input to policy:

```bash
rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd
```

Switch robot input back to teleop:

```bash
rosservice call /robot/arm_left/joint_cmd_mux_select /teleop/arm_left/joint_states_single
rosservice call /robot/arm_right/joint_cmd_mux_select /teleop/arm_right/joint_states_single
```

Teleop follow mode is also external:

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh follow
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh teleop
```

### 11.4 Runtime safety behavior

The node has publish-side guards only (no mode changes):

- `guard-profile=medium` by default (smoothing + per-step clamp + gripper clamp)
- If any input stream is missing/stale/desynced, publishing is paused
- Publishing resumes automatically when streams recover

Even under errors, the node does not modify mux selection or follow flags.

### 11.5 Verified Log (2026-02-18) and quick troubleshooting

Status on **February 18, 2026**: end-to-end policy publisher pipeline is confirmed runnable.

Verification checklist:

1. Start policy node and keep it running:

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
source /home/jameszhao2004/catkin_ws/.venv_train_act/bin/activate
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_act_checkpoint_ros.py \
  --checkpoint-dir /home/jameszhao2004/catkin_ws/policies/by_run/<run_name>/best \
  --device cuda \
  --rate 20 \
  --temporal-ensemble-coeff 0.01 \
  --guard-profile medium
```

2. Check publisher output rate:

```bash
rostopic hz /robot/arm_left/vla_joint_cmd
rostopic hz /robot/arm_right/vla_joint_cmd
```

Expected: close to configured `--rate` (default 20 Hz).

3. Manually switch mux only when you want robot takeover:

```bash
rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd
```

Observed pitfalls and fixes:

- If error shows `ModuleNotFoundError: rospkg` in venv:
  - install once in the same venv:
  - `python -m pip install rospkg catkin_pkg`
- If node starts and appears "quiet":
  - this can be normal in healthy state (no waiting/error logs after startup)
  - validate with `rostopic hz /robot/arm_*/vla_joint_cmd`
- If waiting for streams, use debug mode:
  - `--debug-streams`
  - inspect `raw/valid/drop` counters to separate subscription vs decode vs dimension issues.
