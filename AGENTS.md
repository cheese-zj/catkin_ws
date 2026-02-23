# AGENTS.md

Handoff summary for future Codex sessions in this workspace.

Last updated: **2026-02-21**

## Scope and Goal

Primary integration target is still dual-arm teleop in `catkin_ws` using:

- `src/zeno-wholebody-intervation/zeno-wholebody-robot`
- `src/zeno-wholebody-intervation/zeno-wholebody-teleop`

Recent work also added an intervention-data pipeline:

- keyboard rosbag recording
- rosbag -> LeRobot dataset conversion
- ACT checkpoint online publish-only inference

Main constraints:

- Robot and teleop trees still contain duplicated ROS package names.
- Lab CAN names are custom: `can_sl`, `can_sr`, `can_ml`, `can_mr` (plus `can_opp` occasionally).
- Fisheye stack is not used in normal operation; RealSense path is the default.

## Workspace Architecture (Critical)

Do **not** merge robot + teleop package trees into one Catkin source space.
Use role-isolated workspaces:

- `workspaces/ws_robot`
- `workspaces/ws_teleop`

Use these scripts only:

- `workspaces/scripts/use_robot.sh`
- `workspaces/scripts/use_teleop.sh`
- `workspaces/scripts/build_robot_ws.sh`
- `workspaces/scripts/build_teleop_ws.sh`

Notes:

- Source scripts strip existing Catkin overlays to avoid duplicate-package collisions.
- Build scripts now check for missing ROS dependency `serial` and print install guidance.
- Optional auto-install path exists: set `AUTO_INSTALL_MISSING_ROS_DEPS=1`.

## Canonical Runbooks

Source of truth for launch and diagnostics:

- English: `workspaces/LAUNCH_RUNBOOK.md`
- Chinese: `workspaces/LAUNCH_RUNBOOK_CN.md`

Keep AGENTS concise; put detailed operator flow in runbooks above.

## Lab Interface Mapping

CAN naming used in this lab:

- Robot left arm: `can_sl`
- Robot right arm: `can_sr`
- Teleop left arm: `can_ml`
- Teleop right arm: `can_mr`

Current camera USB mapping used in runbooks:

- left wrist D405: `2-2.2.1.4`
- right wrist D405: `2-2.2.1.2`
- top D435: `2-2.2.1.3`

Helper checks:

- `bash /home/jameszhao2004/catkin_ws/find_all_can_port.sh`
- `bash /home/jameszhao2004/catkin_ws/find_all_camera_port.sh`

## Known-Good Launch Baseline

Terminal A (robot, minimal/no cameras):

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  enable_cameras:=false enable_rviz:=false enable_handeye_tf:=false enable_fisheye:=false
```

Terminal B (teleop):

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
roslaunch teleop_setup start_teleop_all.launch \
  left_can_port:=can_ml right_can_port:=can_mr
```

Camera-enabled robot variant:

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  enable_cameras:=true enable_rviz:=true enable_handeye_tf:=true enable_fisheye:=false \
  camera_left_usb_port:=2-2.2.1.4 camera_right_usb_port:=2-2.2.1.2 camera_top_usb_port:=2-2.2.1.3
```

## Current Config Reality (Important)

### Teleop launch and control behavior

File:

- `src/zeno-wholebody-intervation/zeno-wholebody-teleop/teleop_side/teleop_setup/launch/start_teleop_all.launch`

Current behavior:

- defaults to `left_can_port:=can_ml`, `right_can_port:=can_mr`
- supports runtime overrides for:
  - MIT/gravity mix (`gravity_mix_mode`, `mit_enable_tor`, torque scales/sign)
  - shutdown safety (`shutdown_mode`, hold/disable retry params)
  - gripper haptic shaping params (sign/deadband/bias/max/filter/opening relief)
- passes `left_can_port` and `right_can_port` into `piper_dual_teleop.launch`

### Gripper haptic implementation changed (old one-line experiment is obsolete)

File:

- `src/zeno-wholebody-intervation/zeno-wholebody-teleop/common/piper_ctrl/scripts/piper_ctrl_node.py`

Current state:

- No longer a single `_mit_control` sign flip.
- Uses `_compute_gripper_haptic_effort_cmd()` with configurable shaping.
- If `gripper_haptic_effort_sign` is unset (`0.0`), sign derives from `gripper_reverse`.

### Teleop gripper reverse in lab config

File:

- `src/zeno-wholebody-intervation/zeno-wholebody-teleop/common/piper_ctrl/config/piper_dual.yaml`

Expected for this lab:

- `piper_teleop_left_node.gripper_reverse: false`
- `piper_teleop_right_node.gripper_reverse: false`

### Robot launch caveat (args exist but effective CAN source is YAML)

File:

- `src/zeno-wholebody-intervation/zeno-wholebody-robot/robot_side/robot_setup/launch/start_robot_all.launch`

Current state:

- launch has args `left_can_port` / `right_can_port`
- robot CAN values effectively come from:
  - `src/zeno-wholebody-intervation/zeno-wholebody-robot/common/piper_ctrl/config/piper_dual.yaml`
- this YAML currently sets robot CAN to `can_sl` and `can_sr`

### `piper_dual.yaml` load pattern differs by role

- Teleop:
  - `src/zeno-wholebody-intervation/zeno-wholebody-teleop/common/piper_ctrl/launch/piper_dual_teleop.launch`
  - loads `piper_dual.yaml` once globally (`ns="/"`), then applies per-node overrides
- Robot:
  - `src/zeno-wholebody-intervation/zeno-wholebody-robot/common/piper_ctrl/launch/piper_dual_robot.launch`
  - still loads `piper_dual.yaml` inside each node block

## Intervention and ACT Pipeline (New in Recent Weeks)

### 1) Keyboard rosbag recording

Script:

- `workspaces/scripts/record_act_keyboard.py`

Defaults:

- profile: `workspaces/config/rosbag_profiles/act_rgb_profile.yaml`
- output root: `/home/jameszhao2004/catkin_ws/data/rosbags`
- controls: `SPACE` start/stop episode, `Q` safe quit
- per-episode `metadata.json` with rosparam snapshot and topic/hz stats

Useful flags:

- `--camera-transport compressed|raw|profile`
- `--keep-debug-logs`
- `--split --split-size-mb 512`

### 2) Teleop/follow + mux switching helpers

Scripts:

- `workspaces/scripts/set_teleop_mode.sh` (`teleop` or `follow`)
- `workspaces/scripts/fake_policy_loop.py` (publish test `vla_joint_cmd`)

Manual mux services:

- `/robot/arm_left/joint_cmd_mux_select`
- `/robot/arm_right/joint_cmd_mux_select`

### 3) ACT checkpoint online inference (publish-only)

Script:

- `workspaces/scripts/run_act_checkpoint_ros.py`

Design constraints:

- publishes only to `/robot/arm_left/vla_joint_cmd` and `/robot/arm_right/vla_joint_cmd`
- does **not** call mux services
- does **not** publish `/conrft_robot/slave_follow_flag`
- does **not** switch teleop/follow modes

Runtime guards included (staleness/sync checks + command smoothing/clamp).

### 4) Rosbag -> LeRobot conversion

Scripts:

- `workspaces/scripts/rosbag_to_lerobot_v21.py`
- `workspaces/scripts/convert_lerobot_uv.sh` (recommended wrapper)

Recent additions:

- optional image timestamp mode `--image-use-bag-time`
- optional in-place v2.1 -> v3.0 conversion flags

Practical note:

- direct converter default `--output-root` is machine-specific and may be wrong
- prefer wrapper script or pass explicit `--output-root`

## Fast Checks After Launch

```bash
rosparam get /piper_ctrl_left_node/can_port
rosparam get /piper_ctrl_right_node/can_port
rosparam get /piper_teleop_left_node/can_port
rosparam get /piper_teleop_right_node/can_port
rosparam get /piper_teleop_left_node/gripper_reverse
rosparam get /piper_teleop_right_node/gripper_reverse
```

Expected:

- robot: `can_sl`, `can_sr`
- teleop: `can_ml`, `can_mr`
- teleop gripper reverse: `false`, `false`

Optional tor/gravity checks when testing force feedback:

```bash
rosparam get /piper_teleop_left_node/mit/enable_tor
rosparam get /piper_teleop_right_node/mit/enable_tor
rosparam get /piper_teleop_left_node/mit/gravity_mix_mode
rosparam get /piper_teleop_right_node/mit/gravity_mix_mode
```

## Common Failure Patterns

- Duplicate package names during build:
  - use fresh shell; source only one role script
- `CAN socket can_left does not exist`:
  - wrong CAN params or stale assumptions about active YAML/launch overrides
- `Resource not found: v4l2_cam_launch`:
  - fisheye package absent; keep `enable_fisheye:=false`
- `Could not find package "serial"` / CMake serial error:
  - install `ros-noetic-serial` and rebuild
- `ERROR: Unable to communicate with master!`:
  - no `roscore` / `roslaunch` master
- RealSense busy (`RS2_USB_STATUS_BUSY`, power-state errors):
  - close competing camera processes, replug camera, relaunch

## Notes for Future Agents

- Prefer editing `zeno-wholebody-teleop` tree for teleop behavior changes.
- Keep robot/teleop duplicate configs aligned only when runtime needs it.
- Runtime behavior depends on sourced role workspace, not on editing both trees.
- `catkin_ws` root **is** a Git repo in this environment now; `git status` at root is valid.
