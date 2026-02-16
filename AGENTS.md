# AGENTS.md

This file is a handoff summary for future Codex sessions in this workspace.

## Scope and Goal

The team is integrating `zeno-wholebody-intervation` into `catkin_ws` for dual-arm teleop.
Main constraints:

- Robot and teleop trees contain duplicated ROS package names.
- Lab CAN names are custom (`can_sl`, `can_sr`, `can_ml`, `can_mr`).
- Fisheye stack is not used; RealSense-based camera setup is used.

## Workspace Architecture (Important)

Do **not** merge robot + teleop package trees into one Catkin source space.
Use role-isolated workspaces:

- `workspaces/ws_robot`
- `workspaces/ws_teleop`

Use scripts:

- `workspaces/scripts/use_robot.sh`
- `workspaces/scripts/use_teleop.sh`
- `workspaces/scripts/build_robot_ws.sh`
- `workspaces/scripts/build_teleop_ws.sh`

These scripts strip existing Catkin overlays to avoid duplicate-package collisions.

## Primary Runbooks

- English: `workspaces/LAUNCH_RUNBOOK.md`
- Chinese: `workspaces/LAUNCH_RUNBOOK_CN.md`

These are the source of truth for day-to-day launch commands and diagnostics.

## Key Launch/Config Changes Already Applied

### 1) Robot launch CAN defaults and hard overrides

File:
- `src/zeno-wholebody-intervation/zeno-wholebody-robot/robot_side/robot_setup/launch/start_robot_all.launch`

Behavior:
- defaults: `left_can_port:=can_sl`, `right_can_port:=can_sr`
- hard params:
  - `/piper_ctrl_left_node/can_port`
  - `/piper_ctrl_right_node/can_port`
- includes `piper_dual_robot.launch` with these args.

### 2) Teleop launch CAN defaults and hard overrides

File:
- `src/zeno-wholebody-intervation/zeno-wholebody-teleop/teleop_side/teleop_setup/launch/start_teleop_all.launch`

Behavior:
- defaults: `left_can_port:=can_ml`, `right_can_port:=can_mr`
- hard params:
  - `/piper_teleop_left_node/can_port`
  - `/piper_teleop_right_node/can_port`

### 3) Fixed CAN param race in dual-arm launch

Problem found:
- `piper_dual.yaml` was loaded with `ns="/"` inside each node block.
- second node load could overwrite the first node's `can_port`.

Fix:
- load `piper_dual.yaml` once globally in:
  - `src/zeno-wholebody-intervation/zeno-wholebody-robot/common/piper_ctrl/launch/piper_dual_robot.launch`
  - `src/zeno-wholebody-intervation/zeno-wholebody-teleop/common/piper_ctrl/launch/piper_dual_teleop.launch`
- keep per-node `can_port` overrides from launch args.

### 4) Camera/fisheye behavior

In robot launch:
- fisheye default disabled (`enable_fisheye:=false`).
- RealSense enabled path added.
- left/right configured around infra stream use.
- top camera configured for RGB.
- depth/pointcloud disabled in this setup.

### 5) Gripper reverse

Teleop gripper reverse was changed to `false` for this lab:

- `src/zeno-wholebody-intervation/zeno-wholebody-teleop/common/piper_ctrl/config/piper_dual.yaml`
  - `piper_teleop_left_node.gripper_reverse: false`
  - `piper_teleop_right_node.gripper_reverse: false`

Mirrored in robot-side duplicate config as safety:
- `src/zeno-wholebody-intervation/zeno-wholebody-robot/common/piper_ctrl/config/piper_dual.yaml`

## Current Experimental Change (Do Not Miss)

For quick haptic validation, MIT-mode gripper effort sign was changed in teleop controller:

- file:
  `src/zeno-wholebody-intervation/zeno-wholebody-teleop/common/piper_ctrl/scripts/piper_ctrl_node.py`
- around line ~641 in `_mit_control`:
  - from: `gripper_effort = -self.gripper_cmd_effort`
  - to:   `gripper_effort = self.gripper_cmd_effort`

Status: experimental, not yet marked as final decision.
If needed, revert this single line.

## Known-Good Launch Pattern

Terminal A (robot):

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  enable_cameras:=false enable_rviz:=false enable_handeye_tf:=false
```

Terminal B (teleop):

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
roslaunch teleop_setup start_teleop_all.launch \
  left_can_port:=can_ml right_can_port:=can_mr
```

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
- gripper reverse: `false`, `false`

## Common Failure Patterns

- Duplicate package names during build:
  use fresh shell; source only one role script.
- `CAN socket can_left does not exist`:
  wrong can params or stale param override.
- `Resource not found: v4l2_cam_launch`:
  fisheye package absent; keep fisheye disabled.
- `ERROR: Unable to communicate with master!`:
  roscore/roslaunch not running.

## Notes for Future Agents

- `catkin_ws` root itself is not a Git repo in this environment; avoid relying on `git status` at root.
- Prefer editing the `zeno-wholebody-teleop` tree when changing teleop behavior.
- Keep robot/teleop config copies consistent only when needed; runtime behavior depends on the sourced role workspace.
