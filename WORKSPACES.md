# Role-Isolated Catkin Workspaces

This root-level document is canonical (`workspaces/README.md` is a redirect).

This repository contains two isolated Catkin workspaces to avoid package-name
collisions between:

- `zeno-wholebody-robot`
- `zeno-wholebody-teleop`

## Layout

- `workspaces/ws_robot`: robot-side workspace
- `workspaces/ws_teleop`: teleop-side workspace
- `workspaces/scripts/use_robot.sh`: source robot workspace
- `workspaces/scripts/use_teleop.sh`: source teleop workspace
- `workspaces/scripts/build_robot_ws.sh`: build robot workspace
- `workspaces/scripts/build_teleop_ws.sh`: build teleop workspace
- `LAUNCH_RUNBOOK.md`: lab launch checklist and troubleshooting
- `LAUNCH_RUNBOOK_CN.md`: Chinese launch checklist and troubleshooting
- `RUNBOOK_3ARM_INTERVENTION.md`: 3-arm intervention workflow (OPP switch + recorder)

Each workspace uses symlinks to shared dependencies (`piper_ros`,
`piper_teleop`, `realsense-ros`) plus one zeno side only.

## Launch Guide

For day-to-day startup commands (robot + teleop), CAN naming, camera options,
and troubleshooting, use:

- `LAUNCH_RUNBOOK.md`
- `LAUNCH_RUNBOOK_CN.md`

For 3-arm intervention sessions (robot left/right + OPP + keyboard switch):

- `RUNBOOK_3ARM_INTERVENTION.md`

## Guidebook Selector

Use this table as the document entrypoint:

| Scenario | Use this guide first | Notes |
|---|---|---|
| First-time workspace bring-up | `WORKSPACES.md` | Build/source scripts and isolation rules |
| Daily 2-arm robot+teleop launch | `LAUNCH_RUNBOOK.md` | Canonical baseline in English |
| Daily 2-arm launch (Chinese) | `LAUNCH_RUNBOOK_CN.md` | Same baseline in Chinese |
| 3-arm intervention data collection | `RUNBOOK_3ARM_INTERVENTION.md` | Includes OPP gravity checks and `opp_master_switch.py` |
| Burst pipeline operations | `README_burst_monitor.md` | Burst-specific scripts and checks |
| Host/docker environment setup | `DOCKER_SETUP_README.md` | Host-side environment and container notes |

## 3-Arm Documentation Notes

- Always verify teleop package resolution before a 3-arm session:
  `rospack find teleop_setup`
- In strict mode, `opp_master_switch.py` requires:
  `/robot/arm_opp/joint_states_compensated`
- If OPP compensated topic is missing, check:
  `rosparam get /piper_gravity_compensation_node/enable_opp_arm`

## Safe Usage

Open one terminal per role and source exactly one role script:

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
```

or

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
```

Required system ROS dependency (`dm_hw`):

```bash
sudo apt-get update
sudo apt-get install -y ros-noetic-serial
```

Then build once per workspace:

```bash
/home/jameszhao2004/catkin_ws/workspaces/scripts/build_robot_ws.sh
/home/jameszhao2004/catkin_ws/workspaces/scripts/build_teleop_ws.sh
```

If ROS is not under `/opt/ros`, set:

```bash
export ROS_SETUP_BASH=/path/to/your/ros/setup.bash
```

## Why This Is Low Risk

- Original repo structure is unchanged.
- Runtime switching logic still uses ROS topics/services; this only isolates
  package discovery/build.
- Prevents accidental loading of both duplicated package sets in one Catkin
  environment.

## If You See Duplicate-Package Errors

If `catkin_make` reports duplicated packages from the original
`/home/jameszhao2004/catkin_ws/src`, your current shell has an existing Catkin
overlay sourced.

Use a fresh terminal and run only:

```bash
/home/jameszhao2004/catkin_ws/workspaces/scripts/build_robot_ws.sh
```

or

```bash
/home/jameszhao2004/catkin_ws/workspaces/scripts/build_teleop_ws.sh
```

The scripts now strip existing workspace overlays before building/sourcing.
