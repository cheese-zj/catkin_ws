# 3-Arm Intervention Runbook

Use this runbook when your session needs:

- robot left/right + `arm_opp` running together
- teleop with OPP gravity compensation enabled
- `opp_master_switch.py` keyboard control
- intervention recording with 3-arm rosbag profile

This runbook is intentionally strict about OPP gravity topic readiness.

## 1) Preconditions

- CAN interfaces are up: `can_sl`, `can_sr`, `can_opp`, `can_ml`, `can_mr`
- Camera USB mapping is verified if cameras are enabled
- One shell per role (robot / teleop), never source both role workspaces in one shell

Helpers:

```bash
bash /home/jameszhao2004/catkin_ws/find_all_can_port.sh
bash /home/jameszhao2004/catkin_ws/find_all_camera_port.sh
```

## 2) Terminal A: ROS master

```bash
ros1
roscore
```

## 3) Terminal B: Robot side (with OPP arm)

```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  opp_can_port:=can_opp enable_opp_arm:=true \
  enable_cameras:=true enable_rviz:=true enable_handeye_tf:=true enable_fisheye:=false \
  camera_left_usb_port:=2-2.2.1.4 camera_right_usb_port:=2-2.2.1.2 camera_top_usb_port:=2-2.2.1.3
```

If cameras are not needed, set:

```bash
enable_cameras:=false enable_rviz:=false enable_handeye_tf:=false
```

## 4) Terminal C: Teleop side (must enable OPP gravity path)

```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
rospack find teleop_setup
roslaunch /home/jameszhao2004/catkin_ws/src/zeno-wholebody-intervation/zeno-wholebody-teleop/teleop_side/teleop_setup/launch/start_teleop_all.launch \
  left_can_port:=can_ml right_can_port:=can_mr \
  enable_paddle:=false \
  enable_opp_arm:=true \
  master_slave_enable:=true mit_enable_tor:=true \
  mit_enable_pos:=true \
  mit_max_torque_abs:=8.0 \
  mit_torque_scale_left:="[1, 0.12, 0.2, 0.1, 0.2, 0.20]" \
  mit_torque_scale_right:="[1, 0.12, 0.2, 0.1, 0.2, 0.20]" \
  mit_torque_feedback_sign_left:="[-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]" \
  mit_torque_feedback_sign_right:="[-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]" \
  gravity_mix_mode:=additive \
  gravity_unwrap_joint_positions:=true \
  gravity_max_joint_step:=0.08 \
  gravity_max_torque_delta_warn:=2.0 \
  enforce_joint_limits:=true \
  gravity_joint_scale_left:="[1.0, 0.96, 0.96, 0.94, 1.0, 1.0]" \
  gravity_joint_scale_right:="[1.0, 0.96, 0.96, 0.94, 1.0, 1.0]" \
  gripper_haptic_effort_sign:=-1.0 \
  gripper_haptic_effort_deadband:=0.0 \
  gripper_haptic_effort_bias:=0.20 \
  gripper_haptic_effort_max:=0.02 \
  gripper_haptic_cmd_max:=2000 \
  gripper_haptic_cmd_enable_threshold:=1 \
  gripper_haptic_effort_alpha:=0.85 \
  gripper_haptic_release_when_opening:=true \
  gripper_haptic_opening_relief:=0.00 \
  gripper_haptic_opening_sign:=1.0 \
  gripper_haptic_opening_direction_eps:=0.00002
```

Expected `rospack` path:

- `/home/jameszhao2004/catkin_ws/src/zeno-wholebody-intervation/zeno-wholebody-teleop/teleop_side/teleop_setup`

## 5) Mandatory 3-arm readiness checks

Run before `opp_master_switch.py`:

```bash
rosparam get /piper_gravity_compensation_node/enable_opp_arm
rostopic hz /robot/arm_opp/joint_states_single
rostopic hz /robot/arm_opp/joint_states_compensated
```

Expected:

- `/piper_gravity_compensation_node/enable_opp_arm` is `true`
- both OPP topics stream at stable rate

If `enable_opp_arm` is `false` while launch arg was true, you likely launched the wrong `teleop_setup` package instance. Re-launch Terminal C using the absolute launch path shown above.

## 6) Terminal D: OPP keyboard switch

```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/opp_master_switch.py
```

Keys:

- `1`: OPP follows teleop left
- `2`: OPP follows teleop right
- `h`: hold OPP
- `s`: print status
- `q`: quit

## 7) Terminal E: 3-arm intervention recorder

```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard_intervention.py \
  --profile /home/jameszhao2004/catkin_ws/workspaces/config/rosbag_profiles/act_rgb_3arm_profile.yaml
```

Keyboard:

- `S`: start episode
- `SPACE`: stop episode
- `P`: pause
- `R`: resume
- `Q`: quit safely

## 8) Optional mode switch helpers

```bash
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh follow
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh teleop
```

## 9) Common failure signatures

- `No messages on /robot/arm_opp/joint_states_compensated`:
  teleop gravity node is not running with `enable_opp_arm=true`.
- `/piper_gravity_compensation_node/enable_opp_arm=False`:
  wrong teleop launch/package instance selected.
- `CAN socket ... does not exist`:
  interface naming mismatch or CAN activation missing.
- `ERROR: Unable to communicate with master!`:
  ROS master not running.

## 10) Related documents

- Baseline 2-arm launch and troubleshooting:
  `/home/jameszhao2004/catkin_ws/workspaces/LAUNCH_RUNBOOK.md`
- Chinese baseline launch guide:
  `/home/jameszhao2004/catkin_ws/workspaces/LAUNCH_RUNBOOK_CN.md`
- Workspace structure and scripts overview:
  `/home/jameszhao2004/catkin_ws/workspaces/README.md`
