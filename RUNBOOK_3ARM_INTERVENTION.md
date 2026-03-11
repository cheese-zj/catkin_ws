# 3-Arm Intervention Runbook

Use this runbook when your session needs:

- robot left/right + `arm_opp` running together
- teleop with OPP gravity compensation enabled
- `opp_master_switch.py` keyboard control
- intervention recording with 3-arm rosbag profile

This runbook is intentionally strict about OPP gravity topic readiness.

This root-level file is canonical (`workspaces/RUNBOOK_3ARM_INTERVENTION.md` is a redirect).

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
  enable_cameras:=true enable_rviz:=true enable_handeye_tf:=false enable_fisheye:=false \
  camera_left_usb_port:=2-3.2  \
  camera_top_usb_port:=2-3.3   \
  camera_right_usb_port:=2-3.4 \
  opp_camera_usb_port:= \
  opp_camera_device:=/dev/wrist_opp_camera \
  opp_camera_fourcc:=YUYV
```

Alternative profile (serial/by-id based, validated in lab on 2026-03-11):

```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  opp_can_port:=can_opp enable_opp_arm:=true \
  enable_cameras:=true enable_opp_camera:=true \
  enable_camera_left:=true enable_camera_right:=true enable_camera_top:=true \
  enable_rviz:=true enable_handeye_tf:=false enable_fisheye:=false \
  camera_initial_reset:=false \
  camera_left_serial_no:=352122273590 \
  camera_right_serial_no:=352122273242 \
  camera_top_serial_no:=348522075799 \
  opp_camera_device:=/dev/v4l/by-id/usb-Alcor_Micro__Corp._PC_camera-video-index0 \
  opp_camera_width:=640 opp_camera_height:=480 opp_camera_fps:=30 \
  opp_camera_fourcc:=YUYV \
  opp_camera_capture_backend:=usb opp_camera_output_encoding:=rgb8
```

If cameras are not needed, set:

```bash
enable_cameras:=false enable_rviz:=false enable_handeye_tf:=false
```

OPP USB camera uses strict `YUYV` here. If the device cannot stream `YUYV`, the node keeps retrying `YUYV` and does not auto-switch to `MJPG`.

One-time udev setup for stable OPP camera path:

```bash
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/install_opp_camera_udev_rule.sh \
  --device /dev/video18 \
  --symlink wrist_opp_camera
```

If you still prefer USB-port pinning, set `opp_camera_usb_port:=1-4:1.0`. When this
is set, the node treats that USB port as authoritative and refuses to fall back to
another `/dev/videoN`.

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

OPP camera quick checks:

```bash
rostopic hz /wrist_opp/image_raw
rostopic hz /wrist_opp/image_raw/compressed
rosparam get /wrist_opp/opp_usb_camera_pub/active_fourcc
```

Expected:

- `image_raw` and/or `image_raw/compressed` has non-zero Hz
- `active_fourcc` is `YUYV`

If the OPP camera still has no image, first identify the resolved `videoN` node from `bash find_all_camera_port.sh`, then check whether another process is holding it:

```bash
fuser -v /dev/video18
```

If you see `chrome`, browser video calls, webcam preview pages, or another capture app, close that process and relaunch the robot-side launch.

For one-shot isolation testing, you can bypass USB-port resolution entirely and target the detected node directly:

```bash
opp_camera_usb_port:= opp_camera_device:=/dev/video18 opp_camera_fourcc:=YUYV
```

After installing the udev rule, prefer:

```bash
opp_camera_usb_port:= opp_camera_device:=/dev/wrist_opp_camera opp_camera_fourcc:=YUYV
```

## 6) Terminal D: OPP keyboard switch

```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/opp_master_switch.py
```

source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/opp_master_switch.py \
  --gravity-switch-align-lock-pre-hold-sec 0.18 \
  --gravity-switch-calm-max-sec 1.2 \
  --attach-switch-post-settle-sec 0.25


Keys:

- `1`: OPP follows teleop left
- `2`: OPP follows teleop right
- `h`: hold OPP
- `s`: print status
- `q`: quit

## 7) Terminal E: Independent 3-Arm Switch Test (`1/2/H/F/Q`)

Use this when you want to test intervention handover logic without touching recorder keybindings.

```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/intervention_3arm_switch_test.py
```

Optional explicit policy command override:

```bash
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/intervention_3arm_switch_test.py \
  --policy-start-cmd "python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_act_checkpoint_ros.py --checkpoint-dir <ckpt_dir> --device cuda --rate 30 --temporal-ensemble-coeff 0.006 --guard-profile medium"
```

Keys:

- `1`: `L->OPP`, `R->R`, `Left Hold`; stop policy
- `2`: `L->L`, `R->OPP`, `Right Hold`; stop policy
- `H`: `L->L`, `R->R`, `OPP Hold`; stop policy
- `F`: restore auto policy (`L/R/OPP -> vla`) and restart policy
- `S`: print current switch status
- `Q`: stop policy, set all arms to hold, then quit

Switch pipeline for `1/2/H`:

- stop policy
- pre-detach any current robot arm route consuming moving teleop masters (switch to hold first)
- switch teleop feedback route (gravity + torque)
- align teleop masters to target robot poses
- apply final robot mux routes

Feedback routing table (gravity + torque):

- mode `1`: teleop-left feedback -> `opp`, teleop-right feedback -> `right`
- mode `2`: teleop-left feedback -> `left`, teleop-right feedback -> `opp`
- mode `H`: teleop-left feedback -> `left`, teleop-right feedback -> `right`
- mode `F/Q/SAFE_HOLD`: restore defaults (`left/right`)

Optional flags:

```bash
--disable-route-gravity-feedback
--disable-route-torque-feedback
--left-gravity-mux-service /teleop/arm_left/gravity_feedback_mux_select
--right-gravity-mux-service /teleop/arm_right/gravity_feedback_mux_select
--left-torque-mux-service /teleop/arm_left/torque_feedback_mux_select
--right-torque-mux-service /teleop/arm_right/torque_feedback_mux_select
```

Quick checks:

```bash
rostopic echo /robot/arm_left/joint_cmd_mux/selected
rostopic echo /robot/arm_right/joint_cmd_mux/selected
rostopic echo /robot/arm_opp/joint_cmd_mux/selected
rostopic echo /teleop/arm_left/gravity_feedback_mux/selected
rostopic echo /teleop/arm_right/gravity_feedback_mux/selected
rostopic echo /teleop/arm_left/torque_feedback_mux/selected
rostopic echo /teleop/arm_right/torque_feedback_mux/selected
rosnode list | grep run_act_checkpoint
rostopic echo -n1 /conrft_robot/slave_follow_flag
```

## 8) Terminal F: 3-arm intervention recorder

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

## 9) Convert 3-arm rosbags to LeRobot

Default (recommended): preserve full state+image overlap window, fill stale action with hold-last.

```bash
cd /home/jameszhao2004/training_codebase
pipeline/scripts/convert_session_3arm.sh \
  --session-dir /home/jameszhao2004/catkin_ws/data/rosbags/<session_name> \
  --dataset-id <dataset_id> \
  --fps 30 \
  --dataset-version v30
```

Legacy strict behavior (old truncation style):

```bash
cd /home/jameszhao2004/training_codebase
pipeline/scripts/convert_session_3arm.sh \
  --session-dir /home/jameszhao2004/catkin_ws/data/rosbags/<session_name> \
  --dataset-id <dataset_id> \
  --fps 30 \
  --dataset-version v30 \
  -- \
  --action-fill-policy strict_drop \
  --teleop-left-topic /teleop/arm_left/joint_states_single \
  --teleop-right-topic /teleop/arm_right/joint_states_single \
  --opp-action-topic /robot/arm_opp/joint_cmd_mux
```

## 10) Optional mode switch helpers

```bash
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh follow
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh teleop
```

## 11) Common failure signatures

- `No messages on /robot/arm_opp/joint_states_compensated`:
  teleop gravity node is not running with `enable_opp_arm=true`.
- `Timeout waiting service /teleop/arm_*/gravity_feedback_mux_select`:
  teleop feedback mux node not launched or namespace mismatch.
- `Transition mode_* failed` right after mode key:
  script auto-enters `SAFE_HOLD`; check feedback mux service names and OPP gravity topic readiness first.
- `/piper_gravity_compensation_node/enable_opp_arm=False`:
  wrong teleop launch/package instance selected.
- `CAN socket ... does not exist`:
  interface naming mismatch or CAN activation missing.
- `ERROR: Unable to communicate with master!`:
  ROS master not running.

## 12) Related documents

- Baseline 2-arm launch and troubleshooting:
  `/home/jameszhao2004/catkin_ws/LAUNCH_RUNBOOK.md`
- Chinese baseline launch guide:
  `/home/jameszhao2004/catkin_ws/LAUNCH_RUNBOOK_CN.md`
- Workspace structure and scripts overview:
  `/home/jameszhao2004/catkin_ws/WORKSPACES.md`
