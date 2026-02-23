

#

## Activate CAN&Camera

# new can setup

can_sl = 1-2.2.4:1.0
can_sr = 1-2.2.3:1.0
can_opp= 1-2.2.2:1.0
can_ml = 1-2.3:1.0
can_mr = 1-2.4:1.0

bash can_activate.sh can_ml 1000000 1-2.3:1.0
bash can_activate.sh can_mr 1000000 1-2.4:1.0
bash can_activate.sh can_sl 1000000 1-2.2.4:1.0
bash can_activate.sh can_sr 1000000 1-2.2.3:1.0
bash can_activate.sh can_opp 1000000 1-2.2.2:1.0

bash find_all_camera_port.sh

wrist_cam_r = 2-2.2.1.2
wrist_cam_l = 2-2.2.1.4
overhead_cam = 2-2.2.1.3
face_cam = 1-2.1



## Start Teleop System

Terminal 1 roscore
```bash
ros1
roscore
```

Terminal 2 robot node
```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  enable_cameras:=true enable_rviz:=true enable_handeye_tf:=true enable_fisheye:=false \
  camera_left_usb_port:=2-2.2.1.4 camera_right_usb_port:=2-2.2.1.2 camera_top_usb_port:=2-2.2.1.3
```
You should be able to see 
- robot side arms return to $\vec{0}$ 
- 3 cameras in rviz
- `Wrist_Left` shows left wrist (`/realsense_left/color/image_raw`)
- `Wrist_Right` shows right wrist (`/realsense_right/color/image_raw`)
- `Overhead` shows top camera (`/realsense_top/color/image_raw`)

Terminal 3 teleop side
```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
roslaunch teleop_setup start_teleop_all.launch \
  left_can_port:=can_ml right_can_port:=can_mr \
  enable_paddle:=false \
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
  gravity_joint_scale_left:="[1.0, 0.96, 0.97, 0.95, 1.0, 1.0]" \
  gravity_joint_scale_right:="[1.0, 0.96, 0.97, 0.95, 1.0, 1.0]" \
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

## Intervention Data Collection

## MUX switch

robot -> policy
rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd


teleop -> follow
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh follow


teleop -> free
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh teleop

robot -> follow teleop
rosservice call /robot/arm_left/joint_cmd_mux_select /teleop/arm_left/joint_states_single
rosservice call /robot/arm_right/joint_cmd_mux_select /teleop/arm_right/joint_states_single

## Record ACT Rosbag Episodes

Terminal 4 recorder
```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard.py
```

Default behavior (no extra args):
- profile: `workspaces/config/rosbag_profiles/act_rgb_profile.yaml`
- camera topics: `/realsense_left`, `/realsense_right`, `/realsense_top`
- keyboard: `SPACE` start/stop episode, `Q` safe quit

## convert rosbag to lerobot
```bash
source /opt/ros/noetic/setup.bash
cd /home/jameszhao2004/training_codebase

pipeline/scripts/convert_session.sh \
  --session-dir /home/jameszhao2004/catkin_ws/data/rosbags/act_20260222_210318 \
  --dataset-id act_20260222_210318_v21_fps30 \
  --fps 30 \
  --min-frames 32 \
  --task "bimanual teleop"
```


## run policy
```bash
```
