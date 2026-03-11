> Scratch notes only. Canonical docs are:
> `/home/jameszhao2004/catkin_ws/DOCS.md`
> `/home/jameszhao2004/catkin_ws/RUNBOOK_3ARM_INTERVENTION.md`

bash can_activate.sh can_ml 1000000 1-7.4.3:1.0
bash can_activate.sh can_mr 1000000 1-7.4.4:1.0
bash can_activate.sh can_sl 1000000 1-7.4.2.4:1.0
bash can_activate.sh can_sr 1000000 1-7.4.2.3:1.0
bash can_activate.sh can_opp 1000000 1-7.4.2.2:1.0

# rviz = true

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


# teleop 

source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
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



source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/opp_master_switch.py \
  --gravity-switch-align-lock-pre-hold-sec 0.18 \
  --gravity-switch-calm-max-sec 1.2 \
  --attach-switch-post-settle-sec 0.25
