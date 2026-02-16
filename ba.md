
## switch

source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh

robot -> fake policy
rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd


teleop -> follow
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh follow


teleop -> free
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh teleop

robot -> follow teleop
rosservice call /robot/arm_left/joint_cmd_mux_select /teleop/arm_left/joint_states_single
rosservice call /robot/arm_right/joint_cmd_mux_select /teleop/arm_right/joint_states_single

## forcefeedback

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
  gravity_joint_scale_left:="[1.0, 1.0, 1.0, 0.95, 1.0, 1.0]" \
  gravity_joint_scale_right:="[1.0, 1.0, 1.0, 0.95, 1.0, 1.0]" \
  gripper_haptic_effort_sign:=-1.0 \
  gripper_haptic_effort_deadband:=0.10 \
  gripper_haptic_effort_bias:=0.05 \
  gripper_haptic_effort_max:=0.55 \
  gripper_haptic_cmd_max:=450 \
  gripper_haptic_cmd_enable_threshold:=90 \
  gripper_haptic_effort_alpha:=0.35 \
  gripper_haptic_release_when_opening:=true \
  gripper_haptic_opening_relief:=0.08 \
  gripper_haptic_opening_sign:=-1 \
  gripper_haptic_opening_direction_eps:=0.0002