> Scratch notes only. Canonical docs are:
> `/home/jameszhao2004/catkin_ws/DOCS.md`
> `/home/jameszhao2004/catkin_ws/RUNBOOK_3ARM_INTERVENTION.md`

# T1
ros1
roscore

# T2 robot
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr opp_can_port:=can_opp \
  enable_opp_arm:=true \
  enable_cameras:=false enable_rviz:=false enable_handeye_tf:=false enable_fisheye:=false

# T3 teleop
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
roslaunch /home/jameszhao2004/catkin_ws/src/zeno-wholebody-intervation/zeno-wholebody-teleop/teleop_side/teleop_setup/launch/start_teleop_all.launch \
  left_can_port:=can_ml right_can_port:=can_mr \
  enable_paddle:=false enable_opp_arm:=true

# 先做前置检查（T4）
rosservice list | rg "robot/arm_(left|right|opp)/joint_cmd_mux_select|teleop/arm_(left|right)/master_cmd_mux_select"
rostopic hz /robot/arm_opp/joint_states_single
rostopic hz /teleop/arm_left/joint_states_single
rostopic hz /teleop/arm_right/joint_states_single

# fake policy
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/intervention_3arm_switch_test.py \
  --policy-start-cmd "sleep infinity"

# 验证
rostopic echo -n1 /robot/arm_left/joint_cmd_mux/selected
rostopic echo -n1 /robot/arm_right/joint_cmd_mux/selected
rostopic echo -n1 /robot/arm_opp/joint_cmd_mux/selected
rostopic echo -n1 /conrft_robot/slave_follow_flag
ps -ef | rg "sleep infinity|intervention_3arm_switch_test.py" | rg -v rg


/home/jameszhao2004/training_codebase/pipeline/scripts/launch_policy_ros_3arm.sh \
  --run-name 3arm_open_bag_3arm_v30_fps30_local_20260309_001503 \
  --step 040000 \
  --robot-env-script /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh \
  -- --device cuda --rate 30 --temporal-ensemble-coeff 0.01 --guard-profile medium


rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd
rosservice call /robot/arm_opp/joint_cmd_mux_select /robot/arm_opp/vla_joint_cmd


#### set to 0
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_gripper_open_then_zero_ros_3arm.py \
  --rate 30 \
  --open-duration-sec 1 \
  --zero-duration-sec 2 \
  --out-left-topic /robot/arm_left/vla_joint_cmd \
  --out-right-topic /robot/arm_right/vla_joint_cmd \
  --out-opp-topic /robot/arm_opp/vla_joint_cmd \
  --done-mode hold \
  --debug-streams

## replay

source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh

rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd
rosservice call /robot/arm_opp/joint_cmd_mux_select /robot/arm_opp/vla_joint_cmd

python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/replay_rosbag_3arm_vla.py \
  --bag /home/jameszhao2004/Downloads/episode_027-20260310T060705Z-1-001/episode_027/episode.bag \
  --rate 0.5 \
  --debug


python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/estimate_camera_origin_from_apriltag_video.py \
  --video-path /path/to/overhead.mp4 \
  --tag-id 0 \
  --tag-size-m 0.04 \
  --camera-info-yaml /home/jameszhao2004/catkin_ws/workspaces/config/camera_info/realsense_top_color_camera_info.yaml \
  --origin-tag-position 0,0.4,0 \
  --origin-tag-yaw-deg 0


python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/replay_rosbag_2arm_vla.py \
  --bag /home/jameszhao2004/catkin_ws/data/rosbags/converted_lerobot_debug/episode_000/episode.bag \
  --rate 1.0


## Wrist Hand-Eye Calibration Summary (2026-03-10)

### Result files
- left: `/home/jameszhao2004/catkin_ws/src/zeno-wholebody-intervation/zeno-wholebody-robot/robot_side/cam_calibration/data/samples/left/handeye_axxb_20260310_224414.json`
- right: `/home/jameszhao2004/catkin_ws/src/zeno-wholebody-intervation/zeno-wholebody-robot/robot_side/cam_calibration/data/samples/right/handeye_axxb_20260310_230810.json`

### Pose results (gripper_base -> camera_optical_frame)
| side | parent -> child | samples | t (m) [x, y, z] | q (xyzw) | euler xyz (deg) |
|---|---|---:|---|---|---|
| left | `arm_left/gripper_base -> realsense_left_color_optical_frame` | 30 | `[-0.01020, -0.09698, 0.04891]` | `[-0.18729, 0.00624, -0.02959, 0.98184]` | `[-21.6009, 0.0669, -3.4653]` |
| right | `arm_right/gripper_base -> realsense_right_color_optical_frame` | 30 | `[-0.00460, -0.08867, 0.04095]` | `[-0.17516, -0.00219, -0.00136, 0.98454]` | `[-20.1763, -0.2744, -0.1095]` |

### Consistency metrics
| side | all_pass | position std xyz (m) | translation residual median (m) | orientation error median (deg) |
|---|---|---|---:|---:|
| left | `false` | `[0.00468, 0.00478, 0.00983]` | `0.00849` | `2.5050` |
| right | `true` | `[0.00490, 0.00771, 0.00758]` | `0.01094` | `1.2376` |

### Conclusion
- right 结果达标（`all_pass=true`），可直接用于 TF 发布。
- left 结果基本可用，但旋转中位误差略超阈值（2.505° > 2.0°），建议补采大姿态样本后再重算一版。
