# NOTE

This file is kept as an operator scratchpad.
Canonical guidebooks are now:

- `/home/jameszhao2004/catkin_ws/WORKSPACES.md`
- `/home/jameszhao2004/catkin_ws/LAUNCH_RUNBOOK.md`
- `/home/jameszhao2004/catkin_ws/RUNBOOK_3ARM_INTERVENTION.md`


# Runbook v1.0

## Activate CAN&Camera

#### CAN
```
cd /home/jameszhao2004/catkin_ws
```

- can_sl = 1-2.2.4:1.0
- can_sr = 1-2.2.3:1.0
- can_opp= 1-2.2.2:1.0
- can_ml = 1-2.3:1.0
- can_mr = 1-2.4:1.0

```
bash find_all_can_port.sh
```

```
bash can_activate.sh can_ml 1000000 1-2.3:1.0
bash can_activate.sh can_mr 1000000 1-2.4:1.0
bash can_activate.sh can_sl 1000000 1-2.2.4:1.0
bash can_activate.sh can_sr 1000000 1-2.2.3:1.0
bash can_activate.sh can_opp 1000000 1-2.2.2:1.0
```

#### Camera
```
bash find_all_camera_port.sh
```
- wrist_cam_r = 2-2.2.1.2
- wrist_cam_l = 2-2.2.1.4
- overhead_cam = 2-2.2.1.3
- face_cam = 1-2.1

## Start Teleop System

#### Terminal 1 roscore
```bash
ros1
roscore
```

#### Terminal 2 robot node
```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  enable_opp_arm:=false \
  enable_cameras:=true enable_opp_camera:=false \
  enable_camera_left:=true enable_camera_right:=true enable_camera_top:=true \
  enable_rviz:=true enable_handeye_tf:=false enable_fisheye:=false \
  camera_initial_reset:=false \
  camera_left_serial_no:=352122273242 \
  camera_right_serial_no:=352122273590 \
  camera_top_serial_no:=348522075799

```

source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  enable_opp_arm:=false \
  enable_cameras:=true enable_rviz:=true enable_handeye_tf:=false enable_fisheye:=false \
  camera_left_usb_port:=2-3.2 camera_right_usb_port:=2-3.4 camera_top_usb_port:=2-3.3

You should be able to see 
- robot side arms return to $\vec{0}$ 
- 3 cameras in rviz
- `Wrist_Left` shows left wrist (`/realsense_left/color/image_raw`)
- `Wrist_Right` shows right wrist (`/realsense_right/color/image_raw`)
- `Overhead` shows top camera (`/realsense_top/color/image_raw`)

#### Terminal 3 teleop side
```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
rospack find teleop_setup
roslaunch /home/jameszhao2004/catkin_ws/src/zeno-wholebody-intervation/zeno-wholebody-teleop/teleop_side/teleop_setup/launch/start_teleop_all.launch \
  left_can_port:=can_ml right_can_port:=can_mr \
  enable_paddle:=false \
  enable_opp_arm:=false \
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

`rospack find teleop_setup` expected path:
- `/home/jameszhao2004/catkin_ws/src/zeno-wholebody-intervation/zeno-wholebody-teleop/teleop_side/teleop_setup`

#### Terminal 4 opp master switch (keyboard)
```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
rosparam get /piper_gravity_compensation_node/enable_opp_arm
rostopic echo -n1 /robot/arm_opp/joint_states_compensated
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/opp_master_switch.py
# optional: customize integrated recording session directory name
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/opp_master_switch.py --session-name my_session_001
```

Keyboard:
- `1`: `arm_opp` follows `/teleop/arm_left/joint_states_single`
- `2`: `arm_opp` follows `/teleop/arm_right/joint_states_single`
- `h`: `arm_opp` hold current position
- `q`: quit switcher

`arm_opp` defaults to hold after startup. Press `1` or `2` to attach it to a master arm.
Integrated recorder output root defaults to `/home/jameszhao2004/catkin_ws/data/rosbags`.

#### Record ACT Rosbag Episodes

Terminal 5 recorder
```bash
ros1
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard_intervention.py \
  --profile /home/jameszhao2004/catkin_ws/workspaces/config/rosbag_profiles/act_rgb_3arm_profile.yaml
```

python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard.py

Default behavior (no extra args):
- profile: `workspaces/config/rosbag_profiles/act_rgb_profile.yaml`
- camera topics: `/realsense_left`, `/realsense_right`, `/realsense_top`
- keyboard: `SPACE` start/stop episode, `Q` safe quit


## Intervention Data Collection

source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh

**robot -> policy**
```bash
rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd
```

**teleop -> follow**
```
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh follow
```

**teleop -> free**
```
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh teleop
```

**robot -> follow teleop**
```
rosservice call /robot/arm_left/joint_cmd_mux_select /teleop/arm_left/joint_states_single
rosservice call /robot/arm_right/joint_cmd_mux_select /teleop/arm_right/joint_states_single
```

## Convert rosbag to lerobot
```bash
source /opt/ros/noetic/setup.bash
cd /home/jameszhao2004/training_codebase

pipeline/scripts/convert_session.sh \
  --session-dir /home/jameszhao2004/catkin_ws/data/rosbags/jinhe \
  --dataset-id jinhe \
  --fps 30 \
  --min-frames 32 \
  --task "bimanual teleop"
```

### 3arm conversion (default: keep full state+image window, action hold-last)
```bash
cd /home/jameszhao2004/training_codebase
pipeline/scripts/convert_session_3arm.sh \
  --session-dir /home/jameszhao2004/catkin_ws/data/rosbags/3arm_open_bag \
  --dataset-id 3arm_open_bag_v30_fps30_full \
  --fps 30 \
  --dataset-version v30
```

### 3arm conversion (legacy strict-drop behavior)
```bash
cd /home/jameszhao2004/training_codebase
pipeline/scripts/convert_session_3arm.sh \
  --session-dir /home/jameszhao2004/catkin_ws/data/rosbags/3arm_open_bag \
  --dataset-id 3arm_open_bag_v30_fps30_strict \
  --fps 30 \
  --dataset-version v30 \
  -- \
  --action-fill-policy strict_drop \
  --teleop-left-topic /teleop/arm_left/joint_states_single \
  --teleop-right-topic /teleop/arm_right/joint_states_single \
  --opp-action-topic /robot/arm_opp/joint_cmd_mux
```

## Run Policy
```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
source /home/jameszhao2004/catkin_ws/.venv_train_act/bin/activate

rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd

OUT_ROOT=/home/jameszhao2004/training_codebase/outputs/train
RUN_NAME=Simulation_lyj
CKPT_STEP=080000

OUT_ROOT=/home/jameszhao2004/training_codebase/outputs/train
RUN_NAME=pick_place_out_of_view_new_40_v30_fps30_local_20260303_034747
CKPT_STEP=060000

python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_act_checkpoint_ros.py \
  --checkpoint-dir ${OUT_ROOT}/${RUN_NAME}/checkpoints/${CKPT_STEP}/pretrained_model/ \
  --device cuda \
  --rate 15 \
  --temporal-ensemble-coeff 0.01 \
  --guard-profile medium \
  --debug-streams
```

# Intervention Data Extract and Merge

```bash
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/extract_teleop_segments.py \
  --session-dir /home/jameszhao2004/catkin_ws/data/rosbags/test_intervention_001 \
  --output-root /home/jameszhao2004/catkin_ws/data/rosbags/test_intervention_001_teleop_only \
  --overwrite

python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/merge_rosbag_sessions.py \
  --demo-session-dir /home/jameszhao2004/catkin_ws/data/rosbags/towel_folding_260224_act \
  --teleop-session-dir /home/jameszhao2004/catkin_ws/data/rosbags/test_intervention_001_teleop_only \
  --output-root /home/jameszhao2004/catkin_ws/data/rosbags/towel_folding_260224_act_mix_teleop \
  --overwrite
```

**Monica attention Map**
```
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_act_checkpoint_ros_attnmap.py \
  --checkpoint-dir ${OUT_ROOT}/${RUN_NAME}/checkpoints/${CKPT_STEP}/pretrained_model/ \
  --device cuda \
  --rate 30 \
  --temporal-ensemble-coeff 0.01 \
  --guard-profile medium \
  --debug-streams
```


source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
source /home/jameszhao2004/catkin_ws/.venv_train_act/bin/activate

rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd

cd /home/jameszhao2004/training_codebase && \
source .venv/bin/activate && \
RUN_DIR=$(ls -dt outputs/train/burst_towel_20260228_phasehf_mg014_hn028_hq010_chunk24* | head -n1) && \
CKPT=$(ls -dt "$RUN_DIR"/checkpoints/*/pretrained_model | head -n1) && \
echo "Using checkpoint: $CKPT" && \
python3 pipeline/scripts/run_act_checkpoint_ros_modeaware.py \
  --checkpoint-dir "$CKPT" \
  --device cuda \
  --rate 30 \
  --guard-profile medium \
  --temporal-ensemble-coeff 0.001 \
  --fast-mode on \
  --fast-detector joint_speed \
  --fast-enter-thresh 1.2 \
  --fast-exit-thresh 0.9 \
  --fast-metric-ema-alpha 0.35 \
  --mode-min-dwell-steps 5 \
  --k-slow 1 \
  --k-fast 1 \
  --exec-steps-fast 1 \
  --disable-ensemble-fast \
  --enable-fast-guard \
  --fast-guard-alpha 1.0 \
  --fast-max-joint-step 0.08

python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard_intervention.py

python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard_intervention_switch_only.py

source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh

echo "[A] 只切 follow flag，不调 enable"
rostopic pub --once /conrft_robot/slave_follow_flag std_msgs/Bool "data: true"
sleep 1
rostopic pub --once /conrft_robot/slave_follow_flag std_msgs/Bool "data: false"

echo "[B] 只调 enable，不切 follow flag"
rosservice call /teleop/arm_left/enable_srv "enable_request: true"
rosservice call /teleop/arm_right/enable_srv "enable_request: true"


source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh

for t in \
  /realsense_left/color/camera_info \
  /realsense_right/color/camera_info \
  /realsense_top/color/camera_info
do
  echo "===== $t ====="
  rostopic echo -n1 "$t" | egrep "^(width|height|distortion_model|D:|K:|R:|P:)"
done
