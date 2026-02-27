> [!WARNING]
> Deprecated workflow note (2026-02-23): do not use catkin_ws as a training entrypoint.
> Canonical training + policy storage is `/home/jameszhao2004/training_codebase/outputs/train`.
> Preferred commands:
> - `bash /home/jameszhao2004/training_codebase/pipeline/scripts/trainctl.sh start ...`
> - `bash /home/jameszhao2004/training_codebase/pipeline/scripts/trainctl.sh policy --run-name <run_name> --step best`

## switch

robot controlled by policy
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


## videos in rosbag -> mp4
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/unpack_bag_videos.py --bag /home/jameszhao2004/catkin_ws/data/rosbags/act_20260224_182749/episode_060/episode.bag --output-dir /home/jameszhao2004/catkin_ws/data/rosbags/act_20260224_182749/episode_060/episode_videos_mp4  --codec mp4v   --container mp4


## train history (DEPRECATED: do not run here; use training_codebase/pipeline/scripts/trainctl.sh)
source /home/jameszhao2004/catkin_ws/.venv_train_act/bin/activate

DATASET_ID=act_20260222_210318_v21_fps30
DATASET_PARENT=/home/jameszhao2004/training_codebase/data/lerobot
RUN_NAME=act_20260222_210318_chunk100_obs1_100k_bs8_amp
OUT_DIR=/home/jameszhao2004/training_codebase/outputs/train/${RUN_NAME}

# HISTORICAL COMMAND (DO NOT RUN): replaced by training_codebase/pipeline/scripts/trainctl.sh
CUDA_VISIBLE_DEVICES=0 lerobot-train \
  --dataset.repo_id ${DATASET_ID} \
  --dataset.root ${DATASET_PARENT}/${DATASET_ID} \
  --dataset.video_backend torchcodec \
  --policy.type act \
  --policy.device cuda \
  --policy.repo_id local/${RUN_NAME} \
  --policy.push_to_hub false \
  --policy.use_amp true \
  --policy.n_obs_steps 1 \
  --policy.chunk_size 100 \
  --policy.n_action_steps 100 \
  --steps 100000 \
  --save_freq 10000 \
  --log_freq 100 \
  --batch_size 8 \
  --num_workers 4 \
  --wandb.mode disabled \
  --output_dir ${OUT_DIR} \
  --resume true

# run policy:

CKPT_STEP=088000
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_act_checkpoint_ros.py \
  --checkpoint-dir ${OUT_ROOT}/${RUN_NAME}/checkpoints/${CKPT_STEP}/pretrained_model/ \
  --device cuda \
  --rate 30 \
  --temporal-ensemble-coeff 0.002 \
  --guard-profile medium \
  --debug-streams

# Train (ACT, chunk=100, n_action_steps=100, n_obs_steps=1)
source /home/jameszhao2004/catkin_ws/.venv_train_act/bin/activate

# Upgrade an existing local v2.1 dataset to v3.0 in-place (required by lerobot>=v3 format)
source /home/jameszhao2004/catkin_ws/.venv_train_act/bin/activate

python -m lerobot.datasets.v30.convert_dataset_v21_to_v30 \
  --repo-id jinhe \
  --root /home/jameszhao2004/training_codebase/data/lerobot \
  --push-to-hub false

# HISTORICAL COMMAND (DO NOT RUN): replaced by training_codebase/pipeline/scripts/trainctl.sh
CUDA_VISIBLE_DEVICES=0 lerobot-train \
  --dataset.repo_id act_20260220_220456_v21_fps30 \
  --dataset.root /home/jameszhao2004/catkin_ws/data/lerobot/act_20260220_220456_v21_fps30 \
  --dataset.video_backend torchcodec \
  --policy.type act \
  --policy.device cuda \
  --policy.repo_id local/act_20260218_192753_chunk60_obs1_100k_bs4_amp \
  --policy.push_to_hub false \
  --policy.use_amp true \
  --policy.n_obs_steps 1 \
  --policy.chunk_size 100 \
  --policy.n_action_steps 100 \
  --steps 100000 \
  --save_freq 10000 \
  --log_freq 100 \
  --batch_size 8 \
  --num_workers 4 \
  --wandb.mode disabled \
  --output_dir /home/jameszhao2004/training_codebase/outputs/train/act_20260218_192753_chunk60_obs1_100k_bs4_amp


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

# load policy

## robot controlled by policy
rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd

OUT_ROOT=/home/jameszhao2004/training_codebase/outputs/train
RUN_NAME=towel_folding_260224_act_v30_fps30_local_20260225_034326_chunk80
CKPT_STEP=030000

python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_act_checkpoint_ros.py \
  --checkpoint-dir ${OUT_ROOT}/${RUN_NAME}/checkpoints/${CKPT_STEP}/pretrained_model/ \
  --device cuda \
  --rate 30 \
  --temporal-ensemble-coeff 0.01 \
  --guard-profile medium \
  --debug-streams


# replay

# Terminal B (replay one episode to real arms)
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh

SESSION=/home/jameszhao2004/catkin_ws/data/rosbags/act_20260220_220456
EP=30
printf -v EP3 "%03d" "$EP"
BAG="${SESSION}/episode_${EP3}/episode.bag"

rosbag play "$BAG" --pause --rate 1.0 --topics \
  /teleop/arm_left/joint_states_single \
  /teleop/arm_right/joint_states_single \
  /teleop/arm_left/joint_states_single:=/robot/arm_left/vla_joint_cmd \
  /teleop/arm_right/joint_states_single:=/robot/arm_right/vla_joint_cmd









