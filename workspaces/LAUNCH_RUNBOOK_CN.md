# Zeno Teleop 启动手册

这是实验室使用的运行手册，用于在角色隔离的 Catkin 工作空间中启动
robot 侧与 teleop 侧。

## 1) 实验室 CAN 命名

本实验室使用以下接口命名：

- Robot 左臂：`can_sl`
- Robot 右臂：`can_sr`
- Teleop 左臂：`can_ml`
- Teleop 右臂：`can_mr`

检查当前 CAN 接口映射：

```bash
bash /home/jameszhao2004/catkin_ws/find_all_can_port.sh
```

## 2) 工作空间规则

- Robot 角色必须使用 `workspaces/ws_robot`。
- Teleop 角色必须使用 `workspaces/ws_teleop`。
- 同一个 shell 中不要同时 source 两个角色工作空间。
- 每个角色都建议从全新终端启动。

角色 source 脚本：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
```

## 3) 一次性构建

先安装必需的 ROS 系统包（`dm_hw` 依赖 `serial`）：

```bash
sudo apt-get update
sudo apt-get install -y ros-noetic-serial
```

如果你使用 Docker 别名 `ros1`，请把 `ros-noetic-serial` 加到
`ros1-piper` 的 Dockerfile 安装列表里，并重新构建镜像。

每个隔离工作空间至少构建一次（或代码更新后重新构建）：

```bash
/home/jameszhao2004/catkin_ws/workspaces/scripts/build_robot_ws.sh
/home/jameszhao2004/catkin_ws/workspaces/scripts/build_teleop_ws.sh
```

## 4) 日常启动流程

打开两个终端。

当前集成的模型默认值：

- `robot_description` 使用 `piper_x_description/urdf/piper_x_description.urdf`。
- 重力补偿默认使用 `piper_x_description/urdf/piper_x_description.urdf`。

终端 A（robot 侧）：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  enable_cameras:=true enable_rviz:=true enable_handeye_tf:=true enable_fisheye:=false \
  camera_left_usb_port:=2-2.2.1.4 camera_right_usb_port:=2-2.2.1.2 camera_top_usb_port:=2-2.2.1.3

```

终端 B（teleop 侧）：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
roslaunch teleop_setup start_teleop_all.launch \
  left_can_port:=can_ml right_can_port:=can_mr \
  enable_paddle:=false \
  master_slave_enable:=true mit_enable_tor:=false \
  mit_enable_pos:=true \
  gravity_mix_mode:=additive
```

如需启用你之前的 `tor` 反馈 profile（可在稳定基线可用后再切）：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
roslaunch teleop_setup start_teleop_all.launch \
  left_can_port:=can_ml right_can_port:=can_mr \
  enable_paddle:=false \
  master_slave_enable:=true mit_enable_tor:=true \
  mit_enable_pos:=true \
  mit_max_torque_abs:=8.0 \
  mit_torque_scale_left:="[0.9, 0.18, 0.2, 0.18, 0.15, 0.20]" \
  mit_torque_scale_right:="[0.9, 0.18, 0.2, 0.18, 0.15, 0.20]" \
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
```

teleop 可选参数：

- 关闭 paddle/haptic：`enable_paddle:=false`
- 关闭重力补偿：`enable_gravity_compensation:=false`
- 稳定模式（默认，建议先验证）：`mit_enable_tor:=false gravity_mix_mode:=additive`
- 启用扭矩反馈（tor，建议先小增益）：  
  `mit_enable_tor:=true gravity_mix_mode:=additive mit_max_torque_abs:=3.0 mit_torque_scale_left:="[0.12, 0.08, 0.12, 0.10, 0.08, 0.10]" mit_torque_scale_right:="[0.12, 0.08, 0.12, 0.10, 0.08, 0.10]" mit_torque_feedback_sign_left:="[-1, -1, -1, -1, -1, -1]" mit_torque_feedback_sign_right:="[-1, -1, -1, -1, -1, -1]"`
- 退出 launch 锁定当前位置（默认）：`shutdown_mode:=hold`  
  可调重试：`shutdown_hold_retries:=5 shutdown_hold_interval_sec:=0.05`
- 若你明确要退出时下电：`shutdown_mode:=disable`（或旧参数 `disable_on_shutdown:=true`）
- 注意：`mit_enable_tor:=true` 时不要用 `gravity_mix_mode:=gravity_only`，否则 tor 通道会被覆盖，看不到力反馈
- 使用旧重力模型：`gravity_urdf_package:=piper_description gravity_urdf_relpath:=urdf/piper_description.urdf`
- 旧版重力缩放（J1-3 降为 1/4）：`gravity_joint_scale:="[0.25, 0.25, 0.25, 1.0, 1.0, 1.0]"`
- J3 反向修正（双臂）：`gravity_joint_scale_left:="[1.0, 1.0, -1.0, 1.0, 1.0, 1.0]" gravity_joint_scale_right:="[1.0, 1.0, -1.0, 1.0, 1.0, 1.0]"`

模型回退参数（robot + teleop 均可用）：

- 使用旧 `robot_description`：`robot_description_file:=$(find piper_description)/urdf/piper_description.xacro`

## 5) 相机启动变体

如果需要相机链路，使用 robot 启动并开启 cameras：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  enable_cameras:=true enable_rviz:=true enable_handeye_tf:=true enable_fisheye:=false \
  camera_left_usb_port:=2-2.2.1.4 camera_right_usb_port:=2-2.2.1.2 camera_top_usb_port:=2-2.2.1.3
```

当前配置中的相机行为：

- 左/右手腕相机：开启 Infra1 流。
- 顶部相机：开启 RGB 色彩流。
- 关闭 depth/pointcloud。
- 本实验室端口映射：
  左手腕 D405 -> `2-2.2.1.4`，右手腕 D405 -> `2-2.2.1.2`，顶部 D435 -> `2-2.2.1.3`。

先检查 USB 端口映射：

```bash
bash /home/jameszhao2004/catkin_ws/find_all_camera_port.sh
```

## 6) 快速健康检查

启动后执行：

```bash
rosparam get /piper_ctrl_left_node/can_port
rosparam get /piper_ctrl_right_node/can_port
rosparam get /piper_teleop_left_node/can_port
rosparam get /piper_teleop_right_node/can_port
```

期望值：

- Robot：`can_sl`、`can_sr`
- Teleop：`can_ml`、`can_mr`

检查 gripper reverse 配置（本实验室应关闭）：

```bash
rosparam get /piper_teleop_left_node/gripper_reverse
rosparam get /piper_teleop_right_node/gripper_reverse
```

期望值：

- `false`
- `false`

若启用了扭矩反馈（`mit_enable_tor:=true`），额外检查：

```bash
rosparam get /piper_teleop_left_node/mit/enable_tor
rosparam get /piper_teleop_right_node/mit/enable_tor
rosparam get /piper_teleop_left_node/mit/gravity_mix_mode
rosparam get /piper_teleop_right_node/mit/gravity_mix_mode
rosparam get /piper_teleop_left_node/mit/torque_scale
rosparam get /piper_teleop_right_node/mit/torque_scale
rosparam get /piper_teleop_left_node/mit/torque_feedback_sign
rosparam get /piper_teleop_right_node/mit/torque_feedback_sign
```

期望值（示例）：

- `enable_tor`: `true`, `true`
- `gravity_mix_mode`: `additive`, `additive`
- `torque_scale`: 与启动参数一致（如 `[0.12, 0.08, 0.12, 0.10, 0.08, 0.10]`）
- `torque_feedback_sign`: 推荐为负号（如 `[-1, -1, -1, -1, -1, -1]`），用于碰撞时反向反馈

确认角色包解析路径：

```bash
rospack find piper_ctrl
rospack find piper_x_description
```

期望：

- 在 robot 终端：路径位于 `workspaces/ws_robot/src/...`
- 在 teleop 终端：路径位于 `workspaces/ws_teleop/src/...`

## 7) 常见问题与处理

构建时报重复包错误：

- 现象：`Multiple packages found with the same name ...`
- 处理：使用新终端，只 source 一个角色脚本。

CAN socket 名称错误：

- 现象：`CAN socket can_left does not exist`
- 处理：按第 4 节显式传入 launch 参数，然后重新检查 rosparam。

没有 ROS master：

- 现象：`ERROR: Unable to communicate with master!`
- 处理：确认已有 `roslaunch` 在运行，或先启动 `roscore`。

缺少 fisheye 包：

- 现象：`Resource not found: v4l2_cam_launch`
- 处理：保持 `enable_fisheye:=false`（当前 robot launch 默认值）。

缺少 serial 包：

- 现象：`Could not find a package configuration file provided by "serial"`
- 处理：安装 `ros-noetic-serial` 后重新构建：
  `sudo apt-get update && sudo apt-get install -y ros-noetic-serial`

RealSense USB 被占用：

- 现象：`RS2_USB_STATUS_BUSY` 或 `failed to set power state`
- 处理：关闭其他占用相机的进程，必要时重插相机并重新启动。

## 8) 关键配置文件

- Robot 主启动文件：
  `src/zeno-wholebody-intervation/zeno-wholebody-robot/robot_side/robot_setup/launch/start_robot_all.launch`
- Teleop 主启动文件：
  `src/zeno-wholebody-intervation/zeno-wholebody-teleop/teleop_side/teleop_setup/launch/start_teleop_all.launch`
- Robot/teleop 参数映射（robot 副本）：
  `src/zeno-wholebody-intervation/zeno-wholebody-robot/common/piper_ctrl/config/piper_dual.yaml`
- Robot/teleop 参数映射（teleop 副本）：
  `src/zeno-wholebody-intervation/zeno-wholebody-teleop/common/piper_ctrl/config/piper_dual.yaml`

## 9) ACT 键盘录包（SPACE/Q）

先按第 4 节启动 robot + teleop，然后在第三个终端执行：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard.py
```

该命令使用默认话题配置（`workspaces/config/rosbag_profiles/act_rgb_profile.yaml`），适配当前这套相机命名，建议直接使用。

可选参数示例：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard.py \
  --prefix act --notes "pick_place_trial"
```

保留调试日志（默认不保留）：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_act_keyboard.py \
  --keep-debug-logs
```

键盘交互：

- `SPACE`：开始/结束当前 episode
- `Q`：安全退出（若在录制会先停止并写入 metadata）

默认行为：

- 每个 episode 独立 bag
- 会话目录：`<prefix>_<YYYYMMDD>_<HHMMSS>`
- episode 编号：`episode_001`、`episode_002`（自动递增，避免覆盖）
- 压缩：`--lz4`
- split：默认关闭；需要时用 `--split --split-size-mb 512`
- 输出目录：`/home/jameszhao2004/catkin_ws/data/rosbags`
- 话题配置文件：`workspaces/config/rosbag_profiles/act_rgb_profile.yaml`
- 调试日志：默认自动清理 `rosbag_record.log` 和 `hz_*.log`
  （需要排障时用 `--keep-debug-logs` 保留）

metadata（每个 episode）：

- 文件：`metadata.json`
- 内容：开始/结束时间、stop reason、实际录制话题、可选话题缺失、bag 统计、hz 统计、关键 rosparam 快照

## 10) Gripper 数据说明（录包）

当前系统没有独立 gripper state 话题。gripper 数据在 `JointState` 第 7 维（索引 `6`）：

- `position[6]`：gripper 开合位置
- `effort[6]`：gripper 力反馈/受力

主要观测话题：

- `/robot/arm_left/joint_states_single`
- `/robot/arm_right/joint_states_single`
- `/teleop/arm_left/joint_states_single`
- `/teleop/arm_right/joint_states_single`

说明：

- ACT profile 默认录 RGB 三相机 + 双臂 joint/gripper/pose + 核心上下文话题
- 不录 `/tf`、`/tf_static`
- 不录 paddle 相关话题（实验室当前不使用）

## 11) Mux 切换 Demo（含 Fake Policy，可直接复制）

本节用于演示以下两类切换：

- Robot 控制输入切换（`teleop` <-> `vla_joint_cmd`）
- Teleop 主从模式切换（`follow` <-> `teleop`）

### 11.1 终端分工（推荐 5 个终端）

- `T1`：robot 启动终端
- `T2`：teleop 启动终端
- `T3`：切换控制终端（mux/service/flag）
- `T4`：fake policy 发布终端
- `T5`：观测终端

### 11.2 启动命令

`T1`（robot）：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
roslaunch robot_setup start_robot_all.launch \
  left_can_port:=can_sl right_can_port:=can_sr \
  enable_cameras:=false enable_rviz:=false enable_handeye_tf:=false
```

`T2`（teleop）：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_teleop.sh
roslaunch teleop_setup start_teleop_all.launch \
  left_can_port:=can_ml right_can_port:=can_mr \
  enable_paddle:=false \
  master_slave_enable:=true mit_enable_tor:=false mit_enable_pos:=true \
  mit_max_torque_abs:=8.0 \
  gravity_mix_mode:=gravity_only \
  gravity_unwrap_joint_positions:=true \
  gravity_max_joint_step:=0.08 \
  gravity_max_torque_delta_warn:=2.0 \
  enforce_joint_limits:=true \
  gravity_joint_scale_left:="[1.0, 1.0, 0.9, 0.79, 0.6, 1.0]" \
  gravity_joint_scale_right:="[1.0, 1.0, 0.9, 0.79, 0.6, 1.0]" \
  gripper_haptic_effort_sign:=-1.0 \
  gripper_haptic_effort_deadband:=0.10 \
  gripper_haptic_effort_bias:=0.05 \
  gripper_haptic_effort_max:=0.55 \
  gripper_haptic_cmd_max:=450 \
  gripper_haptic_cmd_enable_threshold:=90 \
  gripper_haptic_effort_alpha:=0.35 \
  gripper_haptic_release_when_opening:=true \
  gripper_haptic_opening_relief:=0.08 \
  gripper_haptic_opening_sign:=1.0 \
  gripper_haptic_opening_direction_eps:=0.0002
```

### 11.3 上线自检（`T3`）

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
rosservice list | grep -E "joint_cmd_mux_select|enable_srv"
rostopic list | grep -E "slave_follow_flag|joint_cmd_mux"
rosparam get /piper_teleop_left_node/master_slave/enable
rosparam get /piper_teleop_right_node/master_slave/enable
```

期望：

- 存在 `/robot/arm_left/joint_cmd_mux_select`、`/robot/arm_right/joint_cmd_mux_select`
- 存在 `/conrft_robot/slave_follow_flag`
- `master_slave/enable` 为 `true`、`true`

### 11.4 演示 A：Teleop 控制 Robot（基线模式）

将 robot 输入切到 teleop：

```bash
rosservice call /robot/arm_left/joint_cmd_mux_select /teleop/arm_left/joint_states_single
rosservice call /robot/arm_right/joint_cmd_mux_select /teleop/arm_right/joint_states_single
```

设置 teleop 为自由模式：

```bash
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh teleop
```

预期：

- 手动移动 teleop 臂时，robot 臂同步运动
- `T2` 可见 `Switched to TELEOP mode`（仅在状态变化时打印）

### 11.5 演示 B：Fake Policy 控制 Robot

`T3` 切 mux 到 fake policy 输入：

```bash
rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd
```

`T4` 启动 fake policy 循环发布器：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/fake_policy_loop.py
```

`T5` 观测 mux 输出：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
rostopic echo /robot/arm_left/joint_cmd_mux
```

预期：

- `/robot/arm_left/joint_cmd_mux` 的 `position` 在 A/B 之间平滑往返（A->B 约 2 秒，可用 `--hold-sec` 调整）
- 此时移动 teleop 臂，robot 不再跟随 teleop（因为输入已切到 `vla_joint_cmd`）

### 11.6 演示 C：Robot 走 Fake Policy，Teleop 跟随 Robot

在演示 B 基础上，设置 teleop follow：

```bash
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh follow
rostopic echo -n 1 /conrft_robot/slave_follow_flag
```

预期：

- 输出 `data: true`
- `T2` 出现 `Switched to SLAVE FOLLOW mode`（仅在状态变化时打印）
- 机械上看到 teleop 臂跟随 robot 臂（robot 由 fake policy 驱动）

### 11.7 Demo 结束后恢复到常规遥操作

1) 停止 fake policy（在 `T4` 按 `Ctrl+C`）  
2) 切 teleop 为自由模式：

```bash
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh teleop
```

3) 切 robot 输入回 teleop：

```bash
rosservice call /robot/arm_left/joint_cmd_mux_select /teleop/arm_left/joint_states_single
rosservice call /robot/arm_right/joint_cmd_mux_select /teleop/arm_right/joint_states_single
```

预期：

- 恢复为 teleop 控制 robot

### 11.8 常见错误（Demo 时高频）

- `ERROR: Unable to load type [piper_msgs/Enable]`
  - 原因：当前终端没 source robot 工作空间
  - 处理：`source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh`

- `rostopic echo -n 1 /conrft_robot/slave_follow_flag` 无输出
  - 原因：错过 `--once` 的发布窗口
  - 处理：先开持续 `rostopic echo /conrft_robot/slave_follow_flag`，再执行 `set_teleop_mode.sh`

## 12) ACT Checkpoint 在线推理（仅发布，不做切换）

本节新增脚本：

- `workspaces/scripts/run_act_checkpoint_ros.py`

设计约束（重要）：

- 仅发布策略命令到 robot 侧话题：
  - `/robot/arm_left/vla_joint_cmd`
  - `/robot/arm_right/vla_joint_cmd`
- 不调用 `joint_cmd_mux_select` service
- 不发布 `/conrft_robot/slave_follow_flag`
- 不切换 teleop/follow 模式

### 12.1 前置条件

1) robot + teleop 栈已经按本 runbook 启动  
2) 推理环境可用（建议在 ROS1 Docker 内使用 Python>=3.10 的 venv/uv venv）  
3) 已有 checkpoint 目录（包含 `config.json` 和 `model.safetensors`）

### 12.2 启动策略发布节点

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
source /home/jameszhao2004/catkin_ws/.venv_train_act/bin/activate

python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_act_checkpoint_ros.py \
  --checkpoint-dir /home/jameszhao2004/catkin_ws/outputs/train/act_20260218_smoke/checkpoints/000200/pretrained_model \
  --device cuda \
  --rate 20 \
  --temporal-ensemble-coeff 0.01 \
  --guard-profile medium
```

可选参数：

- `--disable-gripper`：禁用策略对夹爪的控制
- `--robot-left-topic` / `--robot-right-topic`：覆盖状态输入话题
- `--top-camera-topic` / `--left-wrist-topic` / `--right-wrist-topic`：覆盖图像输入话题
- `--out-left-topic` / `--out-right-topic`：覆盖发布目标话题

### 12.3 外部切换流程（由你或其他脚本负责）

策略节点启动后，是否“接管 robot”由外部 mux 切换决定：

切到策略输入：

```bash
rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd
```

切回 teleop 输入：

```bash
rosservice call /robot/arm_left/joint_cmd_mux_select /teleop/arm_left/joint_states_single
rosservice call /robot/arm_right/joint_cmd_mux_select /teleop/arm_right/joint_states_single
```

teleop follow 模式同样由外部控制：

```bash
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh follow
bash /home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh teleop
```

### 12.4 运行期安全行为

节点内置发布侧保护（不改系统模式）：

- `guard-profile=medium` 默认启用（平滑 + 步进限幅 + 夹爪范围约束）
- 任一输入流缺失/过旧/时间戳不同步时，暂停发布
- 输入恢复后自动继续发布

即使出现异常，节点也不会自动切 mux、不会自动改 follow 标志。

### 12.5 已验证记录（2026-02-18）与快速排障

**2026-02-18** 实测结论：策略发布链路已可完整跑通。

验证步骤：

1) 启动策略节点并保持运行：

```bash
source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
source /home/jameszhao2004/catkin_ws/.venv_train_act/bin/activate
python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_act_checkpoint_ros.py \
  --checkpoint-dir /home/jameszhao2004/catkin_ws/outputs/train/act_20260218_smoke/checkpoints/000200/pretrained_model \
  --device cuda \
  --rate 20 \
  --temporal-ensemble-coeff 0.01 \
  --guard-profile medium
```

2) 检查发布频率：

```bash
rostopic hz /robot/arm_left/vla_joint_cmd
rostopic hz /robot/arm_right/vla_joint_cmd
```

期望：接近 `--rate`（默认 20Hz）。

3) 需要接管 robot 时，手动切 mux：

```bash
rosservice call /robot/arm_left/joint_cmd_mux_select /robot/arm_left/vla_joint_cmd
rosservice call /robot/arm_right/joint_cmd_mux_select /robot/arm_right/vla_joint_cmd
```

本次排障记录（高频）：

- 若 venv 内报 `ModuleNotFoundError: rospkg`：
  - 在同一 venv 安装一次：
  - `python -m pip install rospkg catkin_pkg`
- 若节点启动后“没有继续打印日志”：
  - 健康状态下这是正常现象（无等待/错误日志）
  - 用 `rostopic hz /robot/arm_*/vla_joint_cmd` 判断是否在持续发布
- 若卡在等待输入：
  - 用 `--debug-streams`
  - 看 `raw/valid/drop` 计数，快速区分订阅问题、图像解码问题、维度过滤问题。
