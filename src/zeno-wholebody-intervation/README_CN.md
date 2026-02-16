# Zeno Wholebody Intervention

[English](README.md)

双臂遥操作系统的控制切换框架，支持多控制源无缝切换和主从臂模式动态切换。

## 特性

- **多控制源切换 (Robot端)**: 通过 ROS Mux 实现遥操作、策略推理、自主控制等多种输入源的无缝切换
- **主从模式切换 (Teleop端)**: 通过动态调整 MIT 控制参数实现从臂跟随/自由移动模式切换
- **双臂支持**: 同时支持左右双臂独立控制
- **实时切换**: 运行时动态切换，无需重启节点

## 项目结构

```
zeno-wholebody-intervation/
├── zeno-wholebody-robot/          # Robot端 (被控端)
│   └── common/
│       └── piper_ctrl/
│           ├── config/
│           │   └── piper_dual.yaml    # Robot端配置
│           └── scripts/
│               └── piper_ctrl_node.py
│
└── zeno-wholebody-teleop/         # Teleop端 (操作端)
    └── common/
        └── piper_ctrl/
            ├── config/
            │   └── piper_dual.yaml    # Teleop端配置
            └── scripts/
                └── piper_ctrl_node.py
```

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Control Flow                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌─────────────┐         ┌─────────────┐         ┌─────────────┐           │
│   │   Teleop    │ ──────> │     Mux     │ ──────> │    Robot    │           │
│   │    Arm      │         │  (切换器)    │         │     Arm     │           │
│   └─────────────┘         └─────────────┘         └─────────────┘           │
│         │                       ↑                       │                    │
│         │                       │                       │                    │
│         │                 ┌─────────────┐               │                    │
│         │                 │   Policy    │               │                    │
│         │                 │  Inference  │               │                    │
│         │                 └─────────────┘               │                    │
│         │                                               │                    │
│         └───────────────────────────────────────────────┘                    │
│                    slave_follow_flag (主从切换)                               │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Part 1: Robot端 - 多控制源切换

### 概述

Robot端通过 ROS `topic_tools/mux` 实现多个控制源之间的切换。将关节命令话题重映射到 Mux，即可通过服务调用切换不同的输入源。

### 配置

配置文件: `zeno-wholebody-robot/common/piper_ctrl/config/piper_dual.yaml`

```yaml
# 左臂配置
piper_ctrl_left_node:
  remap:
    joint_pos_cmd_to: /robot/arm_left/joint_cmd_mux
    gripper_pos_cmd_to: /robot/arm_left/joint_cmd_mux

# 右臂配置
piper_ctrl_right_node:
  remap:
    joint_pos_cmd_to: /robot/arm_right/joint_cmd_mux
    gripper_pos_cmd_to: /robot/arm_right/joint_cmd_mux
```

### 使用方法

#### Python 示例

```python
#!/usr/bin/env python
import rospy
from topic_tools.srv import MuxSelect

class ArmMuxController:
    def __init__(self):
        rospy.wait_for_service('/robot/arm_right/joint_cmd_mux/select')
        rospy.wait_for_service('/robot/arm_left/joint_cmd_mux/select')

        self.mux_right = rospy.ServiceProxy('/robot/arm_right/joint_cmd_mux/select', MuxSelect)
        self.mux_left = rospy.ServiceProxy('/robot/arm_left/joint_cmd_mux/select', MuxSelect)

    def switch_to_teleop(self):
        """切换到遥操作模式"""
        self.mux_right('/teleop/arm_right/joint_pos_cmd')
        self.mux_left('/teleop/arm_left/joint_pos_cmd')

    def switch_to_policy(self):
        """切换到策略推理模式"""
        self.mux_right('/policy/arm_right/joint_pos_cmd')
        self.mux_left('/policy/arm_left/joint_pos_cmd')

if __name__ == '__main__':
    rospy.init_node('mux_controller')
    controller = ArmMuxController()
    controller.switch_to_teleop()
```

#### 命令行

```bash
# 切换到遥操作输入
rosservice call /robot/arm_right/joint_cmd_mux/select "/teleop/arm_right/joint_pos_cmd"

# 切换到策略输入
rosservice call /robot/arm_left/joint_cmd_mux/select "/policy/arm_left/joint_pos_cmd"

# 查看当前选中的话题
rostopic echo /robot/arm_right/joint_cmd_mux/selected -n 1
```

### 架构图

```
                    ┌─────────────────────┐
 /teleop/.../cmd ──>│                     │
                    │   joint_cmd_mux     │──> piper_ctrl_node ──> Robot Arm
 /policy/.../cmd ──>│   (topic_tools)     │
                    │                     │
 /auto/.../cmd   ──>│                     │
                    └─────────────────────┘
                            │
                            v
                    MuxSelect Service
```

---

## Part 2: Teleop端 - 主从模式切换

### 概述

Teleop端支持**主从切换**功能，通过一个 Bool 话题控制从臂（teleop arm）的工作模式：
- **跟随模式** (`True`): 从臂跟随主臂位置
- **遥操作模式** (`False`): 从臂自由移动，用于数据采集

核心原理：通过动态切换 MIT 控制模式的 **kp 参数**实现不同刚度控制。

### 配置

配置文件: `zeno-wholebody-teleop/common/piper_ctrl/config/piper_dual.yaml`

```yaml
# 左臂配置
piper_teleop_left_node:
  master_slave:
    enable: true
    master_position_topic: "/robot/arm_left/joint_states_single"  # 主臂位置
    master_flag_topic: "/conrft_robot/slave_follow_flag"          # 切换信号
    kp_follow: [7.0, 7.0, 7.0, 7.0, 7.0, 7.0]                     # 跟随模式kp

# 右臂配置
piper_teleop_right_node:
  master_slave:
    enable: true
    master_position_topic: "/robot/arm_right/joint_states_single"
    master_flag_topic: "/conrft_robot/slave_follow_flag"
    kp_follow: [7.0, 7.0, 7.0, 7.0, 7.0, 7.0]
```

### 工作原理

| 模式 | `slave_follow_flag` | kp 值 | 行为 |
|------|---------------------|-------|------|
| 遥操作模式 | `False` | 低 (如 0.5) | 从臂自由移动，用于采集数据 |
| 跟随模式 | `True` | 高 (如 7.0) | 从臂跟随主臂位置 |

**核心代码逻辑** (`piper_ctrl_node.py`):

```python
def master_flag_callback(self, msg: Bool):
    with self.slave_follow_mode_lock:
        if msg.data != self.slave_follow_mode:
            self.slave_follow_mode = msg.data
            if self.slave_follow_mode:
                self.mit_kp = self.mit_kp_follow[:]      # 高kp，跟随主臂
            else:
                self.mit_kp = self.mit_kp_teleop[:]     # 低kp，自由移动
```

### 使用方法

#### Python 示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

class TeleopModeController:
    def __init__(self):
        self.pub = rospy.Publisher('/conrft_robot/slave_follow_flag', Bool, queue_size=1)

    def set_follow_mode(self):
        """跟随模式 - 从臂跟随主臂"""
        self.pub.publish(Bool(data=True))

    def set_teleop_mode(self):
        """遥操作模式 - 从臂自由移动"""
        self.pub.publish(Bool(data=False))

if __name__ == '__main__':
    rospy.init_node('teleop_mode_controller')
    controller = TeleopModeController()

    # 数据采集流程
    controller.set_teleop_mode()   # 1. 进入遥操作模式
    # ... 采集数据 ...
    controller.set_follow_mode()   # 2. 回到跟随模式
```

#### 命令行

```bash
# 切换到跟随模式
rostopic pub /conrft_robot/slave_follow_flag std_msgs/Bool "data: true" -1

# 切换到遥操作模式
rostopic pub /conrft_robot/slave_follow_flag std_msgs/Bool "data: false" -1
```

### 架构图

```
                                ┌──────────────────────────────────┐
                                │        Teleop Arm Node           │
                                │      (piper_ctrl_node.py)        │
                                │                                  │
/robot/arm_xxx/joint_states ────┼──> master_position_callback()    │
       (主臂位置)                │           │                      │
                                │           v                      │
/conrft_robot/slave_follow_flag ┼──> master_flag_callback()        │
       (Bool)                   │           │                      │
                                │           v                      │
                                │   ┌───────────────────┐          │
                                │   │ slave_follow_mode │          │
                                │   └─────────┬─────────┘          │
                                │        True │ False              │
                                │             v                    │
                                │      kp=7.0 / kp=0.5             │
                                │             │                    │
                                │             v                    │
                                │      MIT Control Loop            │
                                └──────────────────────────────────┘
```

---

## 典型使用场景

### 场景 1: 遥操作数据采集

```
1. rostopic pub /conrft_robot/slave_follow_flag std_msgs/Bool "data: false" -1
   → Teleop臂进入自由移动模式

2. 操作员移动Teleop臂，Robot臂跟随执行任务

3. 记录轨迹数据

4. rostopic pub /conrft_robot/slave_follow_flag std_msgs/Bool "data: true" -1
   → Teleop臂回到跟随模式
```

### 场景 2: 策略推理 + 安全接管

```
1. rosservice call /robot/arm_right/joint_cmd_mux/select "/policy/arm_right/joint_pos_cmd"
   → Robot臂切换到策略控制

2. 策略推理控制Robot臂执行任务

3. 检测到异常，需要人工接管:
   rosservice call /robot/arm_right/joint_cmd_mux/select "/teleop/arm_right/joint_pos_cmd"
   → Robot臂切换回遥操作控制

4. 操作员通过Teleop臂安全接管
```

---

## 话题与服务参考

### Robot端

| 类型 | 名称 | 说明 |
|------|------|------|
| Service | `/robot/arm_left/joint_cmd_mux/select` | 左臂Mux切换服务 |
| Service | `/robot/arm_right/joint_cmd_mux/select` | 右臂Mux切换服务 |
| Topic | `/robot/arm_xxx/joint_cmd_mux/selected` | 当前选中的输入话题 |

### Teleop端

| 类型 | 名称 | 说明 |
|------|------|------|
| Topic | `/conrft_robot/slave_follow_flag` | 主从切换信号 (Bool) |
| Topic | `/robot/arm_xxx/joint_states_single` | 主臂位置反馈 |
| Topic | `/teleop/arm_xxx/joint_states_single` | 从臂位置反馈 |

---

## 依赖

- ROS Noetic / Melodic
- `topic_tools` (用于 Mux)
- `piper_sdk`
- `piper_msgs`

## License

MIT License
