# Zeno Wholebody Intervention

[中文文档](README_CN.md)

A control switching framework for dual-arm teleoperation systems, supporting seamless multi-source control switching and dynamic master-slave mode transitions.

## Features

- **Multi-Source Control Switching (Robot Side)**: Seamless switching between teleoperation, policy inference, and autonomous control via ROS Mux
- **Master-Slave Mode Switching (Teleop Side)**: Dynamic MIT control parameter adjustment for slave arm follow/free-movement mode switching
- **Dual-Arm Support**: Independent control for both left and right arms
- **Real-Time Switching**: Dynamic switching at runtime without node restart

## Project Structure

```
zeno-wholebody-intervation/
├── zeno-wholebody-robot/          # Robot side (controlled end)
│   └── common/
│       └── piper_ctrl/
│           ├── config/
│           │   └── piper_dual.yaml    # Robot side config
│           └── scripts/
│               └── piper_ctrl_node.py
│
└── zeno-wholebody-teleop/         # Teleop side (operator end)
    └── common/
        └── piper_ctrl/
            ├── config/
            │   └── piper_dual.yaml    # Teleop side config
            └── scripts/
                └── piper_ctrl_node.py
```

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Control Flow                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌─────────────┐         ┌─────────────┐         ┌─────────────┐           │
│   │   Teleop    │ ──────> │     Mux     │ ──────> │    Robot    │           │
│   │    Arm      │         │  (Switcher) │         │     Arm     │           │
│   └─────────────┘         └─────────────┘         └─────────────┘           │
│         │                       ↑                       │                    │
│         │                       │                       │                    │
│         │                 ┌─────────────┐               │                    │
│         │                 │   Policy    │               │                    │
│         │                 │  Inference  │               │                    │
│         │                 └─────────────┘               │                    │
│         │                                               │                    │
│         └───────────────────────────────────────────────┘                    │
│                    slave_follow_flag (Master-Slave Switch)                   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Part 1: Robot Side - Multi-Source Control Switching

### Overview

The robot side uses ROS `topic_tools/mux` to switch between multiple control sources. By remapping joint command topics to Mux, different input sources can be switched via service calls.

### Configuration

Config file: `zeno-wholebody-robot/common/piper_ctrl/config/piper_dual.yaml`

```yaml
# Left arm config
piper_ctrl_left_node:
  remap:
    joint_pos_cmd_to: /robot/arm_left/joint_cmd_mux
    gripper_pos_cmd_to: /robot/arm_left/joint_cmd_mux

# Right arm config
piper_ctrl_right_node:
  remap:
    joint_pos_cmd_to: /robot/arm_right/joint_cmd_mux
    gripper_pos_cmd_to: /robot/arm_right/joint_cmd_mux
```

### Usage

#### Python Example

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
        """Switch to teleoperation mode"""
        self.mux_right('/teleop/arm_right/joint_pos_cmd')
        self.mux_left('/teleop/arm_left/joint_pos_cmd')

    def switch_to_policy(self):
        """Switch to policy inference mode"""
        self.mux_right('/policy/arm_right/joint_pos_cmd')
        self.mux_left('/policy/arm_left/joint_pos_cmd')

if __name__ == '__main__':
    rospy.init_node('mux_controller')
    controller = ArmMuxController()
    controller.switch_to_teleop()
```

#### Command Line

```bash
# Switch to teleoperation input
rosservice call /robot/arm_right/joint_cmd_mux/select "/teleop/arm_right/joint_pos_cmd"

# Switch to policy input
rosservice call /robot/arm_left/joint_cmd_mux/select "/policy/arm_left/joint_pos_cmd"

# Check currently selected topic
rostopic echo /robot/arm_right/joint_cmd_mux/selected -n 1
```

### Architecture Diagram

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

## Part 2: Teleop Side - Master-Slave Mode Switching

### Overview

The teleop side supports **master-slave switching** functionality, using a Bool topic to control the slave arm (teleop arm) operating mode:
- **Follow Mode** (`True`): Slave arm follows master arm position
- **Teleop Mode** (`False`): Slave arm moves freely for data collection

Core principle: Different stiffness control achieved by dynamically switching MIT control mode **kp parameters**.

### Configuration

Config file: `zeno-wholebody-teleop/common/piper_ctrl/config/piper_dual.yaml`

```yaml
# Left arm config
piper_teleop_left_node:
  master_slave:
    enable: true
    master_position_topic: "/robot/arm_left/joint_states_single"  # Master arm position
    master_flag_topic: "/conrft_robot/slave_follow_flag"          # Switch signal
    kp_follow: [7.0, 7.0, 7.0, 7.0, 7.0, 7.0]                     # Follow mode kp

# Right arm config
piper_teleop_right_node:
  master_slave:
    enable: true
    master_position_topic: "/robot/arm_right/joint_states_single"
    master_flag_topic: "/conrft_robot/slave_follow_flag"
    kp_follow: [7.0, 7.0, 7.0, 7.0, 7.0, 7.0]
```

### How It Works

| Mode | `slave_follow_flag` | kp Value | Behavior |
|------|---------------------|----------|----------|
| Teleop Mode | `False` | Low (e.g., 0.5) | Slave arm moves freely for data collection |
| Follow Mode | `True` | High (e.g., 7.0) | Slave arm follows master arm position |

**Core Code Logic** (`piper_ctrl_node.py`):

```python
def master_flag_callback(self, msg: Bool):
    with self.slave_follow_mode_lock:
        if msg.data != self.slave_follow_mode:
            self.slave_follow_mode = msg.data
            if self.slave_follow_mode:
                self.mit_kp = self.mit_kp_follow[:]      # High kp, follow master
            else:
                self.mit_kp = self.mit_kp_teleop[:]     # Low kp, free movement
```

### Usage

#### Python Example

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

class TeleopModeController:
    def __init__(self):
        self.pub = rospy.Publisher('/conrft_robot/slave_follow_flag', Bool, queue_size=1)

    def set_follow_mode(self):
        """Follow mode - slave arm follows master arm"""
        self.pub.publish(Bool(data=True))

    def set_teleop_mode(self):
        """Teleop mode - slave arm moves freely"""
        self.pub.publish(Bool(data=False))

if __name__ == '__main__':
    rospy.init_node('teleop_mode_controller')
    controller = TeleopModeController()

    # Data collection workflow
    controller.set_teleop_mode()   # 1. Enter teleop mode
    # ... collect data ...
    controller.set_follow_mode()   # 2. Return to follow mode
```

#### Command Line

```bash
# Switch to follow mode
rostopic pub /conrft_robot/slave_follow_flag std_msgs/Bool "data: true" -1

# Switch to teleop mode
rostopic pub /conrft_robot/slave_follow_flag std_msgs/Bool "data: false" -1
```

### Architecture Diagram

```
                                ┌──────────────────────────────────┐
                                │        Teleop Arm Node           │
                                │      (piper_ctrl_node.py)        │
                                │                                  │
/robot/arm_xxx/joint_states ────┼──> master_position_callback()    │
       (Master Position)        │           │                      │
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

## Typical Use Cases

### Scenario 1: Teleoperation Data Collection

```
1. rostopic pub /conrft_robot/slave_follow_flag std_msgs/Bool "data: false" -1
   → Teleop arm enters free movement mode

2. Operator moves teleop arm, robot arm follows to perform tasks

3. Record trajectory data

4. rostopic pub /conrft_robot/slave_follow_flag std_msgs/Bool "data: true" -1
   → Teleop arm returns to follow mode
```

### Scenario 2: Policy Inference + Safety Takeover

```
1. rosservice call /robot/arm_right/joint_cmd_mux/select "/policy/arm_right/joint_pos_cmd"
   → Robot arm switches to policy control

2. Policy inference controls robot arm to perform tasks

3. Anomaly detected, manual takeover needed:
   rosservice call /robot/arm_right/joint_cmd_mux/select "/teleop/arm_right/joint_pos_cmd"
   → Robot arm switches back to teleoperation control

4. Operator safely takes over via teleop arm
```

---

## Topics and Services Reference

### Robot Side

| Type | Name | Description |
|------|------|-------------|
| Service | `/robot/arm_left/joint_cmd_mux/select` | Left arm Mux switch service |
| Service | `/robot/arm_right/joint_cmd_mux/select` | Right arm Mux switch service |
| Topic | `/robot/arm_xxx/joint_cmd_mux/selected` | Currently selected input topic |

### Teleop Side

| Type | Name | Description |
|------|------|-------------|
| Topic | `/conrft_robot/slave_follow_flag` | Master-slave switch signal (Bool) |
| Topic | `/robot/arm_xxx/joint_states_single` | Master arm position feedback |
| Topic | `/teleop/arm_xxx/joint_states_single` | Slave arm position feedback |

---

## Dependencies

- ROS Noetic / Melodic
- `topic_tools` (for Mux)
- `piper_sdk`
- `piper_msgs`

## License

MIT License
