# Piper Robotic Arm Teleop System

A ROS 1 Noetic-based teleoperation system for Agilex Piper X robotic arms using a Docker/rocker setup on Ubuntu 24.04.

## Table of Contents

- [Documentation Guide](#documentation-guide)
- [System Overview](#system-overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Architecture](#software-architecture)
- [Installation](#installation)
- [Usage](#usage)
- [Data Collection](#data-collection)
- [Troubleshooting](#troubleshooting)
- [File Structure](#file-structure)

---

## Documentation Guide

Use this section as the top-level document router for this repository.

### Start Here

- Full documentation map and maintenance rules:
  `/home/jameszhao2004/catkin_ws/DOCS.md`
- Workspace and role-isolation overview:
  `/home/jameszhao2004/catkin_ws/WORKSPACES.md`
- Daily launch baseline (English):
  `/home/jameszhao2004/catkin_ws/LAUNCH_RUNBOOK.md`
- Daily launch baseline (Chinese):
  `/home/jameszhao2004/catkin_ws/LAUNCH_RUNBOOK_CN.md`
- 3-arm intervention workflow (OPP switch + recorder):
  `/home/jameszhao2004/catkin_ws/RUNBOOK_3ARM_INTERVENTION.md`

### Which Guide Should I Use?

| Your task | Primary guide |
|---|---|
| Build/source isolated robot and teleop workspaces | `WORKSPACES.md` |
| Standard robot + teleop bring-up and troubleshooting | `LAUNCH_RUNBOOK.md` |
| 3-arm sessions with `arm_opp` and `opp_master_switch.py` | `RUNBOOK_3ARM_INTERVENTION.md` |
| Documentation ownership and where to edit | `DOCS.md` |
| Burst monitor pipeline operations | `README_burst_monitor.md` |
| Docker/host environment setup | `DOCKER_SETUP_README.md` |

### 3-Arm Critical Rule

For 3-arm sessions, verify these before running `opp_master_switch.py`:

- `/piper_gravity_compensation_node/enable_opp_arm` is `true`
- `/robot/arm_opp/joint_states_compensated` is publishing

---

## System Overview

This system enables **bilateral teleoperation** where 2 master arms control 2 slave arms in real-time, while recording synchronized data (joint states + camera) for imitation learning.

```
┌─────────────────────────────────────────────────────────────┐
│                        HOST (Ubuntu 24.04)                  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │              Docker Container (Ubuntu 20.04)          │  │
│  │                    ROS 1 Noetic                       │  │
│  │  ┌─────────────┐    ┌─────────────┐    ┌──────────┐  │  │
│  │  │ Master Arms │───▶│   Teleop    │───▶│  Slave   │  │  │
│  │  │  (can_mr,   │    │    Node     │    │   Arms   │  │  │
│  │  │   can_ml)   │    └─────────────┘    │ (can_sr, │  │  │
│  │  └─────────────┘                       │  can_sl) │  │  │
│  │         ▲                              └──────────┘  │  │
│  │         │              ┌──────────┐         │        │  │
│  │         └──────────────│  rosbag  │◀────────┘        │  │
│  │                        │  record  │                  │  │
│  │  ┌─────────────┐       └────┬─────┘                  │  │
│  │  │  RealSense  │────────────┘                        │  │
│  │  │    D435     │                                     │  │
│  │  └─────────────┘                                     │  │
│  └───────────────────────────────────────────────────────┘  │
│                              ▲                              │
│                              │ --network=host               │
│                              │ --privileged                 │
│  ┌───────────────────────────┴───────────────────────────┐  │
│  │                    CAN Interfaces                     │  │
│  │         can_sr, can_sl, can_mr, can_ml               │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### Why Docker?

**Problem:** ROS 1 Noetic requires Ubuntu 20.04, but modern hardware (Intel Raptor Lake) needs Ubuntu 22.04+ for WiFi/driver support.

**Solution:** Run ROS 1 in a Docker container while using Ubuntu 24.04 as the host OS.

### Why Rocker?

[Rocker](https://github.com/osrf/rocker) is a tool that simplifies running Docker containers with:
- NVIDIA GPU passthrough (for rviz, visualization)
- X11 forwarding (GUI applications)
- User permissions (no root issues)
- Home directory mounting (persistent workspace)

---

## Hardware Requirements

### Robotic Arms
- **4x Agilex Piper X** robotic arms
  - 2 Master arms (for human input)
  - 2 Slave arms (follows master)
- **4x USB-to-CAN adapters** (one per arm)

### Camera
- **Intel RealSense D435** (RGB + Depth)
- **USB 3.0 cable required** (USB 2.0 will fail!)

### Computer
- Ubuntu 24.04 host
- NVIDIA GPU (recommended for visualization)
- 4+ USB ports (or USB hub)

### USB Port Mapping (Example)
```
USB Port 1-7.1 → can_sr (Slave Right)
USB Port 1-7.2 → can_sl (Slave Left)
USB Port 1-7.3 → can_mr (Master Right)
USB Port 1-7.4 → can_ml (Master Left)
```

> **Note:** Your USB port addresses may differ. Use `bash find_all_can_port.sh` to identify them.

---

## Software Architecture

### Host System (Ubuntu 24.04)
- Manages CAN interfaces
- Runs Docker/rocker
- USB device passthrough

### Docker Container (Ubuntu 20.04)
- ROS 1 Noetic
- piper_sdk (arm control)
- realsense2_camera (camera driver)
- Custom teleop package

### Key ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/master_right/joint_states_single` | sensor_msgs/JointState | Master right arm joint positions (input) |
| `/master_left/joint_states_single` | sensor_msgs/JointState | Master left arm joint positions (input) |
| `/slave_right/joint_ctrl_single` | sensor_msgs/JointState | Slave right arm control commands |
| `/slave_left/joint_ctrl_single` | sensor_msgs/JointState | Slave left arm control commands |
| `/camera/color/image_raw/compressed` | sensor_msgs/CompressedImage | RGB camera feed |
| `/camera/aligned_depth_to_color/image_raw/compressedDepth` | sensor_msgs/CompressedImage | Depth camera feed |

### JointState Message Format
```yaml
name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
position: [7 values in radians, gripper in meters]
velocity: [6 zeros + speed_percentage]  # velocity[6] = 0-100% speed
effort: [6 zeros + gripper_force]       # effort[6] = 0.5-2.0 N
```

> **Important:** The Piper driver uses `velocity[6]` as the overall arm speed percentage (0-100), not gripper velocity!

---

## Installation

### 1. Install Docker and Rocker (Host)

```bash
# Install Docker
sudo apt update
sudo apt install -y docker.io
sudo usermod -aG docker $USER
newgrp docker

# Install rocker via pip
pip install rocker

# Install NVIDIA Container Toolkit (for GPU support)
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### 2. Install CAN Tools (Host)

```bash
sudo apt install -y ethtool can-utils
```

### 3. Create Custom Docker Image

Create `~/ros1_docker/Dockerfile`:

```dockerfile
FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
    iproute2 \
    can-utils \
    ethtool \
    git \
    python3-pip \
    ros-noetic-serial \
    ros-noetic-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install python-can piper_sdk
```

Build the image:

```bash
cd ~/ros1_docker
docker build -t ros1-piper .
```

### 4. Create Shell Alias

Add to `~/.bashrc` or `~/.zshrc`:

```bash
alias ros1='rocker --nvidia --x11 --user --home --network=host --privileged --volume /dev:/dev -- ros1-piper'
```

Reload:

```bash
source ~/.bashrc  # or source ~/.zshrc
```

### 5. Clone and Build piper_ros (Inside Docker)

```bash
ros1  # Enter container

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/agilexrobotics/piper_ros.git
cd piper_ros
git checkout noetic

# Skip MoveIt source build (use system version)
touch src/piper_moveit/moveit-1.1.11/CATKIN_IGNORE
touch src/piper_moveit/moveit_ctrl/CATKIN_IGNORE

cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 6. Create Teleop Package

```bash
mkdir -p ~/catkin_ws/src/piper_teleop/{launch,scripts}
```

Create `~/catkin_ws/src/piper_teleop/package.xml`:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>piper_teleop</name>
  <version>0.0.1</version>
  <description>Piper teleop package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>piper</exec_depend>
</package>
```

Create `~/catkin_ws/src/piper_teleop/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(piper_teleop)
find_package(catkin REQUIRED)
catkin_package()
```

Create `~/catkin_ws/src/piper_teleop/scripts/teleop_node.py`:

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

class PiperTeleop:
    def __init__(self):
        rospy.init_node('piper_teleop')
        
        # Parameters
        self.speed_percent = rospy.get_param('~speed', 100)  # 0-100
        self.gripper_effort = rospy.get_param('~gripper_effort', 1.0)  # 0.5-2.0 N
        
        # Publishers
        self.pub_sr = rospy.Publisher('/slave_right/joint_ctrl_single', JointState, queue_size=1)
        self.pub_sl = rospy.Publisher('/slave_left/joint_ctrl_single', JointState, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/master_right/joint_states_single', JointState, self.cb_mr, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/master_left/joint_states_single', JointState, self.cb_ml, queue_size=1, tcp_nodelay=True)
        
        rospy.loginfo(f"Teleop started - speed: {self.speed_percent}%")
        
    def make_cmd(self, msg):
        cmd = JointState()
        cmd.header.stamp = rospy.Time.now()
        cmd.name = msg.name
        cmd.position = list(msg.position)
        # Piper driver uses velocity[6] as speed percentage (0-100)
        cmd.velocity = [0.0]*6 + [float(self.speed_percent)]
        # effort[6] is gripper force (0.5-2.0 N)
        cmd.effort = [0.0]*6 + [self.gripper_effort]
        return cmd
        
    def cb_mr(self, msg):
        self.pub_sr.publish(self.make_cmd(msg))
        
    def cb_ml(self, msg):
        self.pub_sl.publish(self.make_cmd(msg))
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        teleop = PiperTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
```

Make executable:

```bash
chmod +x ~/catkin_ws/src/piper_teleop/scripts/teleop_node.py
```

Create `~/catkin_ws/src/piper_teleop/launch/multi_arm.launch`:

```xml
<launch>
    <group ns="slave_right">
        <node name="piper_node" pkg="piper" type="piper_ctrl_single_node.py" output="screen">
            <param name="can_port" value="can_sr"/>
            <param name="auto_enable" value="true"/>
            <param name="girpper_exist" value="true"/>
            <param name="gripper_val_mutiple" value="1"/>
        </node>
    </group>

    <group ns="slave_left">
        <node name="piper_node" pkg="piper" type="piper_ctrl_single_node.py" output="screen">
            <param name="can_port" value="can_sl"/>
            <param name="auto_enable" value="true"/>
            <param name="girpper_exist" value="true"/>
            <param name="gripper_val_mutiple" value="1"/>
        </node>
    </group>

    <group ns="master_right">
        <node name="piper_node" pkg="piper" type="piper_ctrl_single_node.py" output="screen">
            <param name="can_port" value="can_mr"/>
            <param name="auto_enable" value="true"/>
            <param name="girpper_exist" value="true"/>
            <param name="gripper_val_mutiple" value="1"/>
        </node>
    </group>

    <group ns="master_left">
        <node name="piper_node" pkg="piper" type="piper_ctrl_single_node.py" output="screen">
            <param name="can_port" value="can_ml"/>
            <param name="auto_enable" value="true"/>
            <param name="girpper_exist" value="true"/>
            <param name="gripper_val_mutiple" value="1"/>
        </node>
    </group>
</launch>
```

Create `~/catkin_ws/src/piper_teleop/launch/teleop_full.launch`:

```xml
<launch>
    <!-- 4 Arms -->
    <include file="$(find piper_teleop)/launch/multi_arm.launch"/>
    
    <!-- Overhead + wrist cameras -->
    <include file="$(find piper_teleop)/launch/multi_camera.launch">
        <arg name="wrist_left_device" value="/dev/video2"/>
        <arg name="wrist_right_device" value="/dev/video4"/>
    </include>
    
    <!-- Teleop -->
    <node pkg="piper_teleop" type="teleop_node.py" name="teleop" output="screen">
        <param name="speed" value="100"/>
        <param name="gripper_effort" value="1.0"/>
    </node>
</launch>
```

Create `~/catkin_ws/src/piper_teleop/launch/slave_only.launch`:

```xml
<launch>
    <group ns="slave_right">
        <node name="piper_node" pkg="piper" type="piper_ctrl_single_node.py" output="screen">
            <param name="can_port" value="can_sr"/>
            <param name="auto_enable" value="true"/>
            <param name="girpper_exist" value="true"/>
        </node>
    </group>

    <group ns="slave_left">
        <node name="piper_node" pkg="piper" type="piper_ctrl_single_node.py" output="screen">
            <param name="can_port" value="can_sl"/>
            <param name="auto_enable" value="true"/>
            <param name="girpper_exist" value="true"/>
        </node>
    </group>
</launch>
```

Rebuild:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Usage

### Step 1: Activate CAN Interfaces (Host)

Run on the **host** (not in Docker):

```bash
# Find your USB port addresses
bash find_all_can_port.sh
bash find_all_camera_port.sh

# Activate each CAN interface (adjust USB addresses as needed)
bash can_activate.sh can_sl 1000000 1-7.3:1.0   # Slave Left
bash can_activate.sh can_sr 1000000 1-7.4:1.0   # Slave Right
bash can_activate.sh can_ml 1000000 1-1.2:1.0   # Master Left
bash can_activate.sh can_mr 1000000 1-1.3:1.0   # Master Right

# Verify
ip link show type can
```

Expected output:
```
can_sr: <NOARP,UP,LOWER_UP,ECHO> ...
can_sl: <NOARP,UP,LOWER_UP,ECHO> ...
can_mr: <NOARP,UP,LOWER_UP,ECHO> ...
can_ml: <NOARP,UP,LOWER_UP,ECHO> ...
```

### Step 2: Enter Docker Container

```bash
ros1
```

### Step 3: Verify CAN Visibility (Inside Docker)

```bash
ip link show type can
```

Should show all 4 CAN interfaces.

### Step 4: Launch Teleop System

**Terminal 1 (Docker):**
```bash
ros1
# Prefer /dev/v4l/by-id/<camera> paths from find_all_camera_port.sh
roslaunch piper_teleop teleop_collect.launch \
    launch_preview:=true \
    wrist_left_device:=/dev/video2 \
    wrist_right_device:=/dev/video4
```

If your wrist cameras are RealSense units, launch them by serial instead:
```bash
roslaunch piper_teleop teleop_collect.launch \
    launch_preview:=true \
    wrist_left_use_realsense:=true \
    wrist_right_use_realsense:=true \
    wrist_left_serial_no:=<LEFT_RS_SERIAL> \
    wrist_right_serial_no:=<RIGHT_RS_SERIAL> \
    wrist_left_image_topic:=/wrist_left/color/image_raw \
    wrist_right_image_topic:=/wrist_right/color/image_raw
```

Wait for all 4 arms to show "使能状态: True"

**Terminal 2 (Docker):**
```bash
ros1
cd ~/teleop_data
rosbag record --lz4 \
    /master_right/joint_states_single \
    /master_left/joint_states_single \
    /slave_right/joint_states_single \
    /slave_left/joint_states_single \
    /camera/color/image_raw/compressed \
    /camera/aligned_depth_to_color/image_raw/compressedDepth \
    /camera/color/camera_info \
    /wrist_left/image_raw \
    /wrist_left/camera_info \
    /wrist_left/color/image_raw/compressed \
    /wrist_left/aligned_depth_to_color/image_raw/compressedDepth \
    /wrist_left/color/camera_info \
    /wrist_right/image_raw \
    /wrist_right/camera_info \
    /wrist_right/color/image_raw/compressed \
    /wrist_right/aligned_depth_to_color/image_raw/compressedDepth \
    /wrist_right/color/camera_info \
    -o demo
```

### Step 5: Perform Demonstration

Move the master arms - the slave arms will follow in real-time.

### Step 6: Stop Recording

Press `Ctrl+C` in both terminals.

---

## Data Collection

### Recorded Topics

| Topic | Rate | Description |
|-------|------|-------------|
| `*/joint_states_single` | 200 Hz | Joint positions, velocities, efforts |
| `/camera/color/image_raw/compressed` | 30 Hz | RGB images (JPEG compressed) |
| `/camera/aligned_depth_to_color/image_raw/compressedDepth` | 30 Hz | Depth images (aligned to RGB) |
| `/camera/color/camera_info` | 30 Hz | Camera intrinsics |
| `/wrist_left/image_raw` | 30 Hz | Left wrist RGB image |
| `/wrist_left/camera_info` | 30 Hz | Left wrist intrinsics |
| `/wrist_left/color/image_raw/compressed` | 30 Hz | Left wrist RGB (RealSense mode) |
| `/wrist_left/aligned_depth_to_color/image_raw/compressedDepth` | 30 Hz | Left wrist depth (RealSense mode) |
| `/wrist_left/color/camera_info` | 30 Hz | Left wrist intrinsics (RealSense mode) |
| `/wrist_right/image_raw` | 30 Hz | Right wrist RGB image |
| `/wrist_right/camera_info` | 30 Hz | Right wrist intrinsics |
| `/wrist_right/color/image_raw/compressed` | 30 Hz | Right wrist RGB (RealSense mode) |
| `/wrist_right/aligned_depth_to_color/image_raw/compressedDepth` | 30 Hz | Right wrist depth (RealSense mode) |
| `/wrist_right/color/camera_info` | 30 Hz | Right wrist intrinsics (RealSense mode) |

### Data Size

- **Uncompressed:** ~3.5 GB / 80 seconds
- **Compressed:** ~176 MB / 80 seconds (~20x smaller)

### Inspect Recorded Data

```bash
rosbag info ~/teleop_data/demo_*.bag
```

### Playback (Video Only)

```bash
# Terminal 1
roscore

# Terminal 2
rosbag play ~/teleop_data/demo_*.bag

# Terminal 3
rqt_image_view
# Then select /camera/color/image_raw/compressed from dropdown
```

### Playback (Physical Replay on Slave Arms)

```bash
# Terminal 1: Start only slave arms
roslaunch piper_teleop slave_only.launch

# Terminal 2: Replay master data → slave control
rosbag play ~/teleop_data/demo_*.bag \
    /master_right/joint_states_single:=/slave_right/joint_ctrl_single \
    /master_left/joint_states_single:=/slave_left/joint_ctrl_single
```

### Export Video from Rosbag

```python
#!/usr/bin/env python3
import rosbag
from cv_bridge import CvBridge
import cv2

bag = rosbag.Bag('demo.bag')
bridge = CvBridge()

out = None
for topic, msg, t in bag.read_messages(topics=['/camera/color/image_raw/compressed']):
    img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    if out is None:
        h, w = img.shape[:2]
        out = cv2.VideoWriter('demo.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (w, h))
    out.write(img)

out.release()
bag.close()
```

---

## Troubleshooting

### CAN Interface Not Found in Docker

**Symptom:** `ip link show type can` shows nothing in Docker

**Solution:** CAN must be activated on the **host** before entering Docker:
```bash
# On HOST (not in Docker)
bash can_activate.sh can_sr 1000000 1-7.1:1.0
```

### "使能状态: False" Timeout

**Symptom:** Arm fails to enable after 5 seconds

**Causes:**
1. CAN not activated
2. Wrong CAN port name
3. Arm not powered on
4. Loose CAN cable connection

**Solutions:**
```bash
# Check CAN is up
ip link show type can

# Check CAN communication
candump can_sr  # Should show data if arm is on

# Check green terminal connectors are secure
```

### RealSense "RGB modules inconsistency" Error

**Symptom:** Camera fails to initialize

**Cause:** USB 2.0 cable (insufficient bandwidth)

**Solution:** Use USB 3.0 cable. Verify:
```bash
lsusb -t | grep -i 5000  # Should show 5000M for RealSense
```

### Teleop is Janky/Slow

**Symptom:** Slave arms move in steps, not smooth

**Cause:** `velocity[6]` too low

**Solution:** Set speed to 100%:
```python
cmd.velocity = [0.0]*6 + [100.0]  # 100% speed
```

### "No module named 'piper_sdk'"

**Symptom:** Python import error

**Solution:** Install in Docker:
```bash
pip3 install piper_sdk python-can
```

Or rebuild Docker image with these in Dockerfile.

### Container Missing Tools (ip, can-utils)

**Symptom:** `bash: ip: command not found`

**Solution:** You're using the wrong Docker image. Use `ros1-piper` not `osrf/ros:noetic-desktop-full`:
```bash
alias ros1='rocker ... -- ros1-piper'  # NOT osrf/ros:noetic-desktop-full
```

Also ensure your `ros1-piper` image includes ROS package `ros-noetic-serial`
for `dm_hw` build dependency.

---

## File Structure

```
~/
├── ros1_docker/
│   └── Dockerfile                    # Custom Docker image
├── catkin_ws/
│   ├── find_all_camera_port.sh       # Find USB-camera mappings
│   └── src/
│       ├── piper_ros/                # Agilex official package
│       │   ├── can_activate.sh       # CAN activation script
│       │   ├── find_all_can_port.sh  # Find USB-CAN mappings
│       │   └── src/
│       │       └── piper/            # Arm driver
│       └── piper_teleop/             # Custom teleop package
│           ├── launch/
│           │   ├── multi_arm.launch      # Launch all 4 arms
│           │   ├── multi_camera.launch   # Overhead + wrist cameras
│           │   ├── teleop_full.launch    # Arms + cameras + teleop
│           │   ├── view_all_cameras.launch   # 3 live camera windows
│           │   ├── teleop_collect.launch # One-command bringup/preview/record
│           │   └── slave_only.launch     # Slave arms only (for replay)
│           └── scripts/
│               └── teleop_node.py        # Master→Slave mapping
└── teleop_data/                      # Recorded demonstrations
    └── demo_*.bag
```

---

## Quick Reference

```bash
# === HOST ===
# Optional: map camera devices to stable /dev/v4l/by-id links
bash find_all_camera_port.sh

# Activate CAN
bash can_activate.sh can_sl 1000000 1-7.3:1.0
bash can_activate.sh can_sr 1000000 1-7.4:1.0
bash can_activate.sh can_ml 1000000 1-1.2:1.0
bash can_activate.sh can_mr 1000000 1-1.3:1.0

# === DOCKER Terminal 1 ===
ros1
# Prefer /dev/v4l/by-id/<camera> paths from find_all_camera_port.sh
roslaunch piper_teleop teleop_collect.launch \
    launch_preview:=true \
    wrist_left_device:=/dev/video2 \
    wrist_right_device:=/dev/video4

# If wrist cameras are RealSense:
# roslaunch piper_teleop teleop_collect.launch \
#     launch_preview:=true \
#     wrist_left_use_realsense:=true \
#     wrist_right_use_realsense:=true \
#     wrist_left_serial_no:=<LEFT_RS_SERIAL> \
#     wrist_right_serial_no:=<RIGHT_RS_SERIAL> \
#     wrist_left_image_topic:=/wrist_left/color/image_raw \
#     wrist_right_image_topic:=/wrist_right/color/image_raw

# === DOCKER Terminal 2 ===
ros1
cd ~/teleop_data && rosbag record --lz4 \
    /master_right/joint_states_single \
    /master_left/joint_states_single \
    /slave_right/joint_states_single \
    /slave_left/joint_states_single \
    /camera/color/image_raw/compressed \
    /camera/aligned_depth_to_color/image_raw/compressedDepth \
    /camera/color/camera_info \
    /wrist_left/image_raw \
    /wrist_left/camera_info \
    /wrist_left/color/image_raw/compressed \
    /wrist_left/aligned_depth_to_color/image_raw/compressedDepth \
    /wrist_left/color/camera_info \
    /wrist_right/image_raw \
    /wrist_right/camera_info \
    /wrist_right/color/image_raw/compressed \
    /wrist_right/aligned_depth_to_color/image_raw/compressedDepth \
    /wrist_right/color/camera_info \
    -o demo
```

---

## License

MIT License
