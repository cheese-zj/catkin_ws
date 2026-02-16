# Docker & Rocker Setup Guide for Piper Robot Arm

This guide explains the Docker-based development environment for the Piper robotic arm with ROS Noetic and MoveIt integration. It's designed for beginners who want to understand each component of the containerized setup.

---

## Table of Contents

1. [What is Docker?](#what-is-docker)
2. [What is Rocker?](#what-is-rocker)
3. [Project Overview](#project-overview)
4. [Docker Image Hierarchy](#docker-image-hierarchy)
5. [GUI Support with gui-docker](#gui-support-with-gui-docker)
6. [CAN Interface Setup](#can-interface-setup)
7. [Quick Start Guide](#quick-start-guide)
8. [Common Commands](#common-commands)
9. [Troubleshooting](#troubleshooting)

---

## What is Docker?

**Docker** is a platform that packages applications and their dependencies into isolated units called **containers**. Think of it like a lightweight virtual machine, but more efficient.

### Key Concepts for Beginners

| Term | Explanation |
|------|-------------|
| **Image** | A read-only template containing the OS, software, and configurations. Like a "snapshot" of a system. |
| **Container** | A running instance of an image. You can have multiple containers from the same image. |
| **Dockerfile** | A text file with instructions to build an image (like a recipe). |
| **Volume** | A way to share files between your host machine and a container. |
| **Build Context** | The files Docker can access when building an image. |

### Why Use Docker for Robotics?

1. **Reproducibility**: Everyone gets the exact same environment
2. **Isolation**: ROS dependencies won't conflict with your system
3. **Portability**: Works on any machine with Docker installed
4. **Easy Setup**: No manual installation of dozens of packages

---

## What is Rocker?

**Rocker** is a tool that extends Docker for robotics applications. It automatically handles:
- GPU acceleration (NVIDIA)
- Display forwarding (for RViz, Gazebo)
- Device access (cameras, sensors)
- User permissions

> **Note**: This project uses a custom `gui-docker` script instead of Rocker, which provides similar functionality for GUI applications and GPU support.

---

## Project Overview

### Workspace Structure

```
/home/jameszhao2004/catkin_ws/
├── build/                    # Compiled binaries (auto-generated)
├── devel/                    # Development workspace (auto-generated)
├── src/
│   └── piper_ros/           # Main project packages
│       ├── piper/           # Core robot control
│       ├── piper_description/   # URDF robot models
│       ├── piper_moveit/    # Motion planning with MoveIt
│       │   └── moveit-1.1.11/.docker/  # <-- Docker configuration lives here
│       ├── piper_msgs/      # Custom ROS messages
│       └── piper_sim/       # Gazebo & MuJoCo simulations
├── can_activate.sh          # CAN interface setup script
├── find_all_can_port.sh     # CAN port discovery script
└── DOCKER_SETUP_README.md   # This file
```

### What Each Package Does

| Package | Purpose |
|---------|---------|
| `piper` | Communicates with the physical robot arm via CAN bus |
| `piper_description` | Contains URDF/SDF files that describe the robot's physical structure |
| `piper_moveit` | Motion planning, collision avoidance, trajectory execution |
| `piper_msgs` | Custom message types for robot communication |
| `piper_sim` | Simulation environments (Gazebo for physics, MuJoCo for contacts) |

---

## Docker Image Hierarchy

The project includes **5 Dockerfiles** that build on each other in a hierarchy:

```
┌─────────────────────────────────────────────────────────────────┐
│                    ros:noetic-ros-base                          │
│                    (Official ROS base image)                    │
└─────────────────────────────────────────────────────────────────┘
                              │
         ┌────────────────────┴────────────────────┐
         │                                         │
         ▼                                         ▼
┌─────────────────────┐               ┌─────────────────────────┐
│    release/         │               │      ci/                │
│ (apt-get install)   │               │ (Build tools + deps)    │
│ Production-ready    │               │ For CI/CD pipelines     │
└─────────────────────┘               └─────────────────────────┘
                                                  │
                                                  ▼
                                      ┌─────────────────────────┐
                                      │    ci-testing/          │
                                      │ (ROS testing repos)     │
                                      │ Unstable package tests  │
                                      └─────────────────────────┘
                                                  │
                                                  ▼
                                      ┌─────────────────────────┐
                                      │    source/              │
                                      │ (Built from source)     │
                                      │ For development         │
                                      └─────────────────────────┘
                                                  │
                                                  ▼
                                      ┌─────────────────────────┐
                                      │    experimental/        │
                                      │ (MongoDB + warehouse)   │
                                      │ Trajectory storage      │
                                      └─────────────────────────┘
```

### Dockerfile Locations

All Dockerfiles are in: `piper_ros/src/piper_moveit/moveit-1.1.11/.docker/`

### Which Image Should You Use?

| Use Case | Recommended Image | Why |
|----------|-------------------|-----|
| Running demos | `moveit/moveit:noetic-release` | Pre-built, fast to download |
| Development | `moveit/moveit:noetic-source` | Full source code, editable |
| CI/CD | `moveit/moveit:noetic-ci` | All build tools included |
| Testing bleeding-edge | `moveit/moveit:noetic-ci-testing` | Latest ROS packages |
| Trajectory storage | `moveit/moveit:master-experimental` | MongoDB integration |

### Understanding Each Dockerfile

#### 1. Release Dockerfile (`release/Dockerfile`)

```dockerfile
# Simplest option - installs MoveIt from apt packages
FROM ros:noetic-ros-base
RUN apt-get install -y ros-noetic-moveit-*
```

**When to use**: Quick demos, learning MoveIt, no source modifications needed.

#### 2. CI Dockerfile (`ci/Dockerfile`)

```dockerfile
# Sets up all build dependencies
FROM ros:noetic-ros-base
RUN apt-get install -y \
    python3-catkin-tools \    # Better build system than catkin_make
    clang clang-format-10 \   # C++ compiler and formatter
    ccache                    # Compilation cache (speeds up rebuilds)
```

**When to use**: Automated testing, building from source.

#### 3. Source Dockerfile (`source/Dockerfile`)

```dockerfile
# Builds MoveIt from source code
FROM moveit/moveit:noetic-ci-testing
COPY . src/moveit
RUN catkin build
```

**When to use**: Development, debugging, modifying MoveIt code.

---

## GUI Support with gui-docker

The `gui-docker` script enables graphical applications (RViz, Gazebo) inside Docker containers.

### Location

```
piper_ros/src/piper_moveit/moveit-1.1.11/.docker/gui-docker
```

### How It Works

```
┌─────────────────────────────────────────────────────────────────┐
│                        Your Host System                         │
│  ┌──────────────┐                                               │
│  │   X Server   │ ◄──── X11 display server (handles windows)    │
│  └──────┬───────┘                                               │
│         │                                                       │
│         │ X11 socket (/tmp/.X11-unix)                          │
│         │                                                       │
│  ┌──────▼───────────────────────────────────────────────────┐  │
│  │                    Docker Container                       │  │
│  │  ┌─────────────┐                                          │  │
│  │  │    RViz     │ ◄──── Uses X11 to display windows       │  │
│  │  │   Gazebo    │                                          │  │
│  │  └─────────────┘                                          │  │
│  │                                                            │  │
│  │  Environment Variables:                                    │  │
│  │  - DISPLAY=$DISPLAY (which display to use)                │  │
│  │  - XAUTHORITY (permission to use X11)                     │  │
│  │                                                            │  │
│  │  Mounted Volumes:                                          │  │
│  │  - /tmp/.X11-unix (X11 socket)                            │  │
│  │  - GPU devices (if NVIDIA detected)                       │  │
│  └────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### GPU Detection Order

The script automatically detects and configures GPU support:

1. **NVIDIA Docker v2** (preferred) - Uses `--runtime=nvidia`
2. **NVIDIA Docker v1** (legacy) - Uses `nvidia-docker` command
3. **DRI fallback** - Uses `/dev/dri` for Intel/AMD graphics
4. **No GPU** - Software rendering (slow but works)

### Basic Usage

```bash
# Navigate to the docker directory
cd piper_ros/src/piper_moveit/moveit-1.1.11/.docker/

# Run with defaults (creates persistent container)
./gui-docker

# Run a specific image (removed on exit)
./gui-docker --rm -it moveit/moveit:noetic-source /bin/bash

# Create a named persistent container
./gui-docker -c my_moveit_container

# Mount your workspace into the container
./gui-docker -v ~/catkin_ws:/root/catkin_ws --rm -it moveit/moveit:noetic-source bash
```

### Persistent vs Temporary Containers

| Type | Command | Behavior |
|------|---------|----------|
| Persistent | `./gui-docker -c mycontainer` | Keeps data between runs, re-enterable |
| Temporary | `./gui-docker --rm -it image bash` | Deleted when you exit |

### Re-entering a Persistent Container

```bash
# First terminal - create container
./gui-docker -c dev_container -it moveit/moveit:noetic-source bash

# Second terminal - enter same container
./gui-docker -c dev_container
```

---

## CAN Interface Setup

The Piper robot arm communicates over **CAN bus** (Controller Area Network). Before running the robot, you must configure the CAN interface.

### What is CAN Bus?

CAN bus is a communication protocol originally designed for automobiles. It allows multiple devices (like robot joints) to communicate on the same wire.

```
┌─────────────┐    CAN Bus    ┌─────────────┐
│  Computer   │◄─────────────►│ Robot Arm   │
│ (USB-CAN)   │   1 Mbps      │ (6 joints)  │
└─────────────┘               └─────────────┘
```

### Prerequisites

Install required packages on your host system:

```bash
sudo apt update
sudo apt install ethtool can-utils
```

### Using can_activate.sh

This script configures your USB-CAN adapter.

#### Basic Usage

```bash
# Default: configure as can0 at 1 Mbps
sudo bash can_activate.sh

# Custom name and bitrate
sudo bash can_activate.sh can0 1000000

# Specify USB port (when multiple adapters connected)
sudo bash can_activate.sh can0 1000000 1-2:1.0
```

#### What the Script Does

1. **Checks dependencies**: Verifies `ethtool` and `can-utils` are installed
2. **Detects CAN interfaces**: Finds USB-CAN adapters
3. **Configures bitrate**: Sets communication speed (default: 1 Mbps)
4. **Activates interface**: Brings the interface up
5. **Renames interface**: Standardizes to `can0` for consistency

#### Finding Your CAN Ports

If you have multiple USB-CAN adapters:

```bash
bash find_all_can_port.sh
```

Output example:
```
Interface can0 is connected to USB port 1-2:1.0
Interface can1 is connected to USB port 1-3:1.0
```

### Verifying CAN Connection

```bash
# Check interface status
ip link show can0

# Monitor CAN traffic
candump can0

# Send a test message
cansend can0 123#DEADBEEF
```

---

## Quick Start Guide

### Step 1: Install Docker

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add yourself to docker group (logout/login required)
sudo usermod -aG docker $USER
```

### Step 2: Install NVIDIA Docker (Optional, for GPU)

```bash
# Add NVIDIA Docker repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Install nvidia-docker2
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

### Step 3: Pull MoveIt Docker Image

```bash
# For development (source build)
docker pull moveit/moveit:noetic-source

# For demos (pre-built)
docker pull moveit/moveit:noetic-release
```

### Step 4: Run a Container with GUI

```bash
cd ~/catkin_ws/piper_ros/src/piper_moveit/moveit-1.1.11/.docker/

# Start container with your workspace mounted
./gui-docker -v ~/catkin_ws:/root/catkin_ws \
  -c piper_dev \
  -it moveit/moveit:noetic-source bash
```

### Step 5: Inside the Container

```bash
# Source ROS
source /opt/ros/noetic/setup.bash

# Navigate to workspace
cd /root/catkin_ws

# Build (first time only)
catkin build

# Source workspace
source devel/setup.bash

# Run MoveIt demo
roslaunch piper_with_gripper_moveit demo.launch
```

---

## Common Commands

### Docker Commands

```bash
# List running containers
docker ps

# List all containers (including stopped)
docker ps -a

# Stop a container
docker stop container_name

# Remove a container
docker rm container_name

# List downloaded images
docker images

# Remove an image
docker rmi image_name

# Enter a running container
docker exec -it container_name bash

# View container logs
docker logs container_name
```

### ROS Commands Inside Container

```bash
# Launch MoveIt with RViz (with gripper)
roslaunch piper_with_gripper_moveit demo.launch

# Launch MoveIt (without gripper)
roslaunch piper_no_gripper_moveit demo.launch

# Launch physical robot control
roslaunch piper start_single_piper.launch

# Launch Gazebo simulation
roslaunch piper_gazebo piper_gazebo.launch

# Launch MuJoCo simulation
roslaunch piper_mujoco piper_mujoco.launch
```

### CAN Commands

```bash
# Activate CAN interface
sudo bash can_activate.sh

# Check CAN status
ip -details link show can0

# Monitor CAN messages
candump can0

# Bring CAN interface down
sudo ip link set can0 down
```

---

## Troubleshooting

### GUI Not Working

**Symptom**: "cannot open display" or blank window

**Solutions**:
```bash
# Allow X11 connections from local
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY

# Verify X11 socket exists
ls -la /tmp/.X11-unix/
```

### NVIDIA GPU Not Detected

**Symptom**: "nvidia-docker not found" or slow rendering

**Solutions**:
```bash
# Check if NVIDIA driver is loaded
nvidia-smi

# Check if nvidia-docker is installed
docker info | grep -i nvidia

# Test NVIDIA Docker
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

### CAN Interface Issues

**Symptom**: "CAN interface not found" or communication errors

**Solutions**:
```bash
# Check if USB-CAN adapter is connected
lsusb | grep -i can

# Check kernel module
lsmod | grep can

# Load CAN modules manually
sudo modprobe can
sudo modprobe can_raw
sudo modprobe gs_usb

# Check interface exists
ip link show type can
```

### Container Cannot Access CAN

**Symptom**: CAN works on host but not in container

**Solution**: Run container with network privileges:
```bash
docker run --network=host --privileged -it image_name bash
```

Or mount specific device:
```bash
docker run --device=/dev/can0 -it image_name bash
```

### Build Failures

**Symptom**: `catkin build` fails with missing dependencies

**Solutions**:
```bash
# Update rosdep
rosdep update

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
catkin clean -y
catkin build
```

### Permission Denied

**Symptom**: Cannot write to mounted volume

**Solution**: Match user IDs or use root:
```bash
# Option 1: Run as root (inside container)
docker run --user root ...

# Option 2: Match host user ID
docker run --user $(id -u):$(id -g) ...
```

---

## Additional Resources

- [MoveIt Docker Documentation](https://moveit.ros.org/install/docker/)
- [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation)
- [Docker Documentation](https://docs.docker.com/)
- [NVIDIA Docker](https://github.com/NVIDIA/nvidia-docker)
- [CAN Bus Basics](https://en.wikipedia.org/wiki/CAN_bus)

---

## File Reference

| File | Location | Purpose |
|------|----------|---------|
| `gui-docker` | `.docker/gui-docker` | Run containers with GUI/GPU support |
| `can_activate.sh` | Root of catkin_ws | Configure CAN interface |
| `find_all_can_port.sh` | Root of catkin_ws | Discover CAN adapters |
| Dockerfile (release) | `.docker/release/` | Pre-built MoveIt image |
| Dockerfile (source) | `.docker/source/` | Build MoveIt from source |
| Dockerfile (ci) | `.docker/ci/` | Continuous integration base |

---

*Last updated: January 2026*
