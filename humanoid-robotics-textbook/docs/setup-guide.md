---
sidebar_position: 1
---

# Development Environment Setup Guide

This guide provides comprehensive instructions for setting up your development environment for the Physical AI & Humanoid Robotics textbook. You will need a workstation running Ubuntu 22.04.

## 1. Install Ubuntu 22.04

**(Assumes prior knowledge of installing Linux operating systems)**

If you don't already have Ubuntu 22.04 installed, please follow a reliable guide to install it on your primary machine or in a virtual machine (e.g., VirtualBox, VMware). Ensure you have sufficient disk space (at least 100GB) and RAM (16GB+ recommended).

## 2. Install ROS 2 Humble/Iron

This textbook targets ROS 2 Humble (LTS) or Iron. Please follow the official installation guide from Open Robotics.

- **Option A: ROS 2 Humble (Recommended)**
  [ROS 2 Humble Hawksbill Documentation: Install on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

- **Option B: ROS 2 Iron Irwini (Latest)**
  [ROS 2 Iron Irwini Documentation: Install on Ubuntu](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)

**Key steps to ensure during installation:**
- Add the ROS 2 apt repository.
- Install `ros-humble-desktop` or `ros-iron-desktop`.
- Source the ROS 2 setup script (`source /opt/ros/humble/setup.bash`).
- Install `colcon` build tools: `sudo apt install python3-colcon-common-extensions`.

## 3. Install NVIDIA Isaac Sim

NVIDIA Isaac Sim is a scalable robotics simulation application and development environment built on NVIDIA Omniverse.

Please follow the official NVIDIA documentation for installation. This typically involves:
1.  Setting up the NVIDIA Omniverse Launcher.
2.  Installing Isaac Sim through the Omniverse Launcher.
3.  Ensuring your NVIDIA GPU drivers are up-to-date.

[NVIDIA Isaac Sim Documentation: Installation Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/install_basic.html)

**Important considerations:**
- Isaac Sim requires a powerful NVIDIA GPU (RTX 4070 Ti+ recommended).
- Ensure you have sufficient VRAM.

## 4. Install Gazebo

Gazebo is a powerful 3D robot simulator. It comes in different versions; the textbook will primarily use Gazebo Garden, which is compatible with ROS 2 Humble/Iron.

Follow the official OSRF installation guide:
[Gazebo Documentation: Installation](https://gazebosim.org/docs/garden/install_ubuntu)

**Key steps:**
- Add the Gazebo apt repository.
- Install `ros-humble-gazebo-ros-pkgs` for ROS 2 integration.

## 5. Install Unity (Optional, for advanced Digital Twin labs)

Unity is another powerful 3D development platform. Some advanced labs might explore Unity for digital twin concepts.

Follow the official Unity Hub installation:
[Unity Documentation: Install the Unity Hub](https://docs.unity3d.com/Manual/GettingStartedInstallingHub.html)

**Important:** Ensure you install a version of Unity that is compatible with the Unity Robotics packages.

## 6. Python Environment Setup

It is highly recommended to use Python virtual environments (`venv`) for your project-specific Python dependencies.

```bash
# Navigate to your project directory (e.g., your ROS 2 workspace src folder)
cd ~/ros2_ws/src

# Create a virtual environment
python3 -m venv venv

# Activate the virtual environment
source venv/bin/activate

# Install any Python dependencies needed for your ROS 2 packages
# For example: pip install numpy
```

Remember to activate your virtual environment in every new terminal session where you intend to work on the textbook's labs.
