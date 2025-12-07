--- 
sidebar_position: 3
---

# NVIDIA Isaac Lab: Visual SLAM (VSLAM)

Visual Simultaneous Localization and Mapping (VSLAM) is a key AI robotics technology that allows a robot to build a map of an unknown environment while simultaneously estimating its own position within that map, using only visual sensor data (e.g., from a camera). This lab will guide you through running a VSLAM pipeline in Isaac Sim using Isaac ROS.

## Learning Objectives

- Understand the concept of VSLAM.
- Run a VSLAM pipeline within Isaac Sim.
- Visualize the generated map and robot's pose in RViz2.

## Prerequisites

- Completed [NVIDIA Isaac Lab: Setting Up Isaac Sim with a Robot](./02-labs.md).
- NVIDIA Isaac Sim installed and a robot model (with a camera sensor) imported into a scene.
- ROS 2 Humble or Iron installed and sourced.
- Basic understanding of ROS 2 topics and RViz2 visualization.

## Lab Steps

### 1. Set Up Isaac ROS VSLAM

Isaac ROS provides a highly optimized VSLAM package. You'll typically use a `ros2 launch` file to bring up the VSLAM nodes.

First, ensure you have Isaac ROS installed and your development environment is set up as per the [Development Environment Setup Guide](../setup-guide.md).

### 2. Prepare Your Isaac Sim Scene

Ensure your Isaac Sim scene has a robot with a camera and a reasonably complex environment for VSLAM to map. A simple empty world might not provide enough features for mapping. You can load one of Isaac Sim's example worlds (e.g., "Warehouse") or create one with obstacles.

### 3. Launch Isaac Sim with ROS 2 Bridge

Isaac Sim needs to be launched with the ROS 2 Bridge enabled for communication with external ROS 2 nodes.

1.  Open Isaac Sim.
2.  Go to `Window > ROS > ROS 2 Bridge`.
3.  Ensure "Enable ROS 2 Bridge" is checked.

### 4. Run the VSLAM Launch File

Isaac ROS typically provides example launch files for its VSLAM pipeline. For demonstration, we'll assume a simplified launch for `isaac_ros_visual_slam`.

Open a **terminal** where your ROS 2 environment is sourced (and sourced your workspace, if applicable).

```bash
# Example: Launch an Isaac ROS VSLAM pipeline.
# The actual command might vary based on the specific Isaac ROS version and package.
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_isaac_sim.launch.py \
    rgb_topic:=/my_robot/camera/rgb/image_raw \
    depth_topic:=/my_robot/camera/depth/image_raw \
    camera_info_topic:=/my_robot/camera/rgb/camera_info
```
*(Note: Replace `/my_robot/camera/...` with the actual topics published by your robot's camera in Isaac Sim. You can find these topics using `ros2 topic list`.)*

### 5. Visualize VSLAM Output in RViz2

Open a **new terminal** (and source your workspace). Launch RViz2:

```bash
ros2 run rviz2 rviz2
```

In RViz2:

1.  **Global Options**: Set "Fixed Frame" to `odom` or `map` (if VSLAM provides one). If you don't have these, you might start with your robot's `base_link` and see if `tf` data is available.
2.  **Add `tf` Display**: To see the coordinate frames, add a "tf" display.
3.  **Add `PointCloud2` for Map**: VSLAM often generates a point cloud of the map. Add a "PointCloud2" display and set its topic to the VSLAM map topic (e.g., `/visual_slam/tracking/slam_cloud`).
4.  **Add `Path` for Robot Trajectory**: If VSLAM publishes the robot's trajectory, add a "Path" display and set its topic accordingly (e.g., `/visual_slam/path`).
5.  **Move the Robot**: In Isaac Sim, move your robot around the environment (either manually or by applying forces/velocities). You should observe the map building in RViz2 and the robot's trajectory being updated.

### 6. Verification

- The Isaac ROS VSLAM launch file runs without critical errors.
- VSLAM publishes relevant topics (e.g., point clouds, poses).
- RViz2 displays a growing map of the environment as the robot moves.
- The robot's estimated pose in RViz2 tracks its movement in Isaac Sim.

Congratulations! You have successfully implemented a Visual SLAM pipeline for your robot in Isaac Sim.
