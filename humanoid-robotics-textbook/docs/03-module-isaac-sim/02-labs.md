---
sidebar_position: 2
---

# NVIDIA Isaac Lab: Setting Up Isaac Sim with a Robot

This lab will guide you through the process of setting up NVIDIA Isaac Sim and importing a robot model. Isaac Sim provides a highly realistic, physics-enabled simulation environment crucial for developing and testing AI-powered robotics applications.

## Learning Objectives

-   Launch NVIDIA Isaac Sim.
-   Understand the Isaac Sim UI and basic navigation.
-   Import a URDF robot model into Isaac Sim.
-   Observe basic robot interaction within the simulation.

## Prerequisites

-   Completed [Development Environment Setup Guide](../setup-guide.md), with NVIDIA Isaac Sim installed and accessible.
-   A working `my_ros2_package` with `my_robot.urdf` from Module 1 (or a more complex humanoid URDF if you have one).
-   A powerful NVIDIA GPU is essential.

## Lab Steps

### 1. Launch NVIDIA Isaac Sim

Start Isaac Sim through the NVIDIA Omniverse Launcher. This will open the Isaac Sim application.

-   **Initial Setup**: The first launch might take some time as it downloads assets and compiles shaders.
-   **Explore the UI**: Familiarize yourself with the main view, the Stage panel (on the left, showing scene hierarchy), and the Property panel (on the right, showing details of selected objects).

### 2. Prepare Your URDF for Isaac Sim

Isaac Sim has its own URDF importer. While it generally supports standard URDF, some features might need adjustments or specific Gazebo plugins need to be removed for optimal performance in Isaac Sim.

For this lab, we will use a simple URDF. If you are using the `my_robot.urdf` from Module 1, ensure it's accessible.

### 3. Import URDF Robot into Isaac Sim

Isaac Sim provides a built-in URDF importer.

1.  In Isaac Sim, go to `File > Import > URDF`.
2.  Navigate to your `~/ros2_ws/src/my_ros2_package/urdf/my_robot.urdf` file and select it.
3.  An "Import URDF" dialog will appear. You can usually accept the default settings for now. Click "Import".

Your robot model should now appear in the Isaac Sim stage.

### 4. Basic Scene Setup

To interact with your robot, you might want to add a simple ground plane and some lighting.

1.  **Add a Ground Plane**: Go to `Create > Physics > Ground Plane`.
2.  **Add Lighting**: Go to `Create > Light > Dome Light` (for ambient lighting) or `Create > Light > Distant Light` (for directional lighting).
3.  **Position Robot**: Select your robot in the Stage panel. In the Property panel, adjust its `Transform` properties (position and rotation) to place it on the ground plane.

### 5. Start Physics Simulation

In the bottom timeline/playback controls, click the "Play" button (triangle icon) to start the physics simulation.

-   Your robot should respond to gravity. If it falls through the floor, ensure the ground plane has collision enabled and your robot's links have collision geometries defined in the URDF.
-   You can manually drag and interact with the robot parts using the mouse while the simulation is running.

### 6. Verification

-   Isaac Sim launches successfully.
-   Your URDF robot model is imported and visible in the scene.
-   The robot responds to physics when the simulation is played (e.g., falls to the ground).
-   You can manipulate the robot parts manually during simulation.

Congratulations! You have successfully set up Isaac Sim and imported your first robot model.
