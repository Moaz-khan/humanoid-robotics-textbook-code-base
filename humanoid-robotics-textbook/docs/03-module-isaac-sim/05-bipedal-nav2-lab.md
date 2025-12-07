---
sidebar_position: 4
---

# NVIDIA Isaac Lab: Bipedal Path Planning (Nav2)

Once a robot can perceive and map its environment (VSLAM), the next crucial step is to enable it to navigate autonomously. For humanoid robots, this involves **bipedal path planning**, which is significantly more complex than planning for wheeled robots due to balance and gait considerations. This lab will guide you through using ROS 2's Navigation2 (Nav2) stack for path planning, adapted for a bipedal robot in Isaac Sim.

## Learning Objectives

-   Understand the basics of Nav2 for autonomous navigation.
-   Configure Nav2 for a simulated bipedal robot.
-   Send navigation goals to your robot in Isaac Sim.
-   Visualize the planned path and robot movement in RViz2.

## Prerequisites

-   Completed [NVIDIA Isaac Lab: Visual SLAM (VSLAM)](./03-vslam-lab.md).
-   A robot model in Isaac Sim with a camera and basic mobility (e.g., configurable joints for bipedal motion, even if simplified).
-   A generated map from your VSLAM lab (or a pre-existing map of your Isaac Sim environment).
-   ROS 2 Humble or Iron installed and sourced.
-   `ros-humble-navigation2` and `ros-humble-nav2-bringup` packages installed (or `iron` equivalents).

## Lab Steps

### 1. Ensure Nav2 is Installed

If you haven't already, install the Nav2 packages:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup # For Humble
# or
sudo apt install ros-iron-navigation2 ros-iron-nav2-bringup # For Iron
```

### 2. Prepare Your Isaac Sim Scene and Robot

1.  **Isaac Sim Launch**: Launch Isaac Sim with the ROS 2 Bridge enabled, as in the previous VSLAM lab.
2.  **Robot Mobility**: For a bipedal robot, you'll need to define how it moves. For simplicity in this lab, you might initially control the robot's base directly via `cmd_vel` topics, or if your robot has articulated legs, publish joint commands. Nav2 typically expects a `cmd_vel` topic for velocity commands. Even for bipedal robots, a base controller that translates `cmd_vel` into appropriate leg movements is required.
3.  **Map**: Ensure you have a map of your environment, either generated from the VSLAM lab or a static map.

### 3. Configure Nav2 Parameters

Nav2 requires extensive configuration, especially for bipedal robots. This involves setting parameters for:

-   **`amcl` (Adaptive Monte Carlo Localization)**: For localization within a known map.
-   **`map_server`**: To load your pre-built map.
-   **`bt_navigator` (Behavior Tree Navigator)**: For high-level decision making.
-   **`planner_server`**: For global path planning (e.g., using A* or Dijsktra).
-   **`controller_server`**: For local path planning and obstacle avoidance (e.g., using DWB or TEB).
-   **`recovery_server`**: For handling navigation failures.

These parameters are typically defined in YAML files. For a bipedal robot, the `controller_server` parameters are especially critical to ensure stable gait and balance.

**Example `nav2_params.yaml` snippet for a bipedal robot (simplified):**

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0 # Bipedal robots might need higher control frequencies
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker:
      base_frame_id: base_link
      period: 0.1
      # Consider custom progress checker for bipedal gaits

    # Specific controller plugin for bipedal motion (placeholder, would be custom)
    controller_plugin_ids: ["SimpleBipedalController"]
    controller_plugin_types: ["bipedal_nav2_controller::SimpleBipedalController"]

    SimpleBipedalController: # Custom controller for bipedal motion
      odom_topic: odom
      cmd_vel_topic: cmd_vel
      # Parameters specific to bipedal gait, balance, etc.
```

### 4. Create a Launch File for Nav2

You'll need a launch file to bring up the Nav2 stack. This will usually include the `map_server`, `amcl`, `planner_server`, `controller_server`, etc.

**Example `nav2_bringup.launch.py` snippet (simplified for illustration):**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_nav_params_path = os.path.join(
        get_package_share_directory('my_ros2_package'),
        'config',
        'nav2_params.yaml'
    )
    map_file_path = os.path.join(
        get_package_share_directory('my_ros2_package'),
        'maps',
        'my_map.yaml' # Your generated map
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file_path,
            description='Full path to map file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': my_nav_params_path,
                'use_sim_time': 'true',
            }.items()
        )
    ])
```
*(You'll need to create `config/nav2_params.yaml` and `maps/my_map.yaml` in your package.)*

### 5. Launch Nav2 and Send a Goal

1.  **Launch Isaac Sim** (with ROS 2 Bridge).
2.  **Launch your robot model** (e.g., using `display_robot_in_gazebo.launch.py` if you have a bipedal model that takes `cmd_vel` or joint commands).
3.  **Launch Nav2**: In a new terminal (sourced), run your Nav2 launch file:
    ```bash
    ros2 launch my_ros2_package nav2_bringup.launch.py
    ```
4.  **Launch RViz2**: In another new terminal (sourced), run:
    ```bash
    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
    ```
    This launches RViz2 with a pre-configured Nav2 view.
5.  **Set Initial Pose**: In RViz2, use the "2D Pose Estimate" tool to set the initial pose of your robot on the map.
6.  **Send Navigation Goal**: Use the "2D Goal Pose" tool in RViz2 to click on a desired location on the map and drag to set the robot's orientation.

Your robot should now plan a path and attempt to navigate to the goal in Isaac Sim, and you'll see the planned path and actual robot movement in RViz2.

### 6. Verification

-   Nav2 stack launches without errors.
-   The robot localizes itself on the map in RViz2.
-   When a goal is sent, Nav2 plans a path.
-   The simulated bipedal robot attempts to follow the path and reaches the goal (or gets close, depending on controller tuning).

Congratulations! You've integrated Nav2 for bipedal path planning in Isaac Sim.
