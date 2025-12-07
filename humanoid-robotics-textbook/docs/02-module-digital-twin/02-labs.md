---
sidebar_position: 2
---

# Digital Twin Lab: Importing URDF into Gazebo

This lab will guide you through importing a URDF robot model, similar to the one you created in Module 1, into the Gazebo simulation environment. This is a crucial step in creating a digital twin of your robot, allowing you to test its physical properties and behaviors in a realistic virtual world.

## Learning Objectives

-   Prepare a ROS 2 package for Gazebo integration.
-   Launch a URDF robot model in Gazebo.
-   Visualize the robot in Gazebo's 3D environment.
-   Understand basic Gazebo controls.

## Prerequisites

-   Completed [Development Environment Setup Guide](../setup-guide.md), with Gazebo installed.
-   Completed the URDF Fundamentals section from Module 1, and have a working URDF file (e.g., `my_robot.urdf` from `my_ros2_package`).
-   ROS 2 Humble or Iron installed and sourced.

## Lab Steps

### 1. Ensure Your URDF is Ready

Make sure your `my_robot.urdf` file (from `~/ros2_ws/src/my_ros2_package/urdf/my_robot.urdf`) is valid and describes a simple robot, such as the two-link arm example.

### 2. Create a Launch File for Gazebo

Gazebo is typically launched via ROS 2 launch files, which allow you to specify the world file, the robot model, and other configurations.

Navigate to your ROS 2 package (`~/ros2_ws/src/my_ros2_package/`) and create a `launch` directory, then a Python launch file:

```bash
cd ~/ros2_ws/src/my_ros2_package
mkdir launch
touch launch/display_robot_in_gazebo.launch.py
```

Open `launch/display_robot_in_gazebo.launch.py` and add the following Python code:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_ros2_package'
    
    # Path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'my_robot.urdf'
    )

    # Robot State Publisher Node
    # Reads the URDF and publishes the /robot_description topic
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file_path, 'r').read()}],
        output='screen'
    )

    # Gazebo launch file
    # Uses an empty world, but you can specify a different one
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'empty.world'}.items()
    )

    # Spawn Entity Node
    # Spawns the robot into Gazebo using the /robot_description topic
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node
    ])
```

### 3. Update `setup.py`

Ensure your `setup.py` file includes the `launch` files and `urdf` files so ROS 2 can find them. Add the following to your `setup.py` (under `data_files`):

```python
from glob import glob
import os

# ... (other imports and package_name definition)

setup(
    # ...
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yem]'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    # ...
)
```

### 4. Build and Source Your Package

Navigate back to your workspace root (`~/ros2_ws/`) and rebuild your package.

```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_package
```

After a successful build, source your workspace's `setup.bash` file so ROS 2 can find your new executables and launch files.

```bash
source install/setup.bash
```

### 5. Launch Your Robot in Gazebo

```bash
ros2 launch my_ros2_package display_robot_in_gazebo.launch.py
```

Gazebo should launch, and you should see your `my_robot` model appear in the simulation environment.

## 6. Verification

-   Gazebo launches successfully.
-   Your `my_robot` URDF model is visible in the Gazebo window.
-   You can navigate the Gazebo environment and observe your robot.

Congratulations! You have successfully imported your URDF robot into Gazebo, creating a basic digital twin.
