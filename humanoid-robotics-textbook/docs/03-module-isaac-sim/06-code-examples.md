---
sidebar_position: 6
---

# NVIDIA Isaac Lab: Code Examples (VSLAM & Nav2)

This section provides the complete code examples and configurations for the VSLAM and Nav2 labs in Module 3. You can use these files to verify your setup or as a reference for your own implementations.

## 1. `my_robot_isaac.urdf` (Example for Isaac Sim)

While Isaac Sim can import standard URDFs, sometimes minor adjustments or specific Isaac Sim plugins are beneficial. For the purpose of these labs, we can assume a `my_robot.urdf` similar to the one from Module 2, but ensure it is well-defined for Isaac Sim physics.

*(Content for `my_robot_isaac.urdf` would be very similar to `my_robot.urdf` from Module 2, but potentially with specific Isaac Sim properties or simplified collision geometries.)*

## 2. Isaac ROS VSLAM Launch File (Example)

This is an example launch file that would typically be used to bring up the `isaac_ros_visual_slam` nodes. Note that specific topic names and node parameters might vary based on your robot and Isaac ROS version.

```python
# ~/ros2_ws/src/my_ros2_package/launch/isaac_ros_vslam.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    isaac_ros_visual_slam_package = get_package_share_directory('isaac_ros_visual_slam')

    # Path to the pre-trained model (if required by your VSLAM config)
    # model_path = os.path.join(isaac_ros_visual_slam_package, 'models', 'some_model.bin')

    vslam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'denoise_input_images': False,
            'rectify_input_images': True, # Set to true if your camera does not provide rectified images
            'enable_imu_fusion': False, # Set to true if your robot has an IMU
            'gyro_noise_density': 0.0002,
            'accel_noise_density': 0.002,
            'gyro_random_walk': 0.00002,
            'accel_random_walk': 0.0002,
            'calibration_topic': '/my_robot/camera/rgb/camera_info', # Adjust to your camera info topic
            'image_topic': '/my_robot/camera/rgb/image_raw',        # Adjust to your image topic
            'depth_topic': '/my_robot/camera/depth/image_raw',      # Adjust to your depth topic
            'imu_topic': '/my_robot/imu',                           # Adjust to your IMU topic (if enable_imu_fusion is true)
            'enable_slam': True,
            'enable_localization': False,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link', # Your robot's base frame
            'odom_pose_frame': 'odom',
            'publish_tf': True,
            'publish_map_to_odom_tf': True,
            'invert_odom_tf': True,
            'input_type': 1, # 0: RGBD, 1: Stereo, 2: Depth
            # 'path_to_model': model_path, # Uncomment if using a pre-trained model
        }]
    )

    return LaunchDescription([
        vslam_node
    ])
```

## 3. Nav2 Parameter File (Simplified for Bipedal Robot)

This `nav2_params.yaml` provides a starting point for configuring Nav2 for a bipedal robot. **Note**: A real bipedal robot would require a custom `controller_server` plugin that translates Nav2 velocity commands into stable walking gaits. This example assumes such a plugin is available.

```yaml
# ~/ros2_ws/src/my_ros2_package/config/nav2_params.yaml
# Common Nav2 parameters
planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      allow_unknown: true
      # ... other planner parameters

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

    # Custom controller for bipedal motion (placeholder, would be a separate package)
    controller_plugin_ids: ["SimpleBipedalController"]
    controller_plugin_types: ["bipedal_nav2_controller::SimpleBipedalController"]

    SimpleBipedalController: # Placeholder for custom bipedal controller plugin
      odom_topic: odom
      cmd_vel_topic: cmd_vel
      # Parameters specific to bipedal gait, balance, etc.
      max_linear_velocity: 0.2
      max_angular_velocity: 0.5

recovery_server:
  ros__parameters:
    use_sim_time: True
    recovery_plugins: ["Spin", "BackUp"]
    Spin:
      # ... spin parameters
    BackUp:
      # ... backup parameters

bt_navigator:
  ros__parameters:
    use_sim_time: True
    # ... behavior tree parameters

# AMCL for localization
amcl:
  ros__parameters:
    use_sim_time: True
    # ... amcl parameters

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "my_map.yaml" # Your generated map file
```

## 4. Nav2 Bringup Launch File (Example)

This launch file brings up the full Nav2 stack using the parameters defined above and loads your map.

```python
# ~/ros2_ws/src/my_ros2_package/launch/nav2_bringup_bipedal.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_ros2_package_dir = get_package_share_directory('my_ros2_package')

    # Path to your custom Nav2 parameters file
    nav2_params_path = os.path.join(my_ros2_package_dir, 'config', 'nav2_params.yaml')

    # Path to your map file (generated by VSLAM or created manually)
    map_file_path = os.path.join(my_ros2_package_dir, 'maps', 'my_map.yaml')

    # Declare launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to map file to load'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_path,
        description='Full path to the Nav2 parameters file'
    )

    # Include Nav2 bringup launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_file_arg,
        nav2_launch
    ])
```
