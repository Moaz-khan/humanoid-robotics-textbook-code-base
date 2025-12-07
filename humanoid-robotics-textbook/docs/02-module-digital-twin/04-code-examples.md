---
sidebar_position: 4
---

# Digital Twin Lab: Code Examples (Gazebo & Sensors)

This section provides the complete code examples used in the "Importing URDF into Gazebo" and "Adding Sensors in Gazebo" labs for your convenience. You can download these files to ensure your setup is correct or to compare with your own implementations.

## 1. `my_robot.urdf` (with Camera Sensor)

This is the URDF file for our simple robot, updated to include the `camera_link`, `camera_joint`, and the `gazebo` plugin for the depth camera.

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Materials -->
  <material name="red">
    <color rgba="0.8 0 0 1"/>
  </material>
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>

  <!-- Link 1: Base -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Link 2: Arm Segment -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15"/>
    </collision>
    <inertial>
      <origin xyz="0 0 0.15"/>
      <mass value="0.2"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint: Connects base_link to arm_link -->
  <joint name="base_to_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" velocity="1.0" effort="10.0"/>
  </joint>

  <!-- Link for the simulated depth camera -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.05 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.01"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>

  <!-- Joint to attach the camera to the base_link -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugin for the depth camera -->
  <gazebo reference="camera_link">
    <sensor name="depth_camera_sensor" type="depth">
      <always_on>true</always_on>
      <update_rate>30.0</update_rate>
      <camera name="camera_sensor">
        <horizontal_fov>1.089</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>8.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
        <baseline>0.05</baseline>
        <alwaysOn>true</alwaysOn>
        <updateRate>30.0</updateRate>
        <cameraName>rgb_camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <depthImageTopicName>depth/image_raw</depthImageTopicName>
        <depthImageInfoTopicName>depth/camera_info</depthImageInfoTopicName>
        <pointCloudTopicName>depth/points</pointCloudTopicName>
        <frameName>camera_link</frameName>
        <hackBaseline>0.0</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>00.0</distortionT2>
        <Cx>319.5</Cx>
        <Cy>239.5</Cy>
        <Fx>277.19135</Fx>
        <Fy>277.19135</Fy>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo default material for black -->
  <gazebo reference="camera_link">
    <material>Gazebo/Black</material>
  </gazebo>
</robot>
```

## 2. `display_robot_in_gazebo.launch.py` (with Sensor Spawn)

This is the launch file used to bring up Gazebo and spawn our robot with its integrated camera.

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
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file_path, 'r').read()}],
        output='screen'
    )

    # Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'empty.world'}.items()
    )

    # Spawn Entity Node
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node
    ])
```
