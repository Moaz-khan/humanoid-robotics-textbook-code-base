---
sidebar_position: 3
---

# Understanding URDF (Unified Robot Description Format)

The Unified Robot Description Format (URDF) is an XML format for describing all elements of a robot. It's used in ROS 2 to represent the kinematic and dynamic properties of a robot, as well as its visual and collision characteristics. A URDF file allows ROS 2 tools and simulation environments (like Gazebo) to correctly interpret and display your robot model.

## Why URDF is Important

-   **Visualization**: Tools like `rviz` use URDF to visualize your robot's structure, joints, and sensors in a 3D environment.
-   **Simulation**: Simulators like Gazebo parse URDF files to understand the robot's physical properties (mass, inertia, collision geometry) and kinematic relationships (how joints move relative to each other).
-   **Motion Planning**: Motion planning libraries (e.g., MoveIt) use the kinematic chain defined in the URDF to plan collision-free paths for the robot's manipulators.
-   **Hardware Interface**: Provides a common interface for software to interact with the robot's physical structure.

## Core Elements of URDF

A URDF file primarily consists of `link` and `joint` elements.

> ![Simple Two-Link Robot Arm in RViz2](/img/urdf_robot_arm_rviz.png)

### `link` Element

A `link` represents a rigid body segment of the robot. It can have several properties:

-   **`inertial`**: Describes the mass, center of mass, and inertia matrix of the link. Essential for physics simulations.
    ```xml
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    ```
-   **`visual`**: Defines how the link looks. This includes its geometry (e.g., box, cylinder, mesh) and material properties (color, texture).
    ```xml
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    ```
-   **`collision`**: Defines the geometry used for collision detection. This can be simpler than the visual geometry for performance.
    ```xml
    <collision>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
    </collision>
    ```

### `joint` Element

A `joint` describes the kinematic and dynamic properties of the connection between two links.

-   **`name`**: Unique identifier for the joint.
-   **`type`**: Specifies the type of motion allowed. Common types include:
    -   `revolute`: A single axis of rotation with limits (e.g., a motor).
    -   `continuous`: A single axis of rotation without limits (e.g., a wheel).
    -   `prismatic`: Linear motion along an axis with limits (e.g., a linear actuator).
    -   `fixed`: No motion allowed (e.g., connecting a sensor rigidly).
-   **`parent` / `child`**: Specifies the two links connected by the joint.
-   **`origin`**: Defines the pose of the child link relative to the parent link.
-   **`axis`**: Specifies the axis of motion for revolute and prismatic joints.
-   **`limit`**: Defines the upper and lower bounds for joint position, velocity, and effort for revolute and prismatic joints.
-   **`dynamics`**: (Optional) Defines friction and damping properties.

```xml
<joint name="base_link_to_arm_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" velocity="1.0" effort="10.0"/>
</joint>
```

## Example: Simple Two-Link Arm

Let's describe a simple arm with a base and a single rotating arm segment.

### `my_robot.urdf`

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Materials (optional, but good practice) -->
  <material name="red">
    <color rgba="0.8 0 0 1"/>
  </material>
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
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
      <origin xyz="0 0 0.15"/> <!-- Offset to make cylinder stand upright on joint -->
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
    <origin xyz="0 0 0.05" rpy="0 0 0"/> <!-- Joint is on top of base_link -->
    <axis xyz="0 0 1"/> <!-- Rotates around Z-axis -->
    <limit lower="-3.14" upper="3.14" velocity="1.0" effort="10.0"/>
  </joint>

</robot>
```

### Visualizing Your URDF (using `rviz2`)

1.  **Save the URDF**: Save the XML code above into a file named `my_robot.urdf` in your ROS 2 package (e.g., `~/ros2_ws/src/my_ros2_package/urdf/my_robot.urdf`). You'll need to create the `urdf` directory first.

    ```bash
    mkdir -p ~/ros2_ws/src/my_ros2_package/urdf
    # Then create my_robot.urdf in that directory with the content above
    ```

2.  **Install `urdf_tutorial`**: This package provides a `state_publisher` that reads your URDF and publishes the joint states, which `rviz2` needs to display the robot.

    ```bash
    sudo apt install ros-humble-urdf-tutorial # or ros-iron-urdf-tutorial
    ```

3.  **Launch `rviz2` and `state_publisher`**:

    ```bash
    # Open a terminal and source your workspace (if not already sourced)
    cd ~/ros2_ws
    source install/setup.bash

    # Launch the state publisher
    ros2 run urdf_tutorial state_publisher my_robot.urdf

    # In a new terminal, launch rviz2
    ros2 run rviz2 rviz2
    ```

4.  **Configure `rviz2`**:
    -   In the `rviz2` window, click "Add" in the "Displays" panel.
    -   Select "RobotModel" and click "OK".
    -   In the "RobotModel" properties, set the "Description Topic" to `/robot_description`.
    -   You should see your simple arm appear. You can interact with it using a `joint_state_publisher_gui` if installed (`sudo apt install ros-humble-joint-state-publisher-gui`).

## Next Steps

Experiment with changing the `origin` `xyz` and `rpy` values, `geometry` sizes, and `joint` `axis` in your URDF file. Observe how these changes affect the robot's visualization in `rviz2`.
