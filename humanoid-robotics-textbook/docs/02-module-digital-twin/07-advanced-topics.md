---
sidebar_position: 7
---

# Optional Advanced Topics: Digital Twin and Simulation

This section explores more advanced topics related to digital twins and robotic simulation beyond the core material of Module 2. These topics are not essential for completing the main labs but offer deeper insights and practical applications for those interested in extending their simulation capabilities.

---

:::note Optional
The content in this section is supplementary. You can skip it and still proceed with the rest of the textbook.
:::

---

## 1. Custom Gazebo Worlds

In our labs, we've used simple empty worlds. Gazebo allows you to create highly detailed and complex environments, known as **Gazebo Worlds**. These are XML files (`.world`) that define:

-   **Models**: Static objects (buildings, furniture, obstacles) and dynamic objects (other robots, moving platforms).
-   **Physics Properties**: Gravity, simulation update rates, solver types.
-   **Sensors**: Global sensors like sun light or wind.
-   **Lights**: Ambient, directional, point, and spot lights to control scene illumination.
-   **Plugins**: World-level plugins for custom simulation logic.

**Creating a simple custom world**:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_simple_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="red_box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1.0</mass><inertia><ixx>0.166</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.166</iyy><iyz>0</iyz><izz>0.166</izz></inertia></inertial>
        <visual><geometry><box><size>1 1 1</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>
        <collision><geometry><box><size>1 1 1</size></box></geometry></collision>
      </link>
    </model>
  </world>
</sdf>
```

You can then launch this world using your `display_robot_in_gazebo.launch.py` by modifying the `world` argument:

```python
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'my_custom_world.world'}.items() # Use your custom world
    )
```

## 2. Advanced Sensor Modeling (Noise and Distortion)

Our simple camera sensor assumed ideal conditions. Real-world sensors are affected by noise, distortion, and environmental factors. Gazebo offers advanced features to model these imperfections:

-   **Noise**: Adding Gaussian noise to sensor readings (e.g., camera images, LiDAR ranges).
-   **Lens Distortion**: Simulating radial and tangential lens distortion for cameras.
-   **Material Properties**: Defining realistic surface properties (roughness, reflectivity) that interact with sensors (e.g., LiDAR reflections).

These advanced models are crucial for developing robust perception algorithms that can handle real-world sensor data.

## 3. Real-time Factor (RTF) and Simulation Performance

The **Real-time Factor (RTF)** in Gazebo indicates how fast the simulation is running compared to real-time. An RTF of 1.0 means the simulation is running in real-time. An RTF < 1.0 means it's slower than real-time, and RTF > 1.0 means it's faster.

Maintaining a good RTF is vital for:

-   **Accurate Physics**: Physics engines are sensitive to time steps; a stable RTF ensures consistent simulation.
-   **Control Loop Timing**: If the simulation runs too slowly, your robot's control loops might miss deadlines.

**Tips for improving RTF**:

-   Simplify your world: Reduce the number of models, polygons, and complex physics interactions.
-   Reduce sensor update rates and resolutions.
-   Use simpler collision geometries than visual geometries.
-   Run Gazebo on a powerful machine with a dedicated GPU.
-   Disable GUI for headless simulations.

## 4. Unity Robotics and ROS 2 Bridge

For more advanced human-robot interaction or photorealistic rendering, Unity is an excellent choice. The **Unity Robotics** packages, combined with the **ROS 2 Bridge**, allow seamless integration:

-   **URDF Importer**: Import your URDF models directly into Unity.
-   **ROS TCP Connector**: Establish communication between Unity and your ROS 2 graph.
-   **Message Generation**: Automatically generate C# message types from ROS 2 `.msg` and `.srv` files.

This enables you to use Unity as a powerful visualization, control, and data generation platform for your ROS 2 robots.

## 5. Multi-Robot Simulation

Gazebo and Unity are capable of simulating multiple robots interacting within the same environment. This is essential for studying swarm robotics, collaborative tasks, and complex logistical scenarios.

-   **Unique Namespaces**: In ROS 2, use namespaces for each robot (`/robot1/cmd_vel`, `/robot2/cmd_vel`) to avoid topic/service name collisions.
-   **Robot Descriptions**: Load multiple robot descriptions into the simulation, each with its own launch file and configuration.
-   **Inter-robot Communication**: Design ROS 2 communication (topics, services) between robots for coordination.
