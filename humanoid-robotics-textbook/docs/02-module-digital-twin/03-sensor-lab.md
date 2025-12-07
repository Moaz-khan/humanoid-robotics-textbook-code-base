---
sidebar_position: 3
---

# Digital Twin Lab: Adding Sensors in Gazebo

In the previous lab, you successfully imported your URDF robot into Gazebo. A robot's ability to perceive its environment is crucial for any autonomous behavior. This lab will guide you through adding a simulated sensor (specifically, a depth camera) to your robot model within Gazebo and visualizing its data in ROS 2.

## Learning Objectives

-   Modify a URDF to include a simulated depth camera.
-   Integrate the depth camera with Gazebo.
-   Visualize sensor data (image, point cloud) in ROS 2 tools like RViz2.

> ![Robot with Sensor in Gazebo and RViz2 Visualization](/img/gazebo_sensor_rviz.png)

## Prerequisites

-   Completed [Digital Twin Lab: Importing URDF into Gazebo](./02-labs.md).
-   A working `my_ros2_package` with `my_robot.urdf` and `display_robot_in_gazebo.launch.py`.
-   ROS 2 Humble or Iron installed and sourced.

## Lab Steps

### 1. Update `my_robot.urdf` to Include a Camera Link and Joint

First, we need to define a new `link` for our camera and a `joint` to attach it to an existing link on `my_robot` (e.g., `base_link`).

Open your `~/ros2_ws/src/my_ros2_package/urdf/my_robot.urdf` file and add the following XML snippets.

**Add a new `link` for the camera:**

Place this before the `</robot>` tag, or after your existing `arm_link`.

```xml
  <!-- Link for the simulated depth camera -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.05 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
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
```

**Add a new `joint` to connect the camera to your `base_link`:**

Place this after your existing `base_to_arm_joint`. Adjust `xyz` as needed to position the camera on your robot.

```xml
  <!-- Joint to attach the camera to the base_link -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/> <!-- Position the camera slightly in front and above the base -->
  </joint>
```

### 2. Add Gazebo Plugins to `my_robot.urdf`

Gazebo uses plugins to simulate sensors. We'll add a plugin for a depth camera that publishes ROS 2 topics.

Place this *inside* the `<robot>` tags but after all your `link` and `joint` definitions.

```xml
  <!-- Gazebo plugin for the depth camera -->
  <gazebo reference="camera_link">
    <sensor name="depth_camera_sensor" type="depth">
      <always_on>true</always_on>
      <update_rate>30.0</update_rate>
      <camera name="camera_sensor">
        <horizontal_fov>1.089</horizontal_fov> <!-- ~60 degrees -->
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
        <distortionT2>0.0</distortionT2>
        <Cx>319.5</Cx>
        <Cy>239.5</Cy>
        <Fx>277.19135</Fx>
        <Fy>277.19135</Fy>
      </plugin>
    </sensor>
  </gazebo>
```

### 3. Rebuild and Launch Your Package

Navigate back to your workspace root (`~/ros2_ws/`) and rebuild your package.

```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_package
source install/setup.bash # Re-source after build
```

Now, launch your robot in Gazebo again using the same launch file:

```bash
ros2 launch my_ros2_package display_robot_in_gazebo.launch.py
```

You should see your robot in Gazebo, now with a small box representing the camera attached.

### 4. Verify Sensor Topics in ROS 2

Open a **new terminal** (and remember to source your workspace).

List the active ROS 2 topics:

```bash
ros2 topic list
```

You should now see new topics related to your depth camera, similar to:

```
/rgb_camera/depth/camera_info
/rgb_camera/depth/image_raw
/rgb_camera/depth/points
/rgb_camera/image_raw
/rgb_camera/camera_info
...
```

To verify data is being published, echo one of the topics:

```bash
ros2 topic echo /rgb_camera/image_raw # For the raw image data
ros2 topic echo /rgb_camera/depth/points # For the point cloud data
```

You should see a continuous stream of image and point cloud messages.

### 5. Visualize Sensor Data in RViz2

Open another **new terminal** (and source your workspace). Launch RViz2:

```bash
ros2 run rviz2 rviz2
```

In RViz2:
1.  Set the "Fixed Frame" (under Global Options) to `camera_link`.
2.  Add a "RobotModel" display to see your robot.
3.  Add an "Image" display and set its topic to `/rgb_camera/image_raw` to see the camera's view.
4.  Add a "PointCloud2" display and set its topic to `/rgb_camera/depth/points` to visualize the depth point cloud.

You should now see the camera's perspective and a point cloud representation of the simulated environment.

## Verification

-   Your `my_robot` model with the camera is visible in Gazebo.
-   New ROS 2 topics for `rgb_camera` are being published.
-   You can visualize the camera image and point cloud in RViz2.

Congratulations! Your robot now has a perception system in the digital twin.
