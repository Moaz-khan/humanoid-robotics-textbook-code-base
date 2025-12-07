---
sidebar_position: 5
---

# Module 2 Checkpoint Quiz: Digital Twin and Simulation

This short quiz will test your understanding of digital twins, Gazebo, Unity, and sensor integration covered in Module 2. Choose the best answer for each question.

---

**1. Which of the following is a primary advantage of using robotic simulation for development?**
    a) Eliminates the need for real-world testing.
    b) Provides a safe, repeatable, and cost-effective environment.
    c) Always guarantees photorealistic rendering.
    d) Requires less computational power than physical hardware.

**2. In the context of ROS 2 and Gazebo, what is the purpose of a URDF file?**
    a) To define ROS 2 communication protocols.
    b) To describe the robot's physical structure, joints, sensors, and properties for simulation.
    c) To control the robot's motors directly.
    d) To generate synthetic sensor data.

**3. Which ROS 2 package is commonly used to spawn a URDF model into Gazebo from a launch file?**
    a) `rclpy`
    b) `std_msgs`
    c) `gazebo_ros` (specifically `spawn_entity.py`)
    d) `robot_state_publisher`

**4. When adding a simulated camera to your robot in Gazebo, which type of XML tag in the URDF is used to define the sensor's properties and publish ROS 2 topics?**
    a) `<link>`
    b) `<joint>`
    c) `<plugin>` (within `<gazebo>` tag)
    d) `<material>`

**5. What is the typical ROS 2 tool used to visualize streamed sensor data (like camera images or point clouds) from a simulated robot?**
    a) `ros2 topic echo`
    b) `colcon build`
    c) `rviz2`
    d) `ros2 run`

---

## Answers

1.  **b) Provides a safe, repeatable, and cost-effective environment.**
2.  **b) To describe the robot's physical structure, joints, sensors, and properties for simulation.**
3.  **c) `gazebo_ros` (specifically `spawn_entity.py`)**
4.  **c) `<plugin>` (within `<gazebo>` tag)**
5.  **c) `rviz2`**
