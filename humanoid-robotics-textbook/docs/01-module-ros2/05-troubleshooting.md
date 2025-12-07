---
sidebar_position: 5
---

# Troubleshooting Common ROS 2 Issues (Module 1)

This section provides solutions to common problems you might encounter while working with ROS 2 in this module.

## 1. `ros2` commands not found or `colcon build` fails

**Problem**: You try to run `ros2` commands (e.g., `ros2 run`, `ros2 topic list`) or `colcon build`, but the terminal reports "command not found" or `colcon` fails with environment-related errors.

**Solution**: This usually means your ROS 2 environment or your workspace environment has not been sourced correctly.

1.  **Source ROS 2**: Ensure you have sourced your main ROS 2 installation. For Humble, it's typically:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    For Iron:
    ```bash
    source /opt/ros/iron/setup.bash
    ```
2.  **Source Workspace**: If you've built packages in a workspace, you also need to source your workspace's setup file. Navigate to the root of your workspace (`~/ros2_ws` from our lab) and source its `install` directory:
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ```
    **Tip**: To avoid repeatedly sourcing, you can add these lines to your `~/.bashrc` file. However, be careful with multiple ROS distributions or workspaces.

## 2. Python Nodes Not Executable

**Problem**: You create a Python node (e.g., `publisher_node.py`), but `ros2 run my_ros2_package my_publisher` gives an error like "Could not find requested executable".

**Solution**: This indicates that ROS 2's build system (ament_python) doesn't know about your Python script.

1.  **Update `setup.py`**: Ensure you've added an entry point for your script in your package's `setup.py` file under `entry_points={'console_scripts': [...]}`. For example:
    ```python
            'console_scripts': [
                'my_publisher = my_ros2_package.publisher_node:main',
                'my_subscriber = my_ros2_package.subscriber_node:main',
            ],
    ```
2.  **Rebuild Package**: After modifying `setup.py`, you **must** rebuild your package:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_ros2_package
    ```
3.  **Resource Workspace**: And re-source your workspace:
    ```bash
    source install/setup.bash
    ```

## 3. Nodes Cannot Communicate (Publisher/Subscriber Lab)

**Problem**: Your publisher and subscriber nodes are running, but the subscriber isn't receiving messages, or `ros2 topic echo /my_topic` shows nothing.

**Solution**: Verify the communication chain.

1.  **Check Topic List**: Ensure the topic exists:
    ```bash
    ros2 topic list
    ```
    You should see `/my_topic` (and other default ROS 2 topics).
2.  **Check Message Type**: Confirm both publisher and subscriber are using the same message type (`std_msgs/msg/String` in our lab). You can inspect the topic info:
    ```bash
    ros2 topic info /my_topic
    ```
3.  **Check Node Status**: Verify both nodes are active:
    ```bash
    ros2 node list
    ```
    You should see `/my_publisher` and `/my_subscriber`.
4.  **Firewall Issues**: (Less common in local setups, but possible). Ensure no firewall is blocking UDP/TCP communication on relevant ports.

## 4. URDF Visualization in `rviz2` Fails

**Problem**: You launch `rviz2` and `state_publisher`, but your robot model doesn't appear or looks distorted.

**Solution**:
1.  **Check `robot_description` parameter**: Ensure the `state_publisher` is correctly loading your URDF file and setting the `robot_description` parameter. You can check the parameter:
    ```bash
    ros2 param get /robot_state_publisher robot_description
    ```
    (Note: `urdf_tutorial state_publisher` directly uses the URDF file, for other cases with `robot_state_publisher` node, you might need to check its configuration.)
2.  **`rviz2` Configuration**: In `rviz2`, verify:
    -   "Global Status" -> "Fixed Frame" is set to `base_link` (or the root link of your robot).
    -   "RobotModel" display is added and its "Description Topic" is set to `/robot_description`.
3.  **URDF Syntax**: Validate your URDF XML for syntax errors. Even small typos can cause issues. Online URDF validators or `urdf_check` tools can help.
4.  **`urdf_tutorial` Installation**: Make sure `ros-humble-urdf-tutorial` (or `iron`) is installed, as it provides the `state_publisher` executable needed for simple visualization.
