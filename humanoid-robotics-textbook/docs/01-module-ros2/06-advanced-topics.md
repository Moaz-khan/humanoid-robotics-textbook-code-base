---
sidebar_position: 6
---

# Optional Advanced Topics: ROS 2

This section delves into more advanced concepts and features of ROS 2 that are not essential for the core understanding of Module 1 but can significantly enhance your robotic development capabilities. These topics are suitable for students who wish to explore beyond the basics.

---

:::note Optional
The content in this section is supplementary. You can skip it and still proceed with the rest of the textbook.
:::

---

## 1. ROS 2 Launch Files (XML and Python)

While `ros2 run` is great for starting individual nodes, complex robotic systems involve many nodes, often with specific configurations and interdependencies. ROS 2 **Launch Files** are used to define and manage the startup of multiple ROS 2 nodes and processes simultaneously. They allow you to:

-   Start multiple nodes with a single command.
-   Remap topic names.
-   Set node parameters.
-   Include other launch files.
-   Define conditionals for starting nodes.

ROS 2 supports both XML and Python-based launch files. Python launch files are generally preferred due to their flexibility and programmatic capabilities.

**Example Python Launch File (`my_launch_file.py`):**

```python
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_ros2_package',
            executable='my_publisher',
            name='my_publisher_node',
            output='screen',
            emulate_tty=True, # Required for output to console
            parameters=[
                {'timer_period': 1.0} # Override default parameter
            ]
        ),
        launch_ros.actions.Node(
            package='my_ros2_package',
            executable='my_subscriber',
            name='my_subscriber_node',
            output='screen',
            emulate_tty=True,
        )
    ])
```

To run this launch file:
```bash
ros2 launch my_ros2_package my_launch_file.py
```
*(You would need to place `my_launch_file.py` in a `launch` directory within your package and update `setup.py` to include this file.)*

## 2. Quality of Service (QoS) Settings

ROS 2's communication is built on DDS (Data Distribution Service), which offers advanced Quality of Service (QoS) policies. QoS settings allow you to fine-tune the reliability, latency, durability, and other aspects of your communication channels. Understanding QoS is critical for real-time and robust robotic applications.

Common QoS policies include:

-   **`reliability`**:
    -   `reliable`: Guarantees delivery (slower).
    -   `best_effort`: Attempts delivery without guarantees (faster, might lose messages).
-   **`durability`**:
    -   `transient_local`: Keeps messages for late-joining subscribers.
    -   `volatile`: Only for active subscribers.
-   **`history`**:
    -   `keep_last`: Stores a certain number of messages.
    -   `keep_all`: Stores all messages until resource limits are met.

You can specify QoS profiles when creating publishers or subscribers in your code.

**Example (Python Publisher with QoS):**

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MyReliablePublisher(Node):
    def __init__(self):
        super().__init__('my_reliable_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.publisher_ = self.create_publisher(String, 'my_topic', qos_profile)
        # ... rest of your publisher code
```

## 3. Custom Message and Service Definitions

While `std_msgs` offers many basic types, real-world robots often require custom data structures. ROS 2 allows you to define your own message (`.msg`) and service (`.srv`) types.

**Example Custom Message (`MyCustomMessage.msg`):**
```
int32 id
string name
geometry_msgs/Point position
bool is_active
```

This message can then be used in your nodes just like standard messages. Defining custom types involves:
1.  Creating `.msg` (or `.srv`) files in a `msg` (or `srv`) directory within your package.
2.  Adding dependencies in `package.xml` (e.g., `rosidl_default_generators`).
3.  Configuring `CMakeLists.txt` to generate the necessary code for your custom types.
4.  Rebuilding your package.

## 4. Parameter Callbacks and Dynamic Reconfigure

ROS 2 parameters can be dynamically updated at runtime. You can attach **callbacks** to parameters to react to changes immediately. This is particularly useful for tuning a robot's behavior without stopping and restarting nodes.

**Example Parameter Callback:**

```python
from rcl_interfaces.msg import SetParametersResult

class MyDynamicNode(Node):
    def __init__(self):
        super().__init__('my_dynamic_node')
        self.declare_parameter('my_int_param', 0)
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'my_int_param' and param.type_ == rclpy.Parameter.Type.INTEGER:
                self.get_logger().info(f'Parameter my_int_param changed to: {param.value}')
                return SetParametersResult(successful=True)
        return SetParametersResult(successful=False)
```

You can change parameters using `ros2 param set /my_dynamic_node my_int_param 5`.
