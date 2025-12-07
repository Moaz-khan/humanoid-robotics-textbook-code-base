---
sidebar_position: 2
---

# ROS 2 Hands-on Lab: Publisher & Subscriber

This lab will guide you through creating your first ROS 2 nodes: a simple Python publisher that sends "Hello ROS 2" messages and a Python subscriber that receives and prints them. This demonstrates the fundamental publish/subscribe communication pattern in ROS 2.

## Learning Objectives

-   Create a ROS 2 Python package.
-   Write a Python publisher node.
-   Write a Python subscriber node.
-   Build and run ROS 2 packages.
-   Observe inter-node communication using `ros2 topic`.

## Prerequisites

-   Completed [Development Environment Setup Guide](../setup-guide.md).
-   Basic familiarity with the Linux terminal and Python programming.
-   ROS 2 Humble or Iron installed and sourced.

## Lab Steps

### 1. Create a ROS 2 Workspace

First, create a new ROS 2 workspace and a `src` directory within it. This is where your ROS 2 packages will reside.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Create a Python Package

Now, create a new Python ROS 2 package named `my_ros2_package` inside your workspace `src` directory.

```bash
ros2 pkg create --build-type ament_python my_ros2_package --dependencies rclpy std_msgs
```

This command creates a directory `my_ros2_package` with a basic structure, including `setup.py` and `package.xml` for package management, and declares dependencies on `rclpy` (the Python client library for ROS 2) and `std_msgs` (standard ROS 2 message types).

### 3. Create the Publisher Node

Navigate into your new package and create a Python script for the publisher.

```bash
cd my_ros2_package
mkdir my_ros2_package # Create a Python module directory
touch my_ros2_package/__init__.py # Make it a Python package
touch my_ros2_package/publisher_node.py
```

Open `my_ros2_package/publisher_node.py` and add the following code:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Create the Subscriber Node

Create another Python script for the subscriber node.

```bash
touch my_ros2_package/subscriber_node.py
```

Open `my_ros2_package/subscriber_node.py` and add the following code:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Update `setup.py`

You need to tell ROS 2 about your new executables. Open `my_ros2_package/setup.py` and add the entry points for your publisher and subscriber.

Locate the `entry_points` dictionary and add the following lines under `'console_scripts'`:

```python
        'console_scripts': [
            'my_publisher = my_ros2_package.publisher_node:main',
            'my_subscriber = my_ros2_package.subscriber_node:main',
        ],
```

Your `setup.py` file should look similar to this snippet (only the `entry_points` part shown):

```python
from setuptools import find_packages, setup

package_name = 'my_ros2_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_ros2_launch.py']), # Example launch file path
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_publisher = my_ros2_package.publisher_node:main',
            'my_subscriber = my_ros2_package.subscriber_node:main',
        ],
    },
)
```

**Note**: You may need to remove the example `data_files` entry for `launch/my_ros2_launch.py` if that file doesn't exist, or just leave it for now.

### 6. Build Your Package

Navigate back to your workspace root (`~/ros2_ws/`) and build your package using `colcon`.

```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_package
```

After a successful build, source your workspace's `setup.bash` file so ROS 2 can find your new executables.

```bash
source install/setup.bash
```

### 7. Run the Nodes

Open **three separate terminal windows**.

**Terminal 1: Run the Publisher**

```bash
ros2 run my_ros2_package my_publisher
```

You should see output similar to:
```
[INFO] [my_publisher]: Publishing: "Hello ROS 2: 0"
[INFO] [my_publisher]: Publishing: "Hello ROS 2: 1"
...
```

**Terminal 2: Run the Subscriber**

```bash
ros2 run my_ros2_package my_subscriber
```

You should see output similar to:
```
[INFO] [my_subscriber]: I heard: "Hello ROS 2: 0"
[INFO] [my_subscriber]: I heard: "Hello ROS 2: 1"
...
```

**Terminal 3: Monitor the Topic (Optional)**

Use `ros2 topic echo` to directly inspect messages being published on `my_topic`.

```bash
ros2 topic echo /my_topic
```

You should see:
```
data: Hello ROS 2: 0
---
data: Hello ROS 2: 1
---
...
```

### 8. Verification

-   Both publisher and subscriber nodes are running without errors.
-   The subscriber node is receiving and printing messages from the publisher.
-   The `ros2 topic echo` command shows the messages on `/my_topic`.

Congratulations! You have successfully created and run your first ROS 2 publish/subscribe nodes.
