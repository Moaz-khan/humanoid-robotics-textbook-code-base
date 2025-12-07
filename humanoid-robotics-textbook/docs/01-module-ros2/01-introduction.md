---
sidebar_position: 1
---

# Introduction to ROS 2

The Robot Operating System (ROS) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors across a wide variety of robotic platforms. ROS 2 is the latest iteration, re-architected to address the limitations of ROS 1, particularly concerning real-time performance, multi-robot systems, and embedded platform support.

## Why ROS 2?

ROS 2 provides a standardized communication infrastructure and a rich ecosystem of tools that are crucial for developing sophisticated robotic applications. Key advantages include:

-   **Modular Design**: ROS 2 promotes breaking down complex robot functionalities into smaller, manageable units called **nodes**.
-   **Inter-process Communication**: A robust message passing system enables nodes to communicate seamlessly, even across different machines.
-   **Hardware Abstraction**: ROS 2 provides layers of abstraction, allowing developers to write high-level code that can be easily adapted to various robot hardware.
-   **Rich Tooling**: An extensive suite of development and debugging tools (e.g., Rviz, rqt, ros2cli) simplifies the development process.
-   **Community Support**: A large and active global community contributes to the continuous improvement and expansion of the ROS ecosystem.

## Core Concepts of ROS 2

Understanding these fundamental concepts is essential for working with ROS 2:

> ![ROS 2 Node Graph](/img/ros2_node_graph.png)

### Nodes

A **node** is an executable process that performs computation. In ROS 2, each node is responsible for a single, modular purpose (e.g., a node for reading camera data, another for controlling motors, a third for path planning). This modularity allows for easy debugging, replacement, and reuse of components.

### Topics

**Topics** are named buses over which nodes exchange messages. This is a publish/subscribe communication model:
-   A **publisher** node sends messages to a topic.
-   A **subscriber** node receives messages from a topic.

This decoupled communication allows nodes to operate independently, as long as they agree on the topic name and message type.

### Messages

**Messages** are data structures used by topics to transfer information. Each message has a specific type (e.g., `sensor_msgs/msg/Image` for camera data, `geometry_msgs/msg/Twist` for velocity commands). ROS 2 provides a wide range of standard message types, and you can also define custom ones.

### Services

**Services** are a request/reply communication model. They are used for synchronous, one-time transactions where a **client** node sends a request to a **server** node and waits for a reply. This is suitable for operations that need to complete before proceeding, such as triggering an action or querying data.

### Actions

**Actions** are a high-level communication type designed for long-running tasks. They provide a goal, feedback, and result mechanism. An **action client** sends a goal to an **action server**, receives continuous feedback as the goal is processed, and eventually gets a result when the goal is complete (or aborted). Examples include navigating to a target location or performing a complex manipulation task.

### Parameters

**Parameters** are dynamic configuration values for nodes. They allow you to change a node's behavior without recompiling the code. Parameters can be set at runtime, loaded from configuration files, or passed via the command line.

## Getting Started

Before diving into hands-on labs, ensure your ROS 2 environment is correctly set up as described in the [Development Environment Setup Guide](../setup-guide.md).
