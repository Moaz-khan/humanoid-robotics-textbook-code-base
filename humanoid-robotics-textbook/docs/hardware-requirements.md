---
sidebar_position: 3
---

# Hardware & Software Requirements

This textbook is designed to be highly practical, with hands-on labs and simulations that require specific hardware and software configurations to ensure a smooth learning experience. Adhering to these recommendations will help you reproduce all experiments and exercises successfully.

## Minimum Recommended Hardware

To run the simulations and development tools effectively, a capable workstation is required.

-   **Processor**: Intel Core i7 (8th Gen or newer) or AMD Ryzen 7 (3rd Gen or newer).
-   **RAM**: 32 GB DDR4 RAM.
-   **Graphics Card (GPU)**: NVIDIA GeForce RTX 3060 (or equivalent with at least 8 GB VRAM). **NVIDIA GPUs are critical for Isaac Sim.**
-   **Storage**: 500 GB SSD (NVMe recommended) with at least 150 GB free space for operating system, development tools, and simulation assets.
-   **Network**: Stable broadband internet connection.

## Recommended Hardware for Optimal Experience

For the best experience, especially with complex Isaac Sim scenarios and larger Unity projects, we recommend:

-   **Processor**: Intel Core i9 (12th Gen or newer) or AMD Ryzen 9 (5th Gen or newer).
-   **RAM**: 64 GB DDR4 RAM or more.
-   **Graphics Card (GPU)**: NVIDIA GeForce RTX 4070 Ti (or newer, with 12 GB+ VRAM).
-   **Storage**: 1 TB NVMe SSD with at least 300 GB free space.

## Operating System

-   **Ubuntu 22.04 LTS (Jammy Jellyfish)**: This is the primary development and testing environment. All instructions and code examples are validated against this specific Linux distribution.
    -   A fresh installation is recommended to avoid conflicts with existing software.

## Key Software Requirements

The following software will be used extensively throughout the textbook:

-   **ROS 2 Humble Hawksbill** or **ROS 2 Iron Irwini**: The Robot Operating System 2. We primarily use Python for ROS 2 programming.
-   **NVIDIA Isaac Sim (latest stable version)**: For high-fidelity robotics simulation and synthetic data generation. Requires a compatible NVIDIA GPU.
-   **Gazebo Garden (latest stable version)**: For robust 3D robot simulation, integrated with ROS 2.
-   **Unity Editor (LTS version with Robotics package)**: For advanced digital twin experiments and human-robot interaction.
-   **Python 3.10+**: With `pip` and `venv` for managing project dependencies.
-   **Git**: For version control and cloning code repositories.
-   **VS Code (or your preferred IDE)**: With Python, ROS, and C/C++ extensions for development.

## Optional Hardware / Edge AI Kits

For those interested in deploying to physical hardware, the following are examples of compatible platforms:

-   **NVIDIA Jetson Orin Nano / NX**: For edge AI inference and deployment of ROS 2 applications.
-   **Intel RealSense D435i / D455**: Depth cameras often used for robot perception.
-   **ReSpeaker USB Mic Array**: For voice input in VLA applications.

## Cloud Alternatives (Considerations)

While local hardware is recommended for hands-on learning, cloud-based simulation environments like **AWS RoboMaker** or **NVIDIA Omniverse Cloud** can be used as alternatives. However, be aware of:

-   **Latency**: Potential delays in interacting with the simulation.
-   **Cost**: Cloud usage can incur significant costs over time.
-   **Setup Complexity**: May require additional configuration knowledge.

Ensure you have a stable and fast internet connection if opting for cloud solutions.

---

For detailed installation instructions for the required software, please refer to the [Development Environment Setup Guide](setup-guide.md).
