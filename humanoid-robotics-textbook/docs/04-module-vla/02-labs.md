---
sidebar_position: 2
---

# VLA Lab: Basic Conversational Control

This lab introduces the fundamentals of Vision-Language-Action (VLA) systems by guiding you through setting up a basic conversational interface for your simulated humanoid robot. You will integrate a Speech-to-Text (STT) service and a Large Language Model (LLM) to translate high-level natural language commands into simple robot actions.

## Learning Objectives

-   Understand the flow of a basic VLA pipeline.
-   Integrate a Speech-to-Text service.
-   Use an LLM to interpret natural language commands and extract intent/entities.
-   Map LLM output to simple ROS 2 actions.
-   Control a simulated robot using voice commands.

## Prerequisites

-   Completed all labs in Module 3 (NVIDIA Isaac).
-   A simulated robot in Isaac Sim that can receive basic ROS 2 commands (e.g., `cmd_vel` for navigation, or simple joint commands for manipulation).
-   Access to an LLM API (e.g., OpenAI GPT-3.5/4).
-   Python environment with necessary libraries (`openai`, `speech_recognition`, `pyaudio` for STT).

## Architectural Overview

We will implement a simplified VLA pipeline:

```mermaid
graph TD
    A[Human Voice Command] --> B(Speech-to-Text)
    B --> C(Natural Language Interpreter - LLM)
    C -- Robot Action Request (e.g., "move forward") --> D[ROS 2 Command Publisher]
    D --> E[Simulated Robot in Isaac Sim]
```

## Lab Steps

### 1. Set Up Speech-to-Text (STT)

We will use the `SpeechRecognition` library in Python for STT. This library supports various STT engines, including Google Speech Recognition (free for basic use) and OpenAI Whisper.

Install the necessary Python packages:

```bash
pip install SpeechRecognition pyaudio openai
```

Create a Python script (`stt_node.py`) in your `my_ros2_package` to listen for voice commands and publish them as a ROS 2 string message.

```python
# ~/ros2_ws/src/my_ros2_package/my_ros2_package/stt_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading
import time

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.get_logger().info('STT Node initialized. Listening for commands...')
        self.listening_thread = threading.Thread(target=self._listen_loop)
        self.listening_thread.daemon = True
        self.listening_thread.start()

    def _listen_loop(self):
        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().info("Say something!")
                    self.recognizer.adjust_for_ambient_noise(source)
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                
                # Using Google Web Speech API (online, requires internet)
                text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f"You said: {text}")
                
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
            except sr.WaitTimeoutError:
                self.get_logger().info("No speech detected.")
            except sr.UnknownValueError:
                self.get_logger().warn("Could not understand audio.")
            except sr.RequestError as e:
                self.get_logger().error(f"Could not request results from Google Speech Recognition service; {e}")
            time.sleep(0.1) # Small delay to prevent busy-looping

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Set Up Natural Language Interpreter (LLM)

Next, create a node (`nli_node.py`) that subscribes to the `voice_command` topic, uses an LLM to interpret the command, and publishes a robot action request.

```python
# ~/ros2_ws/src/my_ros2_package/my_ros2_package/nli_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai # pip install openai
import os

class NLI_Node(Node):
    def __init__(self):
        super().__init__('nli_node')
        self.subscription = self.create_subscription(
            String,
            'voice_command',
            self.voice_command_callback,
            10
        )
        self.action_publisher_ = self.create_publisher(String, 'robot_action_request', 10)
        
        # Ensure your OpenAI API key is set as an environment variable
        # export OPENAI_API_KEY="sk-..."
        openai.api_key = os.getenv("OPENAI_API_KEY")
        if not openai.api_key:
            self.get_logger().error("OPENAI_API_KEY environment variable not set. LLM will not function.")
            
        self.get_logger().info('NLI Node initialized. Waiting for voice commands...')

    def voice_command_callback(self, msg):
        command_text = msg.data
        self.get_logger().info(f"Received voice command: '{command_text}'")
        
        if not openai.api_key:
            self.get_logger().warn("OpenAI API key missing. Skipping LLM processing.")
            return

        try:
            # Example prompt for simple robot commands
            prompt = (
                f"You are a robot command interpreter. Translate the following human command into a single, simple robot action. "
                f"Valid actions are: 'move forward', 'move backward', 'turn left', 'turn right', 'stop', 'pick up', 'put down'. "
                f"If the command implies one of these, output only the action. Otherwise, output 'unknown'."
                f"\nHuman: '{command_text}'\nRobot Action:"
            )
            
            response = openai.chat.completions.create(
                model="gpt-3.5-turbo", # Or "gpt-4"
                messages=[
                    {"role": "system", "content": prompt},
                    {"role": "user", "content": command_text}
                ],
                max_tokens=20,
                temperature=0.0
            )
            
            robot_action = response.choices[0].message.content.strip().lower()
            self.get_logger().info(f"LLM interpreted action: '{robot_action}'")
            
            action_msg = String()
            action_msg.data = robot_action
            self.action_publisher_.publish(action_msg)

        except openai.OpenAIError as e:
            self.get_logger().error(f"OpenAI API error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error during LLM processing: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NLI_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Create a Robot Action Executor Node

This node (`action_executor_node.py`) subscribes to `robot_action_request` and translates it into actual `geometry_msgs/msg/Twist` commands for your simulated robot (assuming it has a mobile base or can interpret these for bipedal motion).

```python
# ~/ros2_ws/src/my_ros2_package/my_ros2_package/action_executor_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist # For publishing velocity commands
import time

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor_node')
        self.subscription = self.create_subscription(
            String,
            'robot_action_request',
            self.action_request_callback,
            10
        )
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Action Executor Node initialized. Waiting for action requests...')
        self.twist_msg = Twist() # Initialize an empty Twist message

    def action_request_callback(self, msg):
        action = msg.data
        self.get_logger().info(f"Executing action: '{action}'")

        if action == 'move forward':
            self._publish_velocity(0.2, 0.0) # Linear X velocity
            time.sleep(1.0) # Move for 1 second
            self._publish_velocity(0.0, 0.0) # Then stop
        elif action == 'move backward':
            self._publish_velocity(-0.2, 0.0)
            time.sleep(1.0)
            self._publish_velocity(0.0, 0.0)
        elif action == 'turn left':
            self._publish_velocity(0.0, 0.5) # Angular Z velocity
            time.sleep(0.5)
            self._publish_velocity(0.0, 0.0)
        elif action == 'turn right':
            self._publish_velocity(0.0, -0.5)
            time.sleep(0.5)
            self._publish_velocity(0.0, 0.0)
        elif action == 'stop':
            self._publish_velocity(0.0, 0.0)
        elif action == 'pick up':
            self.get_logger().info("Pretending to pick up an object.")
            # In a real scenario, this would involve complex manipulation commands
            pass
        elif action == 'put down':
            self.get_logger().info("Pretending to put down an object.")
            pass
        else:
            self.get_logger().warn(f"Unknown action requested: '{action}'")

    def _publish_velocity(self, linear_x, angular_z):
        self.twist_msg.linear.x = float(linear_x)
        self.twist_msg.angular.z = float(angular_z)
        self.cmd_vel_publisher_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Update `setup.py`

Add the new nodes (`stt_node.py`, `nli_node.py`, `action_executor_node.py`) to your `my_ros2_package/setup.py` under `console_scripts` in the `entry_points` dictionary.

```python
        'console_scripts': [
            'my_publisher = my_ros2_package.publisher_node:main',
            'my_subscriber = my_ros2_package.subscriber_node:main',
            'stt_node = my_ros2_package.stt_node:main',
            'nli_node = my_ros2_package.nli_node:main',
            'action_executor_node = my_ros2_package.action_executor_node:main',
        ],
```

### 5. Build and Source Your Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_package
source install/setup.bash # Re-source after build
```

### 6. Run the VLA Pipeline

**Terminal 1: Isaac Sim and Robot**
Launch Isaac Sim with your robot model, ensuring the ROS 2 Bridge is enabled. Your robot should be configured to subscribe to `cmd_vel` (e.g., via a `diff_drive_controller` in Gazebo, or an equivalent in Isaac Sim).

**Terminal 2: STT Node**
```bash
ros2 run my_ros2_package stt_node
```

**Terminal 3: NLI Node**
```bash
ros2 run my_ros2_package nli_node
```
(Ensure your `OPENAI_API_KEY` environment variable is set in this terminal.)

**Terminal 4: Action Executor Node**
```bash
ros2 run my_ros2_package action_executor_node
```

Now, speak commands into your microphone in the terminal running `stt_node`. For example, "move forward", "turn left", "stop". Observe the output in each terminal and the robot's reaction in Isaac Sim.

## Verification

-   The `stt_node` correctly transcribes your voice commands to text.
-   The `nli_node` successfully interprets the text commands and publishes robot action requests.
-   The `action_executor_node` receives the action requests and publishes `cmd_vel` messages.
-   Your simulated robot in Isaac Sim responds to your voice commands by moving or performing the designated action (if implemented).

Congratulations! You've built a basic Vision-Language-Action pipeline for your robot.

