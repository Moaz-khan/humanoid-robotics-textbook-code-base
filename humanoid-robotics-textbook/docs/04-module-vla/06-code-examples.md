---
sidebar_position: 6
---

# Capstone Project: Autonomous Humanoid - Code Examples

This section provides the complete code examples and configuration files for the Capstone Project and the VLA Lab. These are provided as a reference to help you integrate the various components of your autonomous humanoid.

## 1. `stt_node.py` (Speech-to-Text Node)

This Python node uses the `SpeechRecognition` library to convert spoken commands into text and publishes them as a ROS 2 `std_msgs/msg/String` message on the `voice_command` topic.

```python
# ~/ros2_ws/src/my_ros2_package/my_ros2_package/stt_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading
import time
import os

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
                # Ensure you have internet connection
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
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred in STT: {e}")
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

## 2. `nli_node.py` (Natural Language Interpreter Node)

This Python node subscribes to the `voice_command` topic, uses an OpenAI LLM to interpret the command, and publishes a robot action request as a `std_msgs/msg/String` message on the `robot_action_request` topic.

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

## 3. `action_executor_node.py` (Robot Action Executor Node)

This Python node subscribes to `robot_action_request` and translates it into actual `geometry_msgs/msg/Twist` commands for your simulated robot.

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
