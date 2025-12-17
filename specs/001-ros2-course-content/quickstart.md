# Quickstart Guide: ROS 2 Course Content

## Overview
This guide will help you get started with the "Advanced Course Content: Module 1 - The Robotic Nervous System (ROS 2)" course. This module targets intermediate robotics students and developers with basic Python knowledge who want to integrate AI agents with robotic control systems using ROS 2.

## Prerequisites

Before starting this course, you should have:

1. **Linux Environment**: Ubuntu 22.04 LTS (recommended for ROS 2 Humble)
2. **Python Knowledge**: Basic understanding of Python programming (Python 3.8+)
3. **Basic Control Theory**: Understanding of fundamental control systems concepts
4. **ROS 2 Installation**: ROS 2 Humble Hawksbill installed on your system

### Installing ROS 2 Humble
If you haven't installed ROS 2 yet, follow the official installation guide:
```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/rosSigning.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update
```

### Environment Setup
```bash
# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# For convenience, add this to your ~/.bashrc file:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Course Structure

The course consists of 3 comprehensive chapters:

1. **Chapter 1**: ROS 2 Architecture & Communication Patterns
2. **Chapter 2**: Bridging AI Agents to Robot Controllers
3. **Chapter 3**: Humanoid Robot Representation with URDF

Each chapter includes hands-on coding exercises with validation criteria.

## Getting Started with Chapter 1

### Running Your First ROS 2 Node

1. Create a new workspace:
```bash
mkdir -p ~/ros2_course_ws/src
cd ~/ros2_course_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

2. Create a simple publisher node:
```python
# Save as publisher_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. Run the publisher:
```bash
cd ~/ros2_course_ws
source install/setup.bash
python3 publisher_member_function.py
```

4. In a new terminal, run a subscriber to see messages:
```bash
cd ~/ros2_course_ws
source install/setup.bash
ros2 run demo_nodes_py listener
```

## Setting Up for Chapter 2: AI Integration

Chapter 2 focuses on connecting AI decision-making agents to ROS 2 controllers. You'll need:

1. Basic Python AI libraries (numpy, etc.)
2. The rclpy library for ROS 2 communication

Example AI-ROS bridge code structure:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np  # Example AI library

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')
        
        # ROS 2 subscribers for sensor data
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10
        )
        
        # ROS 2 publishers for commands
        self.command_publisher = self.create_publisher(
            String,
            'robot_commands',
            10
        )
        
        # AI model initialization
        self.ai_model = self.initialize_ai_model()
    
    def sensor_callback(self, msg):
        # Process sensor data through AI model
        processed_data = self.process_with_ai(msg.data)
        
        # Publish command based on AI decision
        command_msg = String()
        command_msg.data = processed_data
        self.command_publisher.publish(command_msg)
    
    def initialize_ai_model(self):
        # Initialize your AI model here
        pass
    
    def process_with_ai(self, sensor_data):
        # Apply AI logic to sensor data
        return "command_based_on_ai_decision"
```

## Setting Up for Chapter 3: URDF

Chapter 3 covers Humanoid Robot Representation with URDF. For this, you'll need:

1. ROS 2 visualization tools (RViz2)
2. URDF validation tools (check_urdf, xacro)

Example command to visualize a URDF file:
```bash
# Check URDF validity
check_urdf /path/to/your/robot.urdf

# Visualize in RViz
ros2 run rviz2 rviz2

# In RViz, add RobotModel display and set Robot Description to your URDF
```

## Course Completion Criteria

To successfully complete this course, you should be able to:

1. Create functional ROS 2 nodes that communicate via topics and services
2. Implement bidirectional communication between Python AI agents and ROS 2 controllers using rclpy
3. Parse and modify URDF files to represent humanoid robot kinematics
4. Explain the architectural differences between ROS 1 and ROS 2 middleware

## Getting Help

- If you encounter issues, check the course exercises with validation criteria
- Use the AI assistant integrated with the course content for additional help
- Review the official ROS 2 documentation for detailed technical information