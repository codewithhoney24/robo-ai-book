# Module 1: The Robotic Nervous System (ROS 2)

## Week 4: Bridging Python Agents to ROS Controllers using rclpy

### 1. Introduction to rclpy (ROS Client Library for Python)

`rclpy` ROS 2 ka official Python client library hai, jo Python developers ko ROS 2 ecosystem ke saath interact karne ki sahulat deta hai. Iski madad se aap ROS 2 nodes, publishers, subscribers, services, aur actions Python mein develop kar sakte hain. `rclpy` C++ client library (`rclcpp`) ke underlying ROS client library (RCL) par based hai, jo performance aur consistency provide karta hai.

### 2. Setting Up Your Python Environment for ROS 2

ROS 2 projects ke liye Python environment set karna zaroori hai. Ismein `colcon` build system aur `pip` ka istemal shamil hai taake dependencies manage ki ja saken. Best practice yeh hai ke aap Python virtual environments (e.g., `venv`) ka istemal karein taake project-specific dependencies ko isolate kiya ja sake.

**Technical Content:**
- ROS 2 installation aur environment setup (source or binary).
- Python virtual environment creation aur activation.
- `colcon` build tool for ROS 2 Python packages.

**Code Example (Bash - Environment Setup Conceptual):**
```bash
# Source ROS 2 setup file (adjust path as per your ROS 2 installation)
# Replace `humble` with your ROS 2 distribution name (e.g., `foxy`, `galactic`, `humble`)
source /opt/ros/humble/setup.bash

# Create and activate a Python virtual environment
python3 -m venv ~/ros2_ws/src/my_python_pkg/venv
source ~/ros2_ws/src/my_python_pkg/venv/bin/activate

# Install rclpy and other dependencies within the virtual environment
# Note: rclpy is usually installed with ROS 2 itself, but you might need to install
# other Python packages that your ROS 2 Python nodes depend on.
# For example:
# pip install numpy

# Create a ROS 2 workspace and package
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_python_pkg --dependencies rclpy std_msgs

# Go back to the workspace root and build
cd ~/ros2_ws
colcon build --packages-select my_python_pkg

# Source the workspace setup file to make your package's executables available
source install/setup.bash
```

**Visual Aid Placeholder:**
```
[Image/Diagram: A flowchart illustrating the steps to set up a Python virtual environment and a ROS 2 Python package.]
```

### 3. Creating ROS 2 Python Nodes with rclpy

`rclpy` ka use karke Python mein ROS 2 nodes banana straight-forward hai. Har node `rclpy.node.Node` class se inherit karta hai aur uske paas publishers, subscribers, clients, servers, aur timers banane ki capability hoti hai.

**Code Example (Python - Node with Publisher and Subscriber):**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PythonAgentNode(Node):

    def __init__(self):
        super().__init__('python_agent_node')
        self.publisher_ = self.create_publisher(String, 'robot_command', 10)
        self.subscription = self.create_subscription(
            String, 'sensor_data', self.listener_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.get_logger().info('Python Agent Node Initialized with command publisher and sensor subscriber!')

    def timer_callback(self):
        msg = String()
        msg.data = f'MOVE FORWARD {self.i} units' # Example command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing command: "{msg.data}"')
        self.i += 1

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard sensor data: "{msg.data}"')
        # In a real agent, this data would be processed to inform decisions

def main(args=None):
    rclpy.init(args=args)
    agent_node = PythonAgentNode()
    rclpy.spin(agent_node)
    agent_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Hardware Context:**
- **Jetson Orin Nano**: `rclpy` nodes ko Jetson Orin Nano jaise embedded platforms par run kiya ja sakta hai, jo on-robot decision-making aur low-latency control loops ke liye perfect hai.

### 4. Interfacing with ROS Controllers

Python agents `rclpy` ka istemal karke hardware-level ROS controllers ke saath interact karte hain. Yeh interaction topics ke zariye commands bhej kar (e.g., motor velocities, joint positions) aur sensor data receive kar ke hota hai. Ismein `ros2_control` framework ka bhi zikr kiya jayega, jo controllers ko abstract karta hai.

**Technical Content:**
- `ros2_control` overview: Hardware interfaces, controller managers, controllers (e.g., `joint_state_broadcaster`, `diff_drive_controller`).
- Custom message types (Agar zaroori ho) jo specific hardware commands ke liye use hote hain.
- Publish-subscribe pattern ka istemal motor commands aur encoder readings ke liye.

**Code Example (Conceptual - Python Agent controlling a Joint using `ros2_control`):**
```python
import rclpy
from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory # For actual use
# from sensor_msgs.msg import JointState # For actual use

class JointControllerAgent(Node):

    def __init__(self):
        super().__init__('joint_controller_agent')
        # self.joint_command_publisher = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        # self.joint_state_subscription = self.create_subscription(
        #     JointState, 'joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop_callback) # 100 Hz control loop
        self.current_joint_position = 0.0
        self.target_joint_position = 1.0 # radians
        self.get_logger().info('Joint Controller Agent Initialized!')

    def joint_state_callback(self, msg):
        # For a specific joint, find its position
        # try:
        #     index = msg.name.index('joint_1') # Assuming 'joint_1' is the name of the joint
        #     self.current_joint_position = msg.position[index]
        #     self.get_logger().info(f'Current joint_1 position: {self.current_joint_position:.2f}')
        # except ValueError:
        #     self.get_logger().warn('Joint "joint_1" not found in joint_states message.')
        pass # Conceptual

    def control_loop_callback(self):
        # Basic P-controller concept for reaching target position
        error = self.target_joint_position - self.current_joint_position
        kp = 0.5 # Proportional gain
        velocity_command = kp * error

        # Create and publish joint trajectory message (conceptual)
        # if self.joint_command_publisher and abs(error) > 0.01:
        #     trajectory_msg = JointTrajectory()
        #     trajectory_msg.joint_names = ['joint_1']
        #     point = JointTrajectoryPoint()
        #     point.positions = [self.current_joint_position + velocity_command * 0.1] # Small step
        #     point.time_from_start = Duration(sec=0, nanosec=100_000_000).to_msg() # 0.1 seconds
        #     trajectory_msg.points.append(point)
        #     self.joint_command_publisher.publish(trajectory_msg)
        #     self.get_logger().info(f'Publishing joint command: {velocity_command:.2f}')
        # else:
        #     self.get_logger().info('Joint at target or command not published.')

        self.get_logger().info(f'Control loop running. Error: {error:.2f}, Cmd: {velocity_command:.2f}')

def main(args=None):
    rclpy.init(args=args)
    agent_node = JointControllerAgent()
    rclpy.spin(agent_node)
    agent_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Visual Aid Placeholder:**
```
[Image/Diagram: A diagram showing a Python agent node communicating with a ros2_control hardware interface via ROS 2 topics.]
```

**Hardware Context:**
- **RTX GPU**: High-level motion planning ya inverse kinematics ke calculations ke liye jo complex robot manipulators ke liye zaroori hain, unko RTX GPU par process kiya ja sakta hai aur phir commands ROS topics ke through controllers ko bheje ja sakte hain.
- **Jetson Orin Nano**: `ros2_control` hardware interfaces ko low-level motor drivers ke saath interact karne ke liye Jetson Orin Nano par deploy kiya ja sakta hai, jo real-time feedback aur control ke liye zaroori hai.