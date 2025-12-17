---
id: ros2-fundamentals
title: The Robotic Nervous System (ROS 2) - Nodes, Topics, and Services
slug: /module-1/ros2-fundamentals
---

## Introduction to ROS 2

The Robot Operating System 2 (ROS 2) provides a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors across a wide variety of robotic platforms. Think of it as the "nervous system" for your robot, coordinating all its different parts.

### Why ROS 2?

ROS 2 addresses many of the limitations of its predecessor, ROS 1, particularly concerning real-time performance, multi-robot systems, and embedded platforms. It leverages DDS (Data Distribution Service) for robust, efficient, and reliable communication.

## ROS 2 Core Concepts

Understanding ROS 2 revolves around its core communication concepts: Nodes, Topics, and Services.

### Nodes: The Brain Cells of Your Robot

In ROS 2, a **Node** is an executable element that performs computation. Nodes are designed to be modular, with each node responsible for a single, well-defined task. This modularity allows for easier development, debugging, and reuse of code.

**Real-world Example:**
- A camera driver node that publishes image data.
- A motor control node that subscribes to velocity commands and sends signals to motors.
- A navigation node that processes sensor data to determine robot pose and plan paths.

#### Creating a Simple ROS 2 Node (Python)

Let's create a basic ROS 2 Python node. This node will simply print "Hello, ROS 2!" repeatedly.

:::note
Before running any ROS 2 code, ensure your ROS 2 environment is sourced. For example, `source /opt/ros/humble/setup.bash` (replace `humble` with your ROS 2 distribution).
:::

**1. Create a ROS 2 Package**

First, navigate to your ROS 2 workspace `src` directory and create a new package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_ros2_package --dependencies rclpy
```

This command creates a new Python package named `my_ros2_package` with a dependency on `rclpy`, the Python client library for ROS 2.

**2. Implement the Node**

Inside `my_ros2_package/my_ros2_package`, create a Python file (e.g., `simple_node.py`) with the following content:

```python
import rclpy
from rclpy.node import Node

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2! %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
:::warning
The above code snippet needs `from std_msgs.msg import String` to be imported. Please ensure to add this import statement.
:::

Let's correct the code and update the explanation. The `SimplePublisher` is more than just printing; it's also publishing to a topic. We will adjust it to reflect a simple node that just prints for clarity and then introduce the publisher.

**Corrected Simple Node (Just Prints):**

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node started!')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()
    rclpy.spin_once(minimal_node) # Spin once to ensure logger message is processed
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
- `import rclpy` and `from rclpy.node import Node`: Import necessary ROS 2 Python client libraries.
- `class MinimalNode(Node):`: Defines a new class `MinimalNode` that inherits from `rclpy.node.Node`.
- `super().__init__('minimal_node')`: Initializes the node with the name `minimal_node`.
- `self.get_logger().info('Minimal node started!')`: Uses the ROS 2 logger to print a message.
- `rclpy.init(args=args)`: Initializes the ROS 2 client library.
- `rclpy.spin_once(minimal_node)`: Processes one set of callbacks (like the logger message) from the node. For continuous operation (e.g., publishing), `rclpy.spin()` would be used.
- `minimal_node.destroy_node()`: Cleans up the node.
- `rclpy.shutdown()`: Shuts down the ROS 2 client library.

**3. Update `setup.py`**

Open `my_ros2_package/setup.py` and add the following inside the `entry_points` dictionary to make your Python script executable as a ROS 2 node:

```python
entry_points={
    'console_scripts': [
        'minimal_node = my_ros2_package.simple_node:main',
    ],
},
```

**4. Build and Run**

Navigate back to your workspace root and build your package:

```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_package
```

Source your workspace (this is important after building):

```bash
source install/setup.bash
```

Now, run your node:

```bash
ros2 run my_ros2_package minimal_node
```

You should see output similar to: `[INFO] [minimal_node]: Minimal node started!`.

### Topics: The Data Pipelines

**Topics** are the primary mechanism for asynchronous, many-to-many communication in ROS 2. Nodes publish data to topics, and other nodes subscribe to those topics to receive the data. This decoupled design means publishers and subscribers don't need to know about each other's existence directly.

**Real-world Examples:**
- A LiDAR sensor node publishes `sensor_msgs/LaserScan` messages to a `/scan` topic.
- A joystick node publishes `geometry_msgs/Twist` messages (velocity commands) to a `/cmd_vel` topic.
- An odometry node publishes `nav_msgs/Odometry` messages to a `/odom` topic.

#### Creating a ROS 2 Publisher (Python)

Building on our previous node, let's create a publisher that sends "Hello, ROS 2!" messages to a topic.

**1. Update `simple_node.py` (or create a new file `publisher_node.py`)**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the String message type

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher_node')
        # Create a publisher that will publish String messages to a topic named 'chatter'
        # The queue size (10) determines how many messages are buffered if subscribers are slow
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        # Create a timer that will call the timer_callback method every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0 # Counter for messages

    def timer_callback(self):
        # Create a new String message
        msg = String()
        msg.data = 'Hello, ROS 2! Count: %d' % self.i # Set the message data
        self.publisher_.publish(msg) # Publish the message
        self.get_logger().info('Publishing: "%s"' % msg.data) # Log that we published
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher) # Keep the node alive, continuously calling callbacks
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**2. Update `setup.py`**

Ensure `setup.py` includes the entry point for `simple_publisher`:

```python
entry_points={
    'console_scripts': [
        'minimal_node = my_ros2_package.simple_node:main', # If you kept minimal_node
        'simple_publisher = my_ros2_package.simple_node:main', # For the publisher
    ],
},
```
:::tip
You can have multiple entry points in `setup.py` mapping different scripts or functions to ROS 2 executables.
:::

**3. Build and Run**

Rebuild your package and source your workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_package
source install/setup.bash
```

Run the publisher node:

```bash
ros2 run my_ros2_package simple_publisher
```

You'll see it publishing messages. To view the messages, you can use the `ros2 topic echo` command in another terminal:

```bash
ros2 topic echo /chatter
```

You can also list active topics:

```bash
ros2 topic list
```

### Services: Request-Response Communication

**Services** provide a synchronous request/response communication model in ROS 2. Unlike topics, where data flows continuously, services are designed for operations that require a specific action to be performed and a result to be returned.

**Real-world Examples:**
- A robotic arm control node offering a service to `move_to_pose`. A client requests the arm to move to a specific position and waits for confirmation.
- A mapping node offering a service to `save_map`. A client requests to save the current map and gets a success/failure response.
- A navigation stack providing a `clear_costmaps` service to reset obstacle data.

#### Creating a ROS 2 Service (Python)

Let's create a service that takes two integers, adds them, and returns the sum.

**1. Define the Service Interface (Srv file)**

First, we need to define the request and response structure for our service. Inside your `my_ros2_package` directory, create a `srv` directory, and inside that, a file named `AddTwoInts.srv`:

```
int64 a
int64 b
---
int64 sum
```

**2. Update `package.xml`**

Add these lines to `package.xml` to declare the service dependency:

```xml
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

**3. Update `CMakeLists.txt`**

In `CMakeLists.txt`, add the following to enable the service generation:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
)
```

**4. Implement the Service Server**

Create a Python file (e.g., `add_two_ints_server.py`) inside `my_ros2_package/my_ros2_package` with the following content:

```python
import rclpy
from rclpy.node import Node
from my_ros2_package.srv import AddTwoInts # Import our custom service type

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        # Create a service that will respond to AddTwoInts requests
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add two ints service ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        self.get_logger().info('Sending back response: [%d]' % response.sum)
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_service = AddTwoIntsService()
    rclpy.spin(add_two_ints_service) # Keep the service node alive
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**5. Update `setup.py`**

Add the entry point for the service server:

```python
entry_points={
    'console_scripts': [
        'minimal_node = my_ros2_package.simple_node:main',
        'simple_publisher = my_ros2_package.simple_node:main',
        'add_two_ints_server = my_ros2_package.add_two_ints_server:main',
    ],
},
```

**6. Build and Run**

Rebuild your package and source your workspace (service interfaces need to be built):

```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_package
source install/setup.bash
```

Run the service server:

```bash
ros2 run my_ros2_package add_two_ints_server
```

In another terminal, you can call the service:

```bash
ros2 service call /add_two_ints my_ros2_package/srv/AddTwoInts "{a: 5, b: 3}"
```

You should see the server logging the request and response, and the client receiving the sum (8).

:::tip
You can list available services using `ros2 service list` and inspect their types with `ros2 service type <service_name>`.
:::

## Real-world Application: Nvidia Jetson

The Nvidia Jetson platform is a popular choice for embedded robotics, and ROS 2 is frequently deployed on it. Jetson devices (like the Jetson Nano, Xavier NX, Orin Nano) provide powerful GPUs for AI inference, making them ideal for tasks such as:

- **Object Detection Nodes:** A node running YOLO or SSD models on camera feeds, publishing bounding box detections to a topic.
- **SLAM (Simultaneous Localization and Mapping) Nodes:** Nodes like Cartographer or RTAB-Map processing LiDAR and camera data to build maps and localize the robot.
- **Autonomous Navigation Stack:** Integrating various ROS 2 nodes for perception, localization, path planning, and motor control to achieve autonomous movement.

These real-world examples showcase how Nodes, Topics, and Services are fundamental building blocks for complex robotic applications on platforms like Nvidia Jetson, enabling sophisticated behaviors through modular and distributed software architectures.
