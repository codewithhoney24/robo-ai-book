---
title: "Module 1: Python & ROS 2 – rclpy ke zariye nodes banana aur custom messages create karna"
sidebar_label: "Module 1: Python & ROS 2"
description: "Creating ROS 2 nodes using rclpy and creating custom messages"
---

# Module 1: Python & ROS 2

## Introduction

This module covers how to create ROS 2 nodes using the rclpy library and how to create custom messages.

## Creating ROS 2 Nodes with rclpy

ROS 2 provides the rclpy library to create nodes in Python. Here's how to create a simple publisher and subscriber:

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialize publishers, subscribers, etc. here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Publisher

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Creating a Subscriber

```python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Creating Custom Messages

ROS 2 allows you to define custom message types. These are defined in `.msg` files in the `msg/` directory of your package.

### Example Custom Message

Create a file `CustomMessage.msg`:
```
string name
int32 id
float64 value
bool is_active
```

### Using Custom Messages in Code

```python
from my_package_msgs.msg import CustomMessage

# In your publisher
msg = CustomMessage()
msg.name = "Sensor 1"
msg.id = 123
msg.value = 42.5
msg.is_active = True

# In your subscriber
def listener_callback(self, msg):
    self.get_logger().info(f'Name: {msg.name}, ID: {msg.id}')
```

## Package Structure

A typical ROS 2 Python package looks like:
```
my_package/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── my_package
├── my_package/
│   ├── __init__.py
│   └── my_node.py
├── msg/
│   └── CustomMessage.msg
└── test/
    └── test_copyright.py
```

## Building and Running

To build a package with custom messages:
```bash
colcon build --packages-select my_package my_package_msgs
source install/setup.bash
ros2 run my_package my_node
```

## Best Practices

- Use meaningful names for your nodes and topics
- Always include error handling
- Use appropriate QoS profiles for your use case
- Follow ROS 2 naming conventions
- Document your custom messages clearly