---
title: "Week 3-5: ROS 2 Fundamentals – Packages banana, Launch files handle karna, aur parameters management"
sidebar_label: "Week 3-5: ROS 2 Fundamentals"
description: "Creating packages, handling launch files, and managing parameters in ROS 2"
---

# Week 3-5: ROS 2 Fundamentals

## Creating Packages

ROS 2 packages are the basic building blocks of any ROS-based project. A package contains nodes, libraries, and other resources needed for your robot application.

### Creating a Package

To create a new ROS 2 package, use the `ros2 pkg create` command:

```bash
ros2 pkg create --build-type ament_python my_robot_package
```

For C++ packages:
```bash
ros2 pkg create --build-type ament_cmake my_robot_package
```

### Package Structure

A typical ROS 2 Python package has the following structure:

```
my_robot_package/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/my_robot_package
├── my_robot_package/
│   ├── __init__.py
│   └── my_node.py
└── test/
    └── test_copyright.py
```

### package.xml

The `package.xml` file contains metadata about your package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.1.0</version>
  <description>My robot package for ROS 2</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Launch Files

Launch files allow you to start multiple nodes with a single command and configure them with parameters.

### Creating Launch Files

Launch files are Python files that define how to launch your nodes:

```python
# my_robot_package/launch/my_launch_file.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
        
        Node(
            package='my_robot_package',
            executable='my_node',
            name='my_node',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'param1': 'value1'}
            ],
            output='screen'
        ),
        
        Node(
            package='another_package',
            executable='another_node',
            name='another_node'
        )
    ])
```

### Launch File Parameters

Launch files can accept parameters that are passed to nodes:

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# In your generate_launch_description function:
use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time if true'
)

# Use the parameter in a node
Node(
    package='my_robot_package',
    executable='my_node',
    parameters=[
        {'use_sim_time': LaunchConfiguration('use_sim_time')}
    ]
)
```

### Running Launch Files

To run a launch file:
```bash
ros2 launch my_robot_package my_launch_file.py
```

With parameters:
```bash
ros2 launch my_robot_package my_launch_file.py use_sim_time:=true
```

## Parameter Management

ROS 2 provides several ways to manage parameters for your nodes.

### YAML Parameter Files

Create a YAML file to define parameters:

```yaml
# my_robot_package/config/my_params.yaml
my_node:
  ros__parameters:
    param1: 10
    param2: "hello"
    param3: [1.0, 2.0, 3.0]
    use_sim_time: false
```

### Using Parameters in Code

In Python nodes:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare parameters with default values
        self.declare_parameter('param1', 5)
        self.declare_parameter('param2', 'default')
        
        # Get parameter values
        self.param1 = self.get_parameter('param1').value
        self.param2 = self.get_parameter('param2').value
        
        # Set a parameter callback to handle changes
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'param1' and param.type_ == Parameter.Type.INTEGER:
                self.param1 = param.value
        return SetParametersResult(successful=True)
```

### Setting Parameters at Runtime

Parameters can be set at runtime using the command line:

```bash
# Set a parameter on a running node
ros2 param set /my_node param1 42

# Get a parameter value
ros2 param get /my_node param1

# List all parameters for a node
ros2 param list /my_node

# Save parameters to a file
ros2 param dump /my_node > my_params.yaml
```

## Service and Action Integration

### Services

Services allow for request-response communication:

```python
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )
    
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

### Actions

Actions provide goal-based communication with feedback:

```python
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
    
    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
            
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )
            
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Best Practices

1. **Package Organization**: Group related functionality in packages
2. **Launch Files**: Use launch files to start multiple nodes together
3. **Parameter Configuration**: Use YAML files for parameter configuration
4. **Namespace Usage**: Use namespaces to avoid name conflicts
5. **Error Handling**: Implement proper error handling in your nodes
6. **Logging**: Use appropriate logging levels for debugging
7. **Testing**: Write tests for your packages

This covers the fundamentals of ROS 2 packages, launch files, and parameter management.