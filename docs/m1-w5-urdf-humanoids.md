# Module 1: The Robotic Nervous System (ROS 2)

## Week 5: Understanding URDF, Launch Files, and Parameter Management

### 1. Introduction to URDF (Unified Robot Description Format)

URDF (Unified Robot Description Format) ek XML-based file format hai jo ROS mein robots ki mechanical structure, kinematics, aur visual properties ko describe karne ke liye use hota hai. Ismein links (robot ke rigid parts) aur joints (jo links ko connect karte hain) ki definition shamil hoti hai. URDF ka istemal simulation (Gazebo), visualization (RViz), aur motion planning (MoveIt!) mein hota hai.

**Technical Content:**
- URDF XML structure: `<robot>`, `<link>`, `<joint>` tags.
- Link properties: `<visual>`, `<collision>`, `<inertial>` elements.
- Joint types: `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`.
- Kinematic chains aur tree structure.

**Code Example (XML - Simple URDF Snippet for a Link and Joint):**
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <joint name="base_to_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.5" />
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.5" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <material name="blue" />
  <material name="red" />

</robot>
```

**Visual Aid Placeholder:**
```
[Image/Diagram: A simplified robot model rendered from a URDF file in RViz, highlighting links and joints.]
```

### 2. URDF for Humanoids

Humanoid robots ke liye URDF files significantly complex hoti hain kyunki unki structure mein kayi degrees of freedom (DoF), multiple limbs, aur intricate joint configurations shamil hoti hain. Humanoid URDF mein balance control, manipulation, aur bipedal locomotion ke liye specific properties aur extensions (jaise `transmission` tags `ros2_control` ke liye) shamil hote hain.

**Technical Content:**
- `xacro` (XML Macros) ka istemal complex URDF files ko modular aur readable banane ke liye.
- `transmission` tags: Joints ko hardware interfaces aur controllers se connect karna.
- Multiple visual aur collision geometries for detailed models.
- Joint limits aur safety controllers ki definition.

**Code Example (XACRO Snippet for a Humanoid Leg Joint):**
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_leg">

  <xacro:macro name="leg_segment" params="prefix parent_link">
    <link name="${prefix}_link">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.3" />
        </geometry>
        <material name="grey" />
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.05" length="0.3" />
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 0.15" rpy="0 0 0"/>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${parent_link}_to_${prefix}_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${prefix}_link"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/> <!-- Adjust origin based on parent link -->
      <axis xyz="0 1 0"/>
      <limit lower="${-pi/2}" upper="${pi/2}" effort="50" velocity="5"/>
    </joint>

    <xacro:if value="${ros2_control_enabled}">
      <transmission name="${prefix}_joint_transmission">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="${parent_link}_to_${prefix}_joint">
          <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name="${prefix}_motor">
          <mechanicalReduction>1</mechanicalReduction>
        </actuator>
      </transmission>
    </xacro:if>
  </xacro:macro>

  <!-- Example usage of the macro -->
  <xacro:property name="pi" value="3.14159265359" />
  <xacro:arg name="ros2_control_enabled" default="false" />

  <link name="torso_link"/> <!-- Assuming a torso link exists -->
  <xacro:leg_segment prefix="upper_leg_right" parent_link="torso_link" />

  <!-- More leg segments and other parts of the humanoid would follow -->

  <material name="grey">
    <color rgba="0.7 0.7 0.7 1" />
  </material>

</robot>
```

**Visual Aid Placeholder:**
```
[Image/Diagram: A detailed URDF model of a humanoid robot leg, showing multiple joints and links.]
```

### 3. Launch Files (Orchestrating ROS 2 Components)

Launch files XML ya Python scripts hote hain jo ROS 2 nodes aur other executables ko start karne aur manage karne ke liye use hote hain. Yeh ek complex robotic system ke components ko ek saath launch karne, unke parameters set karne, aur remappings configure karne ka ek structured way provide karte hain.

**Technical Content:**
- `ros2 launch` command ka istemal.
- Python launch files (recommendation for flexibility): `LaunchDescription`, `Node`, `ExecuteProcess`, `OpaqueFunction`, `DeclareLaunchArgument`.
- XML launch files: `<node>`, `<param>`, `<remap>`, `<include>`.
- Conditionals (`IfCondition`, `UnlessCondition`) aur loops launch files mein.

**Code Example (Python - Simple ROS 2 Launch File):**
```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your package's share directory
    my_package_share_dir = get_package_share_directory('my_python_pkg')

    # Define the path to the URDF file (if applicable)
    # urdf_file = os.path.join(my_package_share_dir, 'urdf', 'my_robot.urdf')

    # Example of a robot_state_publisher node (for URDFs)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}] # if using simulation
        # arguments=[urdf_file] # uncomment if you have a URDF file
    )

    # Example of launching your custom Python agent node
    python_agent_node = Node(
        package='my_python_pkg',
        executable='python_agent_node',
        name='agent',
        output='screen',
        parameters=[{'example_param': 'value'}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        python_agent_node,
    ])
```

**Visual Aid Placeholder:**
```
[Image/Diagram: A sequence diagram showing how a launch file starts multiple ROS 2 nodes and sets up their communication.]
```

### 4. Parameter Management

Parameters ROS 2 nodes ko configuration values provide karne ka mechanism hain jo runtime par dynamically change kiye ja sakte hain. Yeh flexibility provide karta hai ke aap node behavior ko code compile kiye bagair adjust kar saken.

**Technical Content:**
- `ros2 param` command-line tool (`list`, `get`, `set`, `dump`).
- Node parameters ko code mein declare aur access karna.
- YAML files ka istemal parameters ko load karne ke liye (`ros2 run ... --ros-args --params-file`).
- `Parameter` class in `rclpy`.

**Code Example (Python - Node with Parameter):**
```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('message_prefix', 'ROS 2 says:')
        self.declare_parameter('publish_frequency', 1.0) # Hz

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.message_prefix = self.get_parameter('message_prefix').get_parameter_value().string_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        self.get_logger().info(f'Node Initialized with prefix: "{self.message_prefix}" and frequency: {self.publish_frequency} Hz')

        # Example timer that uses a parameter
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.timer_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'message_prefix':
                self.message_prefix = param.value.string_value
                self.get_logger().info(f'Parameter 'message_prefix' updated to: "{self.message_prefix}"')
            elif param.name == 'publish_frequency':
                self.publish_frequency = param.value.double_value
                # Recreate timer with new frequency
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / self.publish_frequency, self.timer_callback)
                self.get_logger().info(f'Parameter 'publish_frequency' updated to: {self.publish_frequency} Hz')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        self.get_logger().info(f'{self.message_prefix} Hello from timer!')


def main(args=None):
    rclpy.init(args=args)
    param_node = ParameterNode()
    rclpy.spin(param_node)
    param_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Hardware Context:**
- **RTX GPU**: Simulation environments mein robot parameters (e.g., joint stiffness, damping) ko dynamically adjust karne ke liye parameter management ka istemal kiya ja sakta hai, jahan RTX GPU high-fidelity physics simulations ko handle karta hai.
- **Jetson Orin Nano**: Edge devices par sensors ke calibration parameters ya control loop gains ko update karne ke liye parameter management buhat useful hai, jo robot ki autonomous capabilities ko fine-tune karne mein madad karta hai.