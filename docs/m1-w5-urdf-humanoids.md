---
title: "Module 2: URDF & Simulation – Robots ka digital dhancha (Unified Robot Description Format) banana aur simulation mein sensors active karna"
sidebar_label: "Module 2: URDF & Simulation"
description: "Creating robot digital structure with URDF and activating sensors in simulation"
---

# Module 2: URDF & Simulation

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including links, joints, and sensors.

## Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define the physical parts of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.2 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## URDF Components

### Links
Links represent the rigid parts of the robot. Each link has:
- Visual properties (how it looks)
- Collision properties (how it interacts with the environment)
- Inertial properties (mass, center of mass, inertia tensor)

### Joints
Joints connect links and define how they can move relative to each other:
- **Fixed**: No movement allowed
- **Revolute**: Rotational movement around an axis
- **Continuous**: Unlimited rotational movement
- **Prismatic**: Linear sliding movement
- **Floating**: 6-DOF movement
- **Planar**: Movement on a plane

## Adding Sensors to URDF

Sensors are represented as special links with sensor plugins:

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.01</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Common Sensor Types in Simulation

### Camera Sensors
- RGB cameras for visual perception
- Depth cameras for 3D information
- Stereo cameras for depth estimation

### LiDAR Sensors
- 2D LiDAR for planar navigation
- 3D LiDAR for full 3D mapping
- Ray-based simulation for accurate measurements

### IMU Sensors
- Accelerometer for linear acceleration
- Gyroscope for angular velocity
- Magnetometer for orientation relative to magnetic north

## URDF Best Practices

1. **Use consistent naming**: Follow a consistent naming convention for links and joints
2. **Validate your URDF**: Use tools like `check_urdf` to validate your URDF
3. **Include inertial properties**: Proper inertial properties are crucial for simulation
4. **Use Xacro for complex robots**: Xacro allows macros and parameterization
5. **Consider visualization**: Make sure your robot looks correct in RViz

## Xacro for Complex Robots

Xacro is an XML macro language that allows you to create more maintainable URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />

  <xacro:macro name="wheel" params="prefix parent xyz">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="${xyz}" rpy="0 ${M_PI/2} 0"/>
      <axis xyz="0 0 1"/>
    </joint>

    <link name="${prefix}_wheel_link">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
  </link>

  <xacro:wheel prefix="front_left" parent="base_link" xyz="0.2 0.2 0"/>
  <xacro:wheel prefix="front_right" parent="base_link" xyz="0.2 -0.2 0"/>
</robot>
```

## Integrating with Gazebo Simulation

To properly simulate your robot in Gazebo, you need to add Gazebo-specific tags to your URDF:

```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
  </plugin>
</gazebo>
```

This completes the setup for simulating your robot with proper kinematics in Gazebo.