# Quickstart Guide: Digital Twin Development for Robotics Simulation

## Overview
This guide provides a rapid introduction to setting up physics-accurate digital twins using Gazebo and Unity simulation environments. It covers the essential steps to get your simulation environments running with ROS2 integration and basic sensor simulation.

## Prerequisites
- Basic ROS2 knowledge (Humble Hawksbill distribution recommended)
- Linux Ubuntu 22.04 LTS or Windows 10/11
- Familiarity with command line operations
- Programming fundamentals in Python and/or C#

## Setup Steps

### 1. Environment Setup

#### Gazebo Environment
1. Install ROS2 Humble Hawksbill following the official guide
2. Install Gazebo Garden (or Fortress) simulation environment
3. Install ROS-Gazebo bridge packages:
   ```bash
   sudo apt install ros-humble-ros-gz
   ```

#### Unity Environment (Isaac Sim)
1. Install Unity Hub and Unity 2022.3 LTS or later
2. Download NVIDIA Isaac Sim from NVIDIA Developer website
3. Install ROS2 Unity bridge:
   ```bash
   git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
   ```

### 2. Basic Robot Model
1. Create or download a URDF model of your robot
2. Place the model in your ROS2 workspace (`/urdf_examples` directory)
3. Verify the model loads correctly in both Gazebo and Unity

### 3. Basic Simulation

#### In Gazebo:
1. Launch the basic simulation:
   ```bash
   ros2 launch gazebo_ros gazebo.launch.py
   ```
2. Spawn your robot into the simulation:
   ```bash
   ros2 run gazebo_ros spawn_entity.py -entity my_robot -file path/to/robot.urdf
   ```

#### In Unity (Isaac Sim):
1. Open Isaac Sim and load your scene
2. Import your robot model using the URDF Importer
3. Add ROS2 connection components to your robot

### 4. Basic Sensor Integration
1. Add a LiDAR sensor to your URDF:
   ```xml
   <gazebo reference="laser_link">
     <sensor type="ray" name="head_laser">
       <ray>
         <scan>
           <horizontal>
             <samples>640</samples>
             <resolution>1</resolution>
             <min_angle>-1.570796</min_angle>
             <max_angle>1.570796</max_angle>
           </horizontal>
         </scan>
         <range>
           <min>0.10</min>
           <max>30.0</max>
           <resolution>0.01</resolution>
         </range>
       </ray>
       <plugin name="laserscan" filename="libgazebo_ros_ray_sensor.so">
         <ros>
           <namespace>/scan</namespace>
           <remapping>~/out:=scan</remapping>
         </ros>
         <output_type>sensor_msgs/LaserScan</output_type>
       </plugin>
     </sensor>
   </gazebo>
   ```

2. Verify sensor data is publishing to ROS2:
   ```bash
   ros2 topic echo /scan sensor_msgs/msg/LaserScan
   ```

### 5. ROS2 Integration
1. Verify ROS2 nodes are communicating with the simulation:
   ```bash
   ros2 node list
   ros2 topic list
   ```
2. Test basic robot control:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

## Validation
To verify your setup is working:
1. Confirm the robot model appears in both simulation environments
2. Verify sensor data is being published to ROS2 topics
3. Check that you can control the robot using ROS2 commands
4. Validate that physics behave realistically in the simulation

## Next Steps
- Chapter 1: Dive deeper into Gazebo physics simulation
- Chapter 2: Explore high-fidelity rendering in Unity
- Chapter 3: Implement advanced sensor simulation techniques

## Troubleshooting
- If Gazebo doesn't start, verify ROS2 installation and environment variables
- For Unity connection issues, check ROS2 bridge configuration
- If sensors don't appear, verify URDF syntax and plugin configurations
- For performance issues, optimize mesh complexity and physics parameters