# Data Model: Digital Twin Development Guide for Robotics Simulation

## Overview
This document defines the key entities and their relationships for the Digital Twin Development Guide for Robotics Simulation. The focus is on the conceptual data structures used in the simulation environments rather than persistent storage.

## Key Entities

### 1. Simulation Environment
- **Description**: A virtual representation of the physical world where robots operate
- **Attributes**:
  - environment_type (Gazebo/Unity)
  - physics_parameters (gravity, friction coefficients, etc.)
  - collision_detection_enabled (boolean)
  - environment_objects (list of objects in the environment)
  - lighting_conditions (for Unity environments)
- **Relationships**:
  - Contains multiple Robot entities
  - Contains multiple Object entities
  - Associated with multiple Sensor entities

### 2. Digital Twin
- **Description**: An accurate virtual model of a physical robot or system that mirrors its real-world behavior
- **Attributes**:
  - robot_model_path (URDF/SDF file path)
  - physical_parameters (mass, dimensions, etc.)
  - sensor_configuration (list of sensors attached)
  - behavior_models (kinematic and dynamic models)
  - calibration_data (for sensors and actuators)
- **Relationships**:
  - Associated with one Physical Robot
  - Connected to one Simulation Environment
  - Contains multiple Sensor entities
  - Connected to ROS2 Network

### 3. Sensor Simulation
- **Description**: Virtual implementations of physical sensors (LiDAR, depth camera, IMU) that generate realistic data
- **Attributes**:
  - sensor_type (LiDAR, depth camera, IMU, etc.)
  - noise_parameters (accuracy, drift, etc.)
  - output_format (point cloud, RGB-D, etc.)
  - update_rate (measurements per second)
  - mounting_position (relative to robot frame)
- **Relationships**:
  - Belongs to one Digital Twin
  - Connected to ROS2 Network (publishes sensor data)
  - Associated with one Simulation Environment

### 4. Physics Model
- **Description**: Mathematical representations of physical properties that govern object interactions in simulation
- **Attributes**:
  - gravity_vector (x, y, z components)
  - friction_coefficients (static, dynamic)
  - collision_properties (elasticity, damping)
  - integration_method (ODE solver type)
- **Relationships**:
  - Applied to one Simulation Environment
  - Affects multiple Object entities
  - Affects multiple Robot entities

### 5. ROS2 Integration
- **Description**: Mechanisms for bidirectional communication between simulation environments and ROS2-based systems
- **Attributes**:
  - node_namespace (ROS2 namespace)
  - topics_published (list of topic names)
  - topics_subscribed (list of topic names)
  - message_types (specific ROS2 message types used)
  - connection_parameters (IP, port, etc.)
- **Relationships**:
  - Connects Simulation Environment to ROS2 Network
  - Associated with multiple Digital Twin entities
  - Associated with multiple Sensor Simulation entities

## Relationships and Interactions

### Primary Relationships
1. A Simulation Environment *contains* multiple Digital Twins
2. A Digital Twin *has* multiple Sensor Simulations
3. A Digital Twin *uses* Physics Models for behavior
4. A Digital Twin *connects* to ROS2 Integration
5. Sensor Simulations *publish data* via ROS2 Integration
6. Physics Models *apply to* both Robots and Environment Objects

### State Transitions

For Digital Twin entity:
- **Design**: Initial creation of the digital twin model based on physical robot
- **Calibration**: Adjust parameters to match real-world behavior
- **Validation**: Compare simulation output with real-world data
- **Deployment**: Final version ready for testing and development work

For Sensor Simulation entity:
- **Configuration**: Set initial parameters based on real sensor specs
- **Calibration**: Adjust noise and accuracy parameters to match real data
- **Validation**: Verify output matches real-world sensor behavior
- **Operation**: Generating realistic sensor data during simulation

## Validation Rules

1. Every Digital Twin must be associated with a valid Simulation Environment
2. Every Sensor Simulation must reference a specific Digital Twin
3. Physics Models must be compatible with the Simulation Environment type (Gazebo/Unity)
4. ROS2 Integration must have valid topic names that follow ROS2 naming conventions
5. Sensor Simulation update rates must be within realistic bounds for the sensor type
6. Digital Twin physical parameters must be within plausible ranges for the robot type

## Entity States

### Simulation Environment States
- **Setup**: Environment configuration files created
- **Configured**: Physics parameters set and validated
- **Running**: Simulation is active and responding to inputs
- **Validated**: Output verified against expected behavior

### Digital Twin States
- **Modeled**: Basic virtual representation created
- **Calibrated**: Parameters adjusted for accuracy
- **Validated**: Behavior verified against real-world robot
- **Operational**: Ready for development and testing

## Implementation Notes

### For Gazebo Environments:
- Simulation Environment entity maps to SDF world files
- Physics Model entity uses Gazebo's physics engine configuration
- Sensor Simulation entities use Gazebo's sensor plugins

### For Unity Environments:
- Simulation Environment entity encompasses Unity scenes
- Physics Model entity implemented through Unity's physics engine
- Sensor Simulation entities implemented as Unity components