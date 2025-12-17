# Module 4: Vision-Language-Action (VLA)

## Week 13 (Part B): Capstone Project Requirements and Final Submission Guidelines

### 1. Introduction to the Capstone Project

Capstone Project course ka ultimate assessment hai, jahan students ko 'Physical AI & Humanoid Robotics' mein apni acquired knowledge aur skills ko demonstrate karna hota hai. Is project ka maqsad students ko real-world robotics challenges ko solve karne ka mauqa dena hai, jo course ke modules (ROS 2, Digital Twin, AI-Robot Brain, VLA) se concepts ko integrate karta hai.

**Project Goal:** Design, simulate, aur implement (conceptual ya partial) ek autonomous humanoid robot system jo specific tasks perform kar sake.

### 2. Capstone Project Requirements

Capstone Project mein mukhtalif components shamil honge, jinmein design, implementation, simulation, aur evaluation shamil hain. Har component ko detail mein outline kiya gaya hai taake students ko clear guidelines mil saken.

#### A. Design and Planning (20%)
- **Problem Definition**: Clear problem statement aur project goals.
- **System Architecture**: Robot ke hardware/software architecture ka detailed design (nodes, topics, controllers, sensors).
- **Task Breakdown**: Project ko smaller, manageable tasks mein divide karna.
- **Risk Assessment**: Potential challenges aur unke mitigation strategies.

**Visual Aid Placeholder:**
```
[Image/Diagram: A high-level system architecture diagram for the autonomous humanoid robot, showing interconnected modules and data flow.]
```

#### B. Robot Modeling and Simulation (30%)
- **URDF/SDF Model**: Detailed 3D model of the humanoid robot, including links, joints, and sensors (LIDAR, cameras, IMU).
- **Gazebo/Isaac Sim Environment**: Custom simulation environment for testing the robot, including obstacles, targets, aur different lighting conditions.
- **Sensor Simulation**: Accurate simulation of all onboard sensors with realistic noise models.
- **Physics Simulation**: Validating robot dynamics aur interactions in the simulated environment.

**Code Example (Conceptual - URDF snippet for a Capstone Robot base):**
```xml
<robot name="capstone_humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.5"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.25"/>
      <mass value="20.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <!-- ... more links and joints for arms, legs, head ... -->

  <material name="white" />
</robot>
```

**Visual Aid Placeholder:**
```
[Image/Diagram: A screenshot of the capstone humanoid robot model in Gazebo/Isaac Sim within its custom environment.]
```

#### C. AI-Robot Brain and Control (30%)
- **Perception**: Object detection, segmentation, localization (VSLAM), using simulated sensor data.
- **Navigation**: Autonomous path planning (Nav2) aur obstacle avoidance.
- **Manipulation/Grasping**: Object manipulation tasks, e.g., pick-and-place.
- **Balance Control**: Humanoid locomotion aur stability control.
- **Decision Making**: High-level AI agent for task sequencing aur behavior selection.

**Code Example (Conceptual - Python Agent for High-Level Task Orchestration):**
```python
import rclpy
from rclpy.node import Node
# from capstone_interfaces.srv import PerformTask # Custom service message
# from nav2_msgs.action import NavigateToPose # Nav2 action

class CapstoneAgent(Node):
    def __init__(self):
        super().__init__('capstone_agent')
        # self.task_service = self.create_service(PerformTask, 'perform_task', self.task_callback)
        # self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Capstone Agent Initialized!")

    def task_callback(self, request, response):
        self.get_logger().info(f"Received task request: {request.task_name}")
        if request.task_name == "navigate_to_shelf":
            # Implement navigation logic (conceptual)
            # self.send_navigation_goal(request.target_pose)
            response.success = True
            response.message = "Navigation task initiated."
        elif request.task_name == "pick_object":
            # Implement manipulation logic (conceptual)
            response.success = True
            response.message = "Pick object task initiated."
        else:
            response.success = False
            response.message = "Unknown task."
        return response

    # def send_navigation_goal(self, pose):
    #     goal_msg = NavigateToPose.Goal()
    #     goal_msg.pose = pose
    #     # self.nav_action_client.wait_for_server()
    #     # self.send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
    #     # self.send_goal_future.add_done_callback(self.goal_response_callback)
    #     self.get_logger().info("Sending navigation goal (conceptual).")

    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected :(')
    #         return
    #     self.get_logger().info('Goal accepted :)')
    #     self.get_result_future = goal_handle.get_result_async()
    #     self.get_result_future.add_done_callback(self.get_result_callback)

    # def get_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f'Navigation result: {result.success}')

def main(args=None):
    rclpy.init(args=args)
    agent = CapstoneAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Visual Aid Placeholder:**
```
[Image/Diagram: A flowchart representing the decision-making logic of the autonomous humanoid robot for a given task.]
```

#### D. Evaluation and Presentation (20%)
- **Performance Metrics**: Robot ke behavior aur task completion ko quantify karna (e.g., success rate, completion time, path efficiency).
- **Video Demonstration**: Simulation mein robot ki capabilities ka video demonstration.
- **Technical Report**: Project ka detailed documentation, including design, implementation, results, aur future work.
- **Presentation**: Project ki oral presentation.

**Visual Aid Placeholder:**
```
[Image/Diagram: A graph showing performance metrics (e.g., task success rate over multiple trials) for the capstone robot.]
```

### 3. Final Submission Guidelines

Project ki timely aur complete submission ko ensure karne ke liye detailed guidelines follow karna zaroori hai.

**Submission Components:**
- **Code Repository**: All source code, launch files, URDF/SDF models, aur configuration files.
- **Technical Report**: PDF format mein, detailed description of the project.
- **Presentation Slides**: PDF format mein.
- **Demonstration Video**: MP4 format mein.

**Deadlines:**
- **Design Document Submission**: Week 7 ka end.
- **Mid-term Progress Review**: Week 10 ka end.
- **Final Project Submission**: Week 13 ka end.

**Hardware Context:**
- **RTX GPU**: Capstone project mein high-fidelity simulation (Isaac Sim), complex deep learning models (perception, decision-making), aur sophisticated control algorithms (e.g., DRL, MPC) ko efficiently run karne ke liye RTX GPUs critical hain. Synthetic data generation aur real-time rendering project ki success ke liye buhat zaroori hain.
- **Jetson Orin Nano**: Agar project mein physical robot hardware shamil hai, toh Jetson Orin Nano edge computing ke liye use kiya ja sakta hai. Yeh perception (VSLAM), navigation (Nav2), aur low-level control loops ko on-board handle karne ki power provide karta hai, jo autonomous humanoid robot ke real-time operations ke liye zaroori hai.