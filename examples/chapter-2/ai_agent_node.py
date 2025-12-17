#!/usr/bin/env python3
"""
AI Agent Node Example - Connects AI decision making to ROS 2 controllers.

This example demonstrates how an AI agent can subscribe to sensor data from a robot
and make decisions that are then published as commands to robot controllers.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import random  # In a real implementation, you might use AI libraries like numpy, tensorflow, etc.


class AIAgentNode(Node):
    """
    An AI agent that processes sensor data and generates robot commands.
    """
    
    def __init__(self):
        super().__init__('ai_agent_node')
        
        # Create subscription to sensor data
        self.sensor_subscription = self.create_subscription(
            String,
            'robot_sensor_data',
            self.sensor_callback,
            10
        )
        self.sensor_subscription  # prevent unused variable warning
        
        # Create publisher for robot commands
        self.command_publisher = self.create_publisher(
            String,
            'robot_commands',
            10
        )
        
        # Create publisher for AI status
        self.status_publisher = self.create_publisher(
            String,
            'ai_agent_status',
            10
        )
        
        # Timer to run AI decision cycle periodically
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.run_ai_cycle)
        
        # Internal state
        self.last_sensor_data = None
        self.robot_state = "normal"  # Could be "normal", "warning", "error"
        
        self.get_logger().info('AI Agent Node initialized')

    def sensor_callback(self, msg):
        """
        Callback function for processing sensor data from the robot.
        """
        self.get_logger().info(f'Received sensor data: {msg.data}')
        self.last_sensor_data = msg.data
        
        # Update robot state based on sensor data if needed
        if "error" in msg.data.lower():
            self.robot_state = "error"
        elif "warning" in msg.data.lower():
            self.robot_state = "warning"
        else:
            self.robot_state = "normal"

    def run_ai_cycle(self):
        """
        Main AI decision-making cycle that runs periodically.
        """
        if self.last_sensor_data is None:
            self.get_logger().info('No sensor data received yet, skipping AI cycle')
            return

        # Simple AI decision logic (in a real implementation, 
        # this would involve more complex processing)
        ai_decision = self.make_decision(self.last_sensor_data)
        
        # Publish the command to the robot
        cmd_msg = String()
        cmd_msg.data = ai_decision
        self.command_publisher.publish(cmd_msg)
        
        # Publish AI status for monitoring
        status_msg = String()
        status_msg.data = f"AI Decision: {ai_decision}, State: {self.robot_state}"
        self.status_publisher.publish(status_msg)
        
        self.get_logger().info(f'Published command: {ai_decision}')

    def make_decision(self, sensor_data):
        """
        AI decision-making logic.
        
        In a real AI agent, this would involve:
        - Processing sensor data through ML models
        - Planning or reasoning about actions
        - Considering goals and constraints
        
        For this example, we'll just implement simple rules.
        """
        # Simple rule-based decision making
        if self.robot_state == "error":
            return "EMERGENCY_STOP"
        elif "obstacle" in sensor_data.lower():
            # If there's an obstacle, decide to turn or stop
            return random.choice(["TURN_LEFT", "TURN_RIGHT", "STOP"])
        elif "target" in sensor_data.lower():
            # If there's a target, move toward it
            return "MOVE_FORWARD"
        elif self.robot_state == "warning":
            # If in warning state, be cautious
            return "SLOW_DOWN"
        else:
            # Default behavior
            return random.choice(["MOVE_FORWARD", "TURN_LEFT", "TURN_RIGHT"])


def main(args=None):
    """
    Main function to run the AI Agent Node.
    """
    rclpy.init(args=args)
    
    ai_agent_node = AIAgentNode()
    
    try:
        rclpy.spin(ai_agent_node)
    except KeyboardInterrupt:
        print("Shutting down AI Agent Node...")
    finally:
        # Destroy the node explicitly
        ai_agent_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()