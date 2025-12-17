#!/usr/bin/env python3
"""
Bidirectional Communication Bridge between AI and ROS Controllers.

This example demonstrates how to implement bidirectional communication patterns
between AI agents and robot controllers using ROS 2 topics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json  # For structured communication


class AIRosBridge(Node):
    """
    A bridge that facilitates bidirectional communication between AI agents 
    and robot controllers.
    """
    
    def __init__(self):
        super().__init__('ai_ros_bridge')
        
        # Subscription for sensor data from robot controllers
        self.sensor_sub = self.create_subscription(
            String,
            'robot_sensors',
            self.sensor_callback,
            10
        )
        
        # Subscription for robot status/feedback
        self.status_sub = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10
        )
        
        # Publisher for commands to robot controllers
        self.command_pub = self.create_publisher(
            String,
            'robot_commands',
            10
        )
        
        # Publisher for AI decisions/requests
        self.ai_decisions_pub = self.create_publisher(
            String,
            'ai_decisions',
            10
        )
        
        # Publisher for feedback to AI agent
        self.feedback_pub = self.create_publisher(
            String,
            'ai_feedback',
            10
        )
        
        # Timer for periodic status updates
        self.timer = self.create_timer(1.0, self.status_report)
        
        # Internal state tracking
        self.robot_data = {
            'sensors': {},
            'status': 'idle',
            'last_command': 'none',
            'last_feedback': 'none',
            'ai_request': 'none'
        }
        
        self.get_logger().info('AI-ROS Bridge initialized')

    def sensor_callback(self, msg):
        """
        Handle incoming sensor data from robot.
        """
        try:
            # Assuming sensor data is in JSON format
            sensor_data = json.loads(msg.data)
            self.robot_data['sensors'] = sensor_data
            self.get_logger().info(f'Updated sensor data: {sensor_data}')
        except json.JSONDecodeError:
            # If not JSON, treat as plain string
            self.robot_data['sensors'] = {'raw': msg.data}
            self.get_logger().info(f'Received sensor data: {msg.data}')
        
        # Process the sensor data and potentially publish feedback to AI
        self.process_sensor_data()

    def status_callback(self, msg):
        """
        Handle incoming status updates from robot.
        """
        self.robot_data['status'] = msg.data
        self.get_logger().info(f'Robot status updated: {msg.data}')
        
        # Send status update to AI agent as feedback
        feedback_msg = String()
        feedback_msg.data = f"STATUS_UPDATE: {msg.data}"
        self.feedback_pub.publish(feed_msg)

    def process_sensor_data(self):
        """
        Process incoming sensor data and potentially communicate with AI.
        """
        # Extract relevant information from sensors
        sensors = self.robot_data['sensors']
        
        # Create a summary for the AI agent
        ai_summary = {
            'timestamp': self.get_clock().now().seconds_nanoseconds(),
            'sensors': sensors,
            'current_status': self.robot_data['status']
        }
        
        # Publish the summary to the AI agent
        summary_msg = String()
        summary_msg.data = json.dumps(ai_summary)
        self.ai_decisions_pub.publish(summary_msg)
        
        self.get_logger().info(f'Sent data to AI agent: {ai_summary}')

    def send_command(self, command):
        """
        Send a command to the robot controllers.
        """
        cmd_msg = String()
        cmd_msg.data = command
        self.command_pub.publish(cmd_msg)
        self.robot_data['last_command'] = command
        
        self.get_logger().info(f'Sent command to robot: {command}')

    def status_report(self):
        """
        Periodically report the bridge's status.
        """
        report = {
            'bridge_status': 'active',
            'robot_status': self.robot_data['status'],
            'last_command': self.robot_data['last_command'],
            'sensor_count': len(self.robot_data['sensors'])
        }
        
        self.get_logger().info(f'Bridge status report: {report}')


def main(args=None):
    """
    Main function to run the AI-ROS Bridge.
    """
    rclpy.init(args=args)
    
    bridge = AIRosBridge()
    
    try:
        # Example: Send a command to the robot periodically
        # In a real implementation, this would likely be triggered by AI decisions
        command_timer = bridge.create_timer(3.0, lambda: bridge.send_command("MOVE_FORWARD"))
        
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("Shutting down AI-ROS Bridge...")
    finally:
        # Destroy the node explicitly
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()