import requests
import json

# Test data
test_data = {
    "book_id": "1",
    "content_blocks": [
        {
            "id": "intro-01",
            "content": "The Physical AI & Humanoid Robotics course bridges the gap between digital AI and the physical body. Students apply AI knowledge to control Humanoid Robots in simulated and real-world environments using ROS 2, Gazebo, and NVIDIA Isaac.",
            "source_location": "Course Overview",
            "book_id": "1"
        },
        {
            "id": "mod-01",
            "content": "Module 1: The Robotic Nervous System (ROS 2). This module focuses on middleware for robot control, ROS 2 Nodes, Topics, Services, and bridging Python Agents to ROS controllers using rclpy.",
            "source_location": "Module 1 Syllabus",
            "book_id": "1"
        },
        {
            "id": "mod-03",
            "content": "Module 3: The AI-Robot Brain (NVIDIA Isaac). This involves advanced perception using NVIDIA Isaac Sim for photorealistic simulation and Isaac ROS for hardware-accelerated VSLAM and navigation.",
            "source_location": "Module 3 Syllabus",
            "book_id": "1"
        },
        {
            "id": "hard-req",
            "content": "Hardware Requirements: The Digital Twin Workstation requires NVIDIA RTX 4070 Ti (12GB VRAM) or higher because Isaac Sim needs high VRAM to load USD assets. The CPU should be Intel Core i7 (13th Gen+) or AMD Ryzen 9.",
            "source_location": "Hardware Requirements",
            "book_id": "1"
        }
    ]
}

# Make the request
response = requests.post(
    "http://127.0.0.1:8000/api/v1/book-content/index",
    headers={"Content-Type": "application/json"},
    json=test_data
)

print("Status Code:", response.status_code)
print("Response Body:", response.text)