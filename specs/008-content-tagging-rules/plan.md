# Implementation Plan: Content Tagging Rules

**Feature Branch**: `008-content-tagging-rules`
**Created**: 2025-12-25
**Status**: Draft
**Input**: User description: "I want to establish the tagging rules for my 'Physical AI & Humanoid Robotics' book. These tags are for a personalization system. Standard Rules: difficulty: Weeks 1-5 = Beginner, Weeks 6-10 = Intermediate, Weeks 11-13 = Advanced. category: Choose from [Foundations, ROS2, Simulation, NVIDIA-Isaac, Hardware, VLA-AI]. hardware_focus: Identify if chapter needs [RTX-GPU, Jetson-Orin, or RealSense]. software_focus: Identify [Python, Ubuntu, or OpenAI-SDK]. Persistence: Never remove existing id, title, or slug from frontmatter."

## Technical Context

Based on analysis of all Markdown files in the /docs folder, here is the mapping of content to tags according to the specified rules:

### File Tagging Mapping

| File | Week | Difficulty | Category | Hardware Focus | Software Focus |
|------|------|------------|----------|----------------|----------------|
| m0-w1-2-introduction-to-physical-ai.md | Weeks 1-2 | Beginner | Foundations | RTX-GPU, Jetson-Orin | Python, Ubuntu |
| module-1/ros2-fundamentals.md | Week 3 | Beginner | ROS2 | - | Python, Ubuntu |
| m1-w3-ros2-architecture.md | Week 3 | Beginner | ROS2 | - | Python, Ubuntu |
| m1-w4-python-agents-rclpy.md | Week 4 | Beginner | ROS2 | - | Python, Ubuntu |
| m1-w5-urdf-humanoids.md | Week 5 | Beginner | ROS2 | - | Python, Ubuntu |
| 02-module-2-digital-twin.md | Week 6 | Intermediate | Simulation | RTX-GPU | Python, Ubuntu |
| m2-w6-gazebo-physics-simulation.md | Week 6 | Intermediate | Simulation | RTX-GPU | Python, Ubuntu |
| m2-w7-sensor-simulation-unity.md | Week 7 | Intermediate | Simulation | RTX-GPU, RealSense | Python, Ubuntu |
| m3-w8-isaac-sim-synthetic-data.md | Week 8 | Intermediate | NVIDIA-Isaac | RTX-GPU, Jetson-Orin | Python, Ubuntu |
| m3-w9-isaac-ros-vslam-nav2.md | Week 9 | Intermediate | NVIDIA-Isaac | RTX-GPU, Jetson-Orin | Python, Ubuntu |
| m3-w10-rl-sim-to-real.md | Week 10 | Intermediate | NVIDIA-Isaac | RTX-GPU, Jetson-Orin | Python, Ubuntu |
| 03-module-3-isaac.md | Week 8-10 | Intermediate | NVIDIA-Isaac | RTX-GPU, Jetson-Orin | Python, Ubuntu |
| m4-w11-humanoid-kinematics-balance.md | Week 11 | Advanced | Hardware | RTX-GPU, Jetson-Orin | Python, Ubuntu |
| m4-w12-manipulation-hri-design.md | Week 12 | Advanced | Hardware | RTX-GPU, Jetson-Orin | Python, Ubuntu |
| m4-w13a-conversational-robotics.md | Week 13 | Advanced | VLA-AI | RTX-GPU, Jetson-Orin | Python, Ubuntu, OpenAI-SDK |
| m4-w13b-capstone-autonomous-humanoid.md | Week 13 | Advanced | VLA-AI | RTX-GPU, Jetson-Orin | Python, Ubuntu, OpenAI-SDK |
| hardware-requirements.md | - | Beginner | Hardware | RTX-GPU, Jetson-Orin, RealSense | Ubuntu |
| learning-outcomes.md | - | Beginner | Foundations | - | - |
| assessment-and-capstone.md | - | Advanced | VLA-AI | RTX-GPU, Jetson-Orin | Python, Ubuntu, OpenAI-SDK |
| 04-module-4-vla.md | Weeks 11-13 | Advanced | VLA-AI | RTX-GPU, Jetson-Orin | Python, Ubuntu, OpenAI-SDK |

## Constitution Check

### Core Principles Alignment

- **PRINCIPLE_1_NAME**: Library-First - The tagging system will be implemented as a standalone utility that can be applied to content files
- **PRINCIPLE_2_NAME**: CLI Interface - A command-line tool will be created to analyze and tag content files
- **PRINCIPLE_3_NAME**: Test-First - Tests will be written before implementation to validate tagging accuracy
- **PRINCIPLE_4_NAME**: Integration Testing - The tagging system will be tested with actual content files
- **PRINCIPLE_5_NAME**: Observability - The tagging process will include logging to track which tags were applied to which files
- **PRINCIPLE_6_NAME**: Simplicity - The tagging logic will be straightforward and maintainable

## Phase 0: Research

### Research Findings

Based on analysis of the content files, here are the decisions for tagging:

**Decision**: Difficulty levels will be determined primarily by the week numbers in the file names
- Files with weeks 1-5: Beginner
- Files with weeks 6-10: Intermediate  
- Files with weeks 11-13: Advanced
- Files without explicit week numbers will be classified based on content complexity

**Decision**: Categories will be determined by the primary topic of each file
- Files about foundational concepts: Foundations
- Files about ROS2: ROS2
- Files about simulation: Simulation
- Files about NVIDIA Isaac: NVIDIA-Isaac
- Files about hardware specifics: Hardware
- Files about VLA (Vision-Language-Action): VLA-AI

**Decision**: Hardware focus will be determined by content references to specific hardware
- Files mentioning RTX GPUs: RTX-GPU
- Files mentioning Jetson platforms: Jetson-Orin
- Files mentioning RealSense sensors: RealSense

**Decision**: Software focus will be determined by content references to specific software
- Files with Python code examples: Python
- Files requiring Ubuntu setup: Ubuntu
- Files using OpenAI SDK: OpenAI-SDK

## Phase 1: Design

### Data Model

**Content Tag Entity**:
- difficulty: string (Beginner, Intermediate, Advanced)
- category: string (Foundations, ROS2, Simulation, NVIDIA-Isaac, Hardware, VLA-AI)
- hardware_focus: array of strings (RTX-GPU, Jetson-Orin, RealSense)
- software_focus: array of strings (Python, Ubuntu, OpenAI-SDK)
- file_path: string (path to the content file)

**Content File Entity**:
- id: string (existing id from frontmatter)
- title: string (existing title from frontmatter)
- slug: string (existing slug from frontmatter)
- tags: Content Tag (newly added tags)

### API Contracts

**Tagging Tool Interface**:
```
POST /tag-content
Request: { filePath: string, content: string }
Response: { success: boolean, filePath: string, tags: ContentTag, originalMetadata: object }
```

**Analysis Tool Interface**:
```
GET /analyze-content
Request: { filePath: string }
Response: { filePath: string, difficulty: string, category: string, hardwareFocus: string[], softwareFocus: string[] }
```

## Quickstart Guide

1. **Setup**: Install the tagging tool in your development environment
2. **Analyze**: Run the analysis tool on your content directory to see what tags would be applied
3. **Tag**: Execute the tagging process to add tags to content files
4. **Verify**: Review the updated files to ensure tags were applied correctly
5. **Integrate**: Connect the tags to your personalization system

## Implementation Approach

1. Create a tagging utility that reads content files
2. Implement logic to determine difficulty based on week numbers
3. Implement logic to determine category based on content
4. Implement logic to identify hardware and software requirements
5. Update frontmatter with new tags while preserving existing metadata
6. Create validation to ensure tags conform to predefined lists
7. Add logging to track the tagging process