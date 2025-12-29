# Task List: Content Tagging Implementation

**Feature**: Content Tagging Rules
**Feature Branch**: `008-content-tagging-rules`
**Created**: 2025-12-25

## Overview
This task list outlines the implementation of content tagging rules for the Physical AI & Humanoid Robotics book. The tagging system will add difficulty, category, hardware_focus, and software_focus tags to all content files in the /docs folder.

## Tasks

### Task 1: Update Module 1 (ROS 2) files
**Priority**: P1
**Status**: Pending

**Files to update:**
- module-1/ros2-fundamentals.md
- m1-w3-ros2-architecture.md
- m1-w4-python-agents-rclpy.md
- m1-w5-urdf-humanoids.md

**Requirements:**
- Tag with difficulty level based on week numbers (Beginner for weeks 3-5)
- Tag with category "ROS2"
- Add appropriate hardware and software focus tags
- Preserve existing id, title, and slug values

**Acceptance Criteria:**
- All Module 1 files have appropriate tags added to frontmatter
- Existing metadata remains unchanged
- Tags conform to predefined lists

### Task 2: Update Module 2 (Digital Twin) files
**Priority**: P2
**Status**: Pending

**Files to update:**
- 02-module-2-digital-twin.md
- m2-w6-gazebo-physics-simulation.md
- m2-w7-sensor-simulation-unity.md

**Requirements:**
- Tag with difficulty level based on week numbers (Intermediate for weeks 6-7)
- Tag with category "Simulation"
- Add appropriate hardware and software focus tags
- Preserve existing id, title, and slug values

**Acceptance Criteria:**
- All Module 2 files have appropriate tags added to frontmatter
- Existing metadata remains unchanged
- Tags conform to predefined lists

### Task 3: Update Module 3 (NVIDIA Isaac) files
**Priority**: P2
**Status**: Pending

**Files to update:**
- 03-module-3-isaac.md
- m3-w8-isaac-sim-synthetic-data.md
- m3-w9-isaac-ros-vslam-nav2.md
- m3-w10-rl-sim-to-real.md

**Requirements:**
- Tag with difficulty level based on week numbers (Intermediate for weeks 8-10)
- Tag with category "NVIDIA-Isaac"
- Add appropriate hardware and software focus tags
- Preserve existing id, title, and slug values

**Acceptance Criteria:**
- All Module 3 files have appropriate tags added to frontmatter
- Existing metadata remains unchanged
- Tags conform to predefined lists

### Task 4: Update Module 4 (VLA) files
**Priority**: P2
**Status**: Pending

**Files to update:**
- 04-module-4-vla.md
- m4-w11-humanoid-kinematics-balance.md
- m4-w12-manipulation-hri-design.md
- m4-w13a-conversational-robotics.md
- m4-w13b-capstone-autonomous-humanoid.md

**Requirements:**
- Tag with difficulty level based on week numbers (Advanced for weeks 11-13)
- Tag with category "VLA-AI"
- Add appropriate hardware and software focus tags
- Preserve existing id, title, and slug values

**Acceptance Criteria:**
- All Module 4 files have appropriate tags added to frontmatter
- Existing metadata remains unchanged
- Tags conform to predefined lists

### Task 5: Update remaining content files
**Priority**: P3
**Status**: Pending

**Files to update:**
- m0-w1-2-introduction-to-physical-ai.md
- hardware-requirements.md
- learning-outcomes.md
- assessment-and-capstone.md

**Requirements:**
- Tag with appropriate difficulty level based on content
- Tag with appropriate category
- Add appropriate hardware and software focus tags
- Preserve existing id, title, and slug values

**Acceptance Criteria:**
- All remaining files have appropriate tags added to frontmatter
- Existing metadata remains unchanged
- Tags conform to predefined lists

### Task 6: Create tagging utility script
**Priority**: P1
**Status**: Pending

**Requirements:**
- Create a script that can analyze content and determine appropriate tags
- Implement logic to determine difficulty based on week numbers
- Implement logic to determine category based on content analysis
- Implement logic to identify hardware and software focus tags
- Preserve existing metadata when updating files
- Log errors when encountering malformed files but continue processing

**Acceptance Criteria:**
- Script can process all content files in under 1 minute
- Script correctly applies tags according to the specification
- Script preserves existing metadata
- Script handles malformed files appropriately by skipping and logging

### Task 7: Test tagging implementation
**Priority**: P2
**Status**: Pending

**Requirements:**
- Run tagging script on all content files
- Verify that all files have appropriate tags
- Verify that existing metadata remains unchanged
- Verify that the system handles malformed files correctly

**Acceptance Criteria:**
- 100% of content files properly tagged
- 0% of content files lose existing metadata
- All tags conform to predefined lists
- System correctly handles malformed files