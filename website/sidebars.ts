import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [

    /* =========================
       Course Overview
    ========================= */
    {
      type: 'category',
      label: 'Course Overview',
      collapsed: false,
      items: [
        'm0-w1-2-introduction-to-physical-ai',
        'learning-outcomes',
        'hardware-requirements',
        'assessment-and-capstone',
      ],
    },

    /* =========================
       Module 0
    ========================= */
    {
      type: 'category',
      label: 'Module 0: Introduction to Physical AI',
      link: {
        type: 'doc',
        id: 'm0-w1-2-introduction-to-physical-ai',
      },
      items: [
        'm0-w1-2-introduction-to-physical-ai'
      ],
    },

    /* =========================
       Module 1 – ROS 2
    ========================= */
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'doc',
        id: 'module-1/ros2-fundamentals',
      },
      items: [
        'm1-w3-ros2-architecture',
        'm1-w4-python-agents-rclpy',
        'm1-w5-urdf-humanoids',
      ],
    },

    /* =========================
       Module 2 – Digital Twin
    ========================= */
    {
      type: 'category',
      label: 'Module 2: Digital Twin & Simulation',
      link: {
        type: 'doc',
        id: 'digital-twin',
      },
      items: [
        'm2-w6-gazebo-physics-simulation',
        'm2-w7-sensor-simulation-unity',
      ],
    },

    /* =========================
       Module 3 – NVIDIA Isaac
    ========================= */
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      link: {
        type: 'doc',
        id: 'module-3-isaac',
      },
      items: [
        'm3-w8-isaac-sim-synthetic-data',
        'm3-w9-isaac-ros-vslam-nav2',
        'm3-w10-rl-sim-to-real',
      ],
    },

    /* =========================
       Module 4 – VLA
    ========================= */
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      link: {
        type: 'doc',
        id: 'module-4-vla',
      },
      items: [
        'm4-w11-humanoid-kinematics-balance',
        'm4-w12-manipulation-hri-design',
        'm4-w13a-conversational-robotics',
      ],
    },

    /* =========================
       Capstone
    ========================= */
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid Robot',
      collapsed: false,
      items: [
        'm4-w13b-capstone-autonomous-humanoid',
      ],
    },

  ],
};

export default sidebars;
