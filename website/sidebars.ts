import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    // Dynamic content will be handled by PersonalizedDocsList component
    // Static overview section
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

    // This will be replaced by personalized content in the component
    {
      type: 'html',
      value: '<div id="personalized-content-placeholder">Personalized Content Loading...</div>',
      defaultStyle: true,
    },

    /* =========================
       All Modules (for reference)
    ========================= */
    {
      type: 'category',
      label: 'All Modules (Unfiltered)',
      collapsed: true,
      items: [
        // Module 0
        {
          type: 'category',
          label: 'Module 0: Introduction to Physical AI',
          link: {
            type: 'doc',
            id: 'm0-w1-2-introduction-to-physical-ai',
          },
          items: [
            'm0-w1-2-introduction-to-physical-ai',
            'foundations-week-1-2',
            'focus-theory'
          ],
        },
        // Module 1
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
            'edge-kit-setup',
            'focus-implementation'
          ],
        },
        // Module 2
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
            'hardware-basic-sensors'
          ],
        },
        // Module 3
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
            'rtx-workstations-sim-to-real',
            'focus-advanced'
          ],
        },
        // Module 4
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
        // Capstone
        {
          type: 'category',
          label: 'Capstone: Autonomous Humanoid Robot',
          collapsed: false,
          items: [
            'm4-w13b-capstone-autonomous-humanoid',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
