import React from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';
import Link from '@docusaurus/Link';

// Define the metadata for each document
interface DocMetadata {
  id: string;
  title: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  category: string;
  hardware_focus: string[];
  software_focus: string[];
}

// Mock data for document metadata - in a real app, this would come from your docs
const docMetadata: DocMetadata[] = [
  {
    id: 'm0-w1-2-introduction-to-physical-ai',
    title: 'Module 1: Introduction to ROS 2 – Nodes, Topics, aur Services ka theoretical ta\'aruf',
    difficulty: 'beginner',
    category: 'Introduction',
    hardware_focus: [],
    software_focus: ['Ubuntu']
  },
  {
    id: 'm2-w6-gazebo-physics-simulation',
    title: 'Module 2: The Digital Twin (Basics) – Gazebo aur Unity interface ko chalana aur simple environments dekhna',
    difficulty: 'beginner',
    category: 'Simulation',
    hardware_focus: ['RTX-GPU'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'foundations-week-1-2',
    title: 'Week 1-2: Foundations of Physical AI – Digital AI vs Physical AI ka farq aur humanoid robotics ki history',
    difficulty: 'beginner',
    category: 'Introduction',
    hardware_focus: [],
    software_focus: ['Ubuntu']
  },
  {
    id: 'hardware-basic-sensors',
    title: 'Hardware: Basic Sensors – Cameras, LiDAR, aur IMUs kaam kaise karte hain (bagair coding ke)',
    difficulty: 'beginner',
    category: 'Hardware',
    hardware_focus: ['Camera', 'LiDAR', 'IMU'],
    software_focus: []
  },
  {
    id: 'focus-theory',
    title: 'Focus: Theory, Terminology, aur "What is Physical AI?"',
    difficulty: 'beginner',
    category: 'Theory',
    hardware_focus: [],
    software_focus: []
  },
  {
    id: 'm1-w4-python-agents-rclpy',
    title: 'Module 1: Python & ROS 2 – rclpy ke zariye nodes banana aur custom messages create karna',
    difficulty: 'intermediate',
    category: 'ROS2',
    hardware_focus: ['RTX-GPU'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'm1-w5-urdf-humanoids',
    title: 'Module 2: URDF & Simulation – Robots ka digital dhancha (Unified Robot Description Format) banana aur simulation mein sensors active karna',
    difficulty: 'intermediate',
    category: 'ROS2',
    hardware_focus: ['RTX-GPU'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'ros2-fundamentals-week-3-5',
    title: 'Week 3-5: ROS 2 Fundamentals – Packages banana, Launch files handle karna, aur parameters management',
    difficulty: 'intermediate',
    category: 'ROS2',
    hardware_focus: ['RTX-GPU'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'edge-kit-setup',
    title: 'Hardware: Edge Kit Setup – NVIDIA Jetson Orin Nano ko configure karna aur OS install karna',
    difficulty: 'intermediate',
    category: 'Hardware',
    hardware_focus: ['Jetson-Orin'],
    software_focus: ['Ubuntu']
  },
  {
    id: 'focus-implementation',
    title: 'Focus: Implementation, Coding, aur Middleware (Nervous System) setup',
    difficulty: 'intermediate',
    category: 'Implementation',
    hardware_focus: ['RTX-GPU'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'm3-w9-isaac-ros-vslam-nav2',
    title: 'Module 3: NVIDIA Isaac & VSLAM – Hardware-accelerated navigation aur Isaac Sim mein realistic environments banana',
    difficulty: 'advanced',
    category: 'NVIDIA-Isaac',
    hardware_focus: ['RTX-GPU', 'Jetson-Orin'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'm4-w13a-conversational-robotics',
    title: 'Module 4: Vision-Language-Action (VLA) – LLMs (GPT/Whisper) ko robot ke saath jorna taake wo voice commands par kaam kare',
    difficulty: 'advanced',
    category: 'VLA-AI',
    hardware_focus: ['RTX-GPU', 'Jetson-Orin'],
    software_focus: ['Python', 'Ubuntu', 'OpenAI-SDK']
  },
  {
    id: 'humanoid-mechanics-week-8-12',
    title: 'Week 8-12: Humanoid Mechanics – Bipedal locomotion (do pairon par chalna), balance control, aur grasping logic',
    difficulty: 'advanced',
    category: 'Humanoid',
    hardware_focus: ['RTX-GPU', 'Jetson-Orin'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'rtx-workstations-sim-to-real',
    title: 'Hardware: RTX Workstations & Sim-to-Real – 4080/4090 GPUs par training aur phir model ko Unitree G1 ya Go2 robot par deploy karna',
    difficulty: 'advanced',
    category: 'Hardware',
    hardware_focus: ['RTX-GPU', 'Unitree-G1', 'Unitree-Go2'],
    software_focus: ['Python', 'Ubuntu', 'ROS2', 'Isaac-Sim', 'Nav2']
  },
  {
    id: 'focus-advanced',
    title: 'Focus: Advanced Perception, LLM Integration, aur Sim-to-Real transfer',
    difficulty: 'advanced',
    category: 'Advanced',
    hardware_focus: ['RTX-GPU', 'Jetson-Orin'],
    software_focus: ['Python', 'Ubuntu', 'OpenAI-SDK']
  },
  {
    id: 'm1-w3-ros2-architecture',
    title: 'Module 1: ROS 2 Architecture',
    difficulty: 'beginner',
    category: 'ROS2',
    hardware_focus: ['RTX-GPU'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'm2-w7-sensor-simulation-unity',
    title: 'Module 2: Sensor Simulation with Unity',
    difficulty: 'intermediate',
    category: 'Simulation',
    hardware_focus: ['RTX-GPU'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'm3-w8-isaac-sim-synthetic-data',
    title: 'Module 3: Isaac Sim Synthetic Data',
    difficulty: 'intermediate',
    category: 'NVIDIA-Isaac',
    hardware_focus: ['RTX-GPU', 'Jetson-Orin'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'm3-w10-rl-sim-to-real',
    title: 'Module 3: RL Sim to Real',
    difficulty: 'advanced',
    category: 'NVIDIA-Isaac',
    hardware_focus: ['RTX-GPU', 'Jetson-Orin'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'm4-w11-humanoid-kinematics-balance',
    title: 'Module 4: Humanoid Kinematics and Balance',
    difficulty: 'advanced',
    category: 'VLA-AI',
    hardware_focus: ['RTX-GPU', 'Jetson-Orin'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'm4-w12-manipulation-hri-design',
    title: 'Module 4: Manipulation and HRI Design',
    difficulty: 'advanced',
    category: 'VLA-AI',
    hardware_focus: ['RTX-GPU', 'Jetson-Orin'],
    software_focus: ['Python', 'Ubuntu']
  },
  {
    id: 'm4-w13b-capstone-autonomous-humanoid',
    title: 'Capstone: Autonomous Humanoid Robot',
    difficulty: 'advanced',
    category: 'VLA-AI',
    hardware_focus: ['RTX-GPU', 'Jetson-Orin'],
    software_focus: ['Python', 'Ubuntu', 'ROS2', 'Isaac-Sim', 'Nav2']
  }
];

interface PersonalizedDocsListProps {
  className?: string;
}

const PersonalizedDocsList: React.FC<PersonalizedDocsListProps> = ({ className = '' }) => {
  const { personalizationData } = usePersonalization();
  const { preferences } = personalizationData;

  // Filter docs based on user preferences
  const filteredDocs = docMetadata.filter(doc => {
    // Filter by difficulty level according to the requirements:
    // Beginner: only beginner content
    // Intermediate: beginner and intermediate content
    // Advanced: all content
    if (preferences.difficulty === 'beginner' && doc.difficulty !== 'beginner') {
      return false;
    } else if (preferences.difficulty === 'intermediate' && doc.difficulty === 'advanced') {
      return false;
    }

    // Filter by preferred topics if specified
    if (preferences.preferredTopics && preferences.preferredTopics.length > 0) {
      const hasPreferredTopic = preferences.preferredTopics.some(topic =>
        doc.category.includes(topic) ||
        doc.software_focus.some(tech => preferences.preferredTopics.includes(tech)) ||
        doc.hardware_focus.some(hw => preferences.preferredTopics.includes(hw))
      );
      if (!hasPreferredTopic) {
        return false;
      }
    }

    // Filter by content context
    if (preferences.contentContext && !doc.category.toLowerCase().includes(preferences.contentContext.toLowerCase())) {
      return false;
    }

    // Filter by hardware background
    if (preferences.hardwareBackground && preferences.hardwareBackground !== 'none') {
      if (preferences.hardwareBackground === 'beginner' && doc.hardware_focus.length > 0) {
        // For beginners, prefer docs without specific hardware requirements
        return doc.hardware_focus.length === 0;
      } else if (preferences.hardwareBackground === 'advanced' && doc.hardware_focus.length === 0) {
        // For advanced users, prefer docs with hardware focus
        return false;
      }
    }

    return true;
  });

  return (
    <div className={className} style={{
      padding: '20px',
      backgroundColor: '#0f172a',
      border: '1px solid rgba(0, 255, 255, 0.3)',
      borderRadius: '8px',
      boxShadow: '0 0 10px rgba(0, 255, 255, 0.1)'
    }}>
      <ul style={{ listStyle: 'none', padding: 0 }}>
        {filteredDocs.length > 0 ? (
          filteredDocs.map((doc) => (
            <li key={doc.id} style={{
              marginBottom: '15px',
              padding: '15px',
              backgroundColor: 'rgba(15, 23, 42, 0.6)',
              border: '1px solid rgba(0, 255, 255, 0.2)',
              borderRadius: '6px'
            }}>
              <Link to={`/module/${doc.id}`} style={{ color: '#00FFFF', textDecoration: 'none' }}>
                <strong>{doc.title}</strong>
              </Link>
              <div style={{ fontSize: '0.9em', color: '#a5f3fc', marginTop: '8px' }}>
                Difficulty: {doc.difficulty} | Category: {doc.category}
              </div>
            </li>
          ))
        ) : (
          <></>
        )}
      </ul>
    </div>
  );
};

export default PersonalizedDocsList;