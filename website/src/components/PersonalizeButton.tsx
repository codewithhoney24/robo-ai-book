import React, { useState, useEffect, useRef } from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';
import { useAuth } from '../contexts/AuthContext';
import { personalizationService } from '../services/personalizationService';

import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  className?: string;
  onClick?: () => void;
  onPersonalize?: () => void;
  debug?: boolean; // Add debug prop to enable debugging
}

const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({
  className = '',
  onClick,
  onPersonalize,
  debug = false
}) => {
  const context = usePersonalization();
  const authContext = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [expandedCategory, setExpandedCategory] = useState<string | null>(null);
  const [debugInfo, setDebugInfo] = useState<string[]>([]);
  const dropdownRef = useRef<HTMLDivElement>(null);

  if (!context) {
    console.error('PersonalizeButton must be used within a PersonalizationProvider');
    return <div>Personalization is not available</div>;
  }

  const { personalizationData, updatePersonalization, refreshContent } = context;

  // Check if user is authenticated
  const isAuthenticated = !!authContext.user;

  // Get user's actual background from auth context
  const userBackground = authContext.user ? {
    softwareBackground: authContext.user.softwareBackground,
    hardwareBackground: authContext.user.hardwareBackground
  } : null;

  // Function to personalize content using backend API
  const personalizeContentForChapter = async (chapterContent: string, chapterTitle?: string) => {
    if (!authContext.user) {
      console.error('User not authenticated');
      return chapterContent; // Return original content if not authenticated
    }

    if (!userBackground) {
      console.error('User background not available');
      return chapterContent; // Return original content if background is not available
    }

    try {
      debugLog('Sending content for personalization to backend');

      const response = await personalizationService.personalizeContent({
        userId: authContext.user.id,
        content: chapterContent,
        userBackground: {
          softwareBackground: userBackground.softwareBackground as any, // Type assertion to match service interface
          hardwareBackground: userBackground.hardwareBackground as any  // Type assertion to match service interface
        },
        chapterTitle
      });

      debugLog('Received personalized content from backend');
      return response.personalizedContent;
    } catch (error) {
      console.error('Error personalizing content:', error);
      // Return original content if personalization fails
      return chapterContent;
    }
  };

  // Debug logging function
  const debugLog = (message: string) => {
    if (debug) {
      console.log(`[PersonalizeButton] ${message}`);
      setDebugInfo(prev => [...prev.slice(-4), `[${new Date().toLocaleTimeString()}] ${message}`]); // Keep last 5 entries
    }
  };

  const toggleDropdown = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault();
    e.stopPropagation();

    debugLog(`Toggle dropdown clicked. Current state: ${isOpen}`);

    const newState = !isOpen;
    setIsOpen(newState);

    if (!newState) {
      setExpandedCategory(null);
      debugLog('Closed dropdown and reset expanded category');
    } else {
      debugLog('Opened dropdown');
    }

    if (onClick) {
      onClick();
    }
  };

  useEffect(() => {
    debugLog(`Component mounted. Current open state: ${isOpen}`);

    const handleClickOutside = (e: MouseEvent) => {
      debugLog(`Outside click detected. Target: ${e.target}`);

      if (dropdownRef.current && !dropdownRef.current.contains(e.target as Node)) {
        debugLog('Click was outside dropdown, closing it');
        setIsOpen(false);
        setExpandedCategory(null);
      } else {
        debugLog('Click was inside dropdown, keeping it open');
      }
    };

    if (isOpen) {
      debugLog('Adding outside click listener');
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      debugLog('Removing outside click listener');
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  const toggleCategory = (categoryId: string, e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();

    debugLog(`Toggle category: ${categoryId}. Current expanded: ${expandedCategory}`);

    const newState = expandedCategory === categoryId ? null : categoryId;
    setExpandedCategory(newState);
    debugLog(`Set expanded category to: ${newState}`);
  };

  const [showChatKit, setShowChatKit] = useState(false);
  const [chatKitContent, setChatKitContent] = useState<{title: string, content: string} | null>(null);

  // Define content difficulty levels with modules
  const difficultyLevels = {
    beginner: {
      title: "Beginner Level (The Explorer)",
      description: "For users who have an initial interest in robotics and want to understand concepts.",
      modules: [
        { id: 'm0-w1-2-introduction-to-physical-ai', title: 'Module 1: Introduction to ROS 2 – Nodes, Topics, aur Services ka theoretical ta\'aruf' },
        { id: 'm2-w6-gazebo-physics-simulation', title: 'Module 2: The Digital Twin (Basics) – Gazebo aur Unity interface ko chalana aur simple environments dekhna' },
        { id: 'foundations-week-1-2', title: 'Week 1-2: Foundations of Physical AI – Digital AI vs Physical AI ka farq aur humanoid robotics ki history' },
        { id: 'hardware-basic-sensors', title: 'Hardware: Basic Sensors – Cameras, LiDAR, aur IMUs kaam kaise karte hain (bagair coding ke)' },
        { id: 'focus-theory', title: 'Focus: Theory, Terminology, aur "What is Physical AI?"' }
      ]
    },
    intermediate: {
      title: "Intermediate Level (The Builder)",
      description: "For users who know programming (Python) and want to build systems themselves.",
      modules: [
        { id: 'm1-w4-python-agents-rclpy', title: 'Module 1: Python & ROS 2 – rclpy ke zariye nodes banana aur custom messages create karna' },
        { id: 'm1-w5-urdf-humanoids', title: 'Module 2: URDF & Simulation – Robots ka digital dhancha (Unified Robot Description Format) banana aur simulation mein sensors active karna' },
        { id: 'm1-w3-ros2-architecture', title: 'Week 3-5: ROS 2 Fundamentals – Packages banana, Launch files handle karna, aur parameters management' },
        { id: 'edge-kit-setup', title: 'Hardware: Edge Kit Setup – NVIDIA Jetson Orin Nano ko configure karna aur OS install karna' },
        { id: 'focus-implementation', title: 'Focus: Implementation, Coding, aur Middleware (Nervous System) setup' }
      ]
    },
    advanced: {
      title: "Advanced Level (The Engineer)",
      description: "For users who want to work on high-end computing, AI brains, and real humanoid hardware.",
      modules: [
        { id: 'm3-w9-isaac-ros-vslam-nav2', title: 'Module 3: NVIDIA Isaac & VSLAM – Hardware-accelerated navigation aur Isaac Sim mein realistic environments banana' },
        { id: 'm4-w13a-conversational-robotics', title: 'Module 4: Vision-Language-Action (VLA) – LLMs (GPT/Whisper) ko robot ke saath jorna taake wo voice commands par kaam kare' },
        { id: 'm4-w11-humanoid-kinematics-balance', title: 'Week 8-12: Humanoid Mechanics – Bipedal locomotion (do pairon par chalna), balance control, aur grasping logic' },
        { id: 'rtx-workstations-sim-to-real', title: 'Hardware: RTX Workstations & Sim-to-Real – 4080/4090 GPUs par training aur phir model ko Unitree G1 ya Go2 robot par deploy karna' },
        { id: 'focus-advanced', title: 'Focus: Advanced Perception, LLM Integration, aur Sim-to-Real transfer' }
      ]
    }
  };

  const navigateToDifficulty = (level: 'beginner' | 'intermediate' | 'advanced') => {
    // Update the difficulty preference
    if (personalizationData.preferences.difficulty !== level) {
      updatePersonalization({
        preferences: {
          ...personalizationData.preferences,
          difficulty: level
        }
      });
      debugLog(`Updated difficulty to: ${level}`);
    }

    // Close the chatkit UI
    setShowChatKit(false);

    // In a real implementation, this would navigate to content with the specified difficulty
    // For now, we'll simulate navigation by updating the URL hash
    window.location.hash = `#difficulty=${level}`;

    // Refresh content to reflect the updated preferences
    refreshContent();

    // Update the chat kit content to show the appropriate modules for the selected level
    const levelInfo = difficultyLevels[level];
    const modulesList = levelInfo.modules.map(module => `- [${module.title}](/module/${module.id})`).join('\n');

    setChatKitContent({
      title: levelInfo.title,
      content: `${levelInfo.description}\n\nModules for this level:\n${modulesList}\n\nShowing ${level} content for your learning background.`
    });
    setShowChatKit(true);

    // In a real implementation, this would navigate to content with the specified difficulty
    // alert(`Navigating to ${level} level content. In a real implementation, this would take you to content tagged with #${level}`);
  };

  const handleOptionSelect = (option: string, e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();

    debugLog(`Option selected: ${option}`);

    // Define content based on option
    let title = '';
    let content = '';

    switch(option) {
      case 'hardwareBackground':
        // Use the user's actual hardware background from auth context
        const actualHardwareBackground = userBackground?.hardwareBackground || personalizationData.preferences.hardwareBackground;

        // Update the personalization preferences to match the user's actual background
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            hardwareBackground: actualHardwareBackground as any
          }
        });
        debugLog(`Updated hardware background to match user's actual background: ${actualHardwareBackground}`);

        title = "Hardware Background Information";

        // Define content based on the user's actual hardware background
        let hardwareContent = '';
        let hardwareLinks = '';

        // Convert the auth context background to the personalization context format if needed
        const displayHardwareBackground = actualHardwareBackground || personalizationData.preferences.hardwareBackground;

        switch(displayHardwareBackground) {
          case 'no_gpu':
            hardwareContent = `Hardware Background: No GPU - Standard laptop\n\nBased on your hardware setup, you'll see content that focuses on CPU-based processing and cloud alternatives. We'll recommend simulation environments and cloud computing solutions that don't require high-end GPUs.`;
            hardwareLinks = `- [Cloud-Based Robotics Setup](/module/m0-w1-2-introduction-to-physical-ai)\n- [CPU-Optimized Algorithms](/module/m2-w6-gazebo-physics-simulation)\n- [AWS/Azure Robotics Services](/module/hardware-requirements)`;
            break;
          case 'rtx_laptop':
            hardwareContent = `Hardware Background: RTX Laptop - RTX 2060-4060\n\nWith your RTX laptop, you can run moderate GPU-intensive tasks. We'll recommend content that leverages your GPU capabilities for simulation and AI processing, with appropriate VRAM usage guidelines.`;
            hardwareLinks = `- [GPU-Accelerated Simulation](/module/m1-w3-ros2-architecture)\n- [CUDA Programming for Robotics](/module/edge-kit-setup)\n- [Optimizing GPU Usage](/module/m1-w5-urdf-humanoids)`;
            break;
          case 'rtx_workstation':
            hardwareContent = `Hardware Background: RTX Workstation - RTX 3090/4090\n\nWith your high-end workstation, you can run complex simulations and AI models. We'll recommend advanced content that takes advantage of your powerful GPU for complex robotics applications.`;
            hardwareLinks = `- [Advanced Isaac Sim Usage](/module/m3-w9-isaac-ros-vslam-nav2)\n- [Large-Scale AI Models](/module/m4-w11-humanoid-kinematics-balance)\n- [Real-time Perception Systems](/module/m4-w13b-capstone-autonomous-humanoid)`;
            break;
          case 'jetson_kit':
            hardwareContent = `Hardware Background: Jetson Kit - Jetson Orin\n\nWith your Jetson kit, you're focused on edge AI and robotics. We'll recommend content that focuses on deploying models to edge devices and working with embedded systems.`;
            hardwareLinks = `- [Jetson Development Guide](/module/m1-w3-ros2-architecture)\n- [Edge AI Deployment](/module/edge-kit-setup)\n- [Embedded Robotics Systems](/module/m1-w5-urdf-humanoids)`;
            break;
          case 'cloud':
            hardwareContent = `Hardware Background: Cloud - AWS/Azure\n\nWith cloud resources, you have access to scalable computing. We'll recommend content that focuses on cloud robotics, distributed systems, and scalable AI solutions.`;
            hardwareLinks = `- [Cloud Robotics Architecture](/module/m0-w1-2-introduction-to-physical-ai)\n- [Distributed Computing for Robotics](/module/m2-w6-gazebo-physics-simulation)\n- [Scalable AI Services](/module/hardware-requirements)`;
            break;
          default:
            hardwareContent = `Hardware Background: Not specified\n\nWe don't have information about your hardware setup. Please update your profile to get personalized content recommendations based on your hardware capabilities.`;
            hardwareLinks = `- [Update Profile](/profile)\n- [Hardware Requirements](/module/hardware-requirements)\n- [Getting Started](/module/m0-w1-2-introduction-to-physical-ai)`;
            break;
        }

        content = `${hardwareContent}\n\nRecommended resources:\n${hardwareLinks}\n\nYour hardware background determines the type of hardware-related content you see. This affects recommendations for simulation environments, computing requirements, and deployment strategies.`;
        break;
      case 'softwareBackground':
        // Use the user's actual software background from auth context
        const actualSoftwareBackground = userBackground?.softwareBackground || personalizationData.preferences.softwareBackground;

        // Update the personalization preferences to match the user's actual background
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            softwareBackground: actualSoftwareBackground as any
          }
        });
        debugLog(`Updated software background to match user's actual background: ${actualSoftwareBackground}`);

        title = "Software Background Information";

        // Define content based on the user's actual software background
        let softwareContent = '';
        let softwareLinks = '';

        // Convert the auth context background to the personalization context format if needed
        const displaySoftwareBackground = actualSoftwareBackground || personalizationData.preferences.softwareBackground;

        switch(displaySoftwareBackground) {
          case 'beginner':
            softwareContent = `Software Background: Beginner - No Python/C++ experience\n\nBased on your software background, you'll see content that introduces basic programming concepts, Python fundamentals, and simple robotics software architectures. We'll start with foundational concepts before moving to more complex topics.`;
            softwareLinks = `- [Python Basics for Robotics](/module/module-1/ros2-fundamentals)\n- [Introduction to rclpy](/module/m1-w4-python-agents-rclpy)\n- [ROS 2 Concepts Explained](/module/m1-w4-python-agents-rclpy)`;
            break;
          case 'python_intermediate':
            softwareContent = `Software Background: Python Intermediate - Python basics\n\nWith your Python experience, you'll see content that builds on your existing knowledge. We'll focus on ROS 2 Python APIs, more complex programming patterns, and practical robotics applications using Python.`;
            softwareLinks = `- [Advanced Python for ROS 2](/module/m1-w3-ros2-architecture)\n- [Creating Custom Messages](/module/m1-w3-ros2-architecture)\n- [ROS 2 Action Servers](/module/m1-w3-ros2-architecture)`;
            break;
          case 'ros2_developer':
            softwareContent = `Software Background: ROS 2 Developer - ROS 1/2 experience\n\nWith your ROS experience, you'll see content that focuses on advanced ROS 2 features, complex system architectures, and best practices for robotics software development. We'll assume familiarity with ROS concepts and focus on advanced applications.`;
            softwareLinks = `- [Advanced ROS 2 Patterns](/module/m1-w3-ros2-architecture)\n- [Multi-Robot Systems](/module/m3-w9-isaac-ros-vslam-nav2)\n- [ROS 2 Performance Optimization](/module/m4-w11-humanoid-kinematics-balance)`;
            break;
          case 'ai_robotics_expert':
            softwareContent = `Software Background: AI Robotics Expert - Professional level\n\nWith your expert-level experience, you'll see content that focuses on cutting-edge AI techniques in robotics, research-level implementations, and professional-grade system architectures. We'll dive deep into advanced algorithms and implementation strategies.`;
            softwareLinks = `- [Isaac ROS and VSLAM](/module/m3-w9-isaac-ros-vslam-nav2)\n- [Vision-Language-Action Models](/module/m4-w13a-conversational-robotics)\n- [Humanoid Control Systems](/module/m4-w11-humanoid-kinematics-balance)`;
            break;
          default:
            softwareContent = `Software Background: Not specified\n\nWe don't have information about your software background. Please update your profile to get personalized content recommendations based on your programming experience.`;
            softwareLinks = `- [Update Profile](/profile)\n- [Getting Started](/module/m0-w1-2-introduction-to-physical-ai)\n- [ROS 2 Fundamentals](/module/module-1/ros2-fundamentals)`;
            break;
        }

        content = `${softwareContent}\n\nRecommended resources:\n${softwareLinks}\n\nYour software background affects the complexity of software concepts presented in the content. More advanced backgrounds will see deeper software implementation details.`;
        break;
      case 'difficulty':
        // Cycle through difficulty levels: beginner -> intermediate -> advanced -> beginner
        let newDifficulty: 'beginner' | 'intermediate' | 'advanced';
        switch(personalizationData.preferences.difficulty) {
          case 'beginner':
            newDifficulty = 'intermediate';
            break;
          case 'intermediate':
            newDifficulty = 'advanced';
            break;
          case 'advanced':
            newDifficulty = 'beginner';
            break;
          default:
            newDifficulty = 'beginner';
        }

        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            difficulty: newDifficulty
          }
        });
        debugLog(`Updated difficulty to: ${newDifficulty}`);

        // Show ChatKit UI for difficulty
        title = "Content Difficulty Information";

        // Determine content based on difficulty level
        const getModuleByDifficulty = (baseName: string) => {
          switch(newDifficulty) {
            case 'beginner': return `Basic ${baseName}`;
            case 'intermediate': return `Intermediate ${baseName}`;
            case 'advanced': return `Advanced ${baseName}`;
            default: return `Basic ${baseName}`;
          }
        };

        const getTagByDifficulty = () => {
          switch(newDifficulty) {
            case 'beginner': return 'basic';
            case 'intermediate': return 'intermediate';
            case 'advanced': return 'advanced';
            default: return 'basic';
          }
        };

        // Determine navigation algorithm based on difficulty
        let navAlgorithm;
        switch(newDifficulty) {
          case 'beginner':
            navAlgorithm = 'Simple Navigation Algorithms';
            break;
          case 'intermediate':
            navAlgorithm = 'Navigation Algorithms';
            break;
          case 'advanced':
            navAlgorithm = 'Complex Navigation Algorithms';
            break;
          default:
            navAlgorithm = 'Simple Navigation Algorithms';
        }

        content = `Current difficulty level: ${newDifficulty}\n\nContent difficulty determines how complex the material is that you see. Higher difficulty levels include more advanced concepts and assume more background knowledge.\n\nRelated modules:\n- ${getModuleByDifficulty('ROS 2 Concepts')}\n- ${navAlgorithm}\n- ${getModuleByDifficulty('Robot Control Systems')}\n\nTags: #${newDifficulty} #${getTagByDifficulty()} #complexity`;
        break;
      case 'learningStyle':
        const newStyle = personalizationData.preferences.learningStyle === 'visual' ? 'textual' : 'visual';
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            learningStyle: newStyle
          }
        });
        debugLog(`Updated learning style to: ${newStyle}`);

        title = "Learning Style Information";
        content = `Current learning style: ${newStyle}\n\nYour learning style preference affects how content is presented to you. Visual learners see more diagrams and videos, textual learners see more written explanations, and hands-on learners see more practical exercises.\n\nRelated modules:\n- ${newStyle === 'visual' ? 'Visual Programming' : newStyle === 'textual' ? 'Documentation & Theory' : 'Hands-on Labs'}\n\nTags: #${newStyle} #presentation #learning`;
        break;
      case 'advancedContent':
        const newAdvancedContent = !personalizationData.preferences.advancedContent;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            advancedContent: newAdvancedContent
          }
        });
        debugLog(`Updated advanced content to: ${newAdvancedContent}`);

        title = "Advanced Content Information";
        content = `Advanced content visibility: ${newAdvancedContent ? 'ON' : 'OFF'}\n\nWhen enabled, you'll see more advanced materials that go beyond the basics. This includes research papers, complex implementations, and cutting-edge techniques.\n\nRelated modules:\n- Research Papers\n- Advanced Implementations\n- Cutting-Edge Techniques\n\nTags: #advanced #research #complex`;
        break;
      case 'foundationalKnowledge':
        const newFoundationalKnowledge = !personalizationData.preferences.foundationalKnowledge;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            foundationalKnowledge: newFoundationalKnowledge
          }
        });
        debugLog(`Updated foundational knowledge to: ${newFoundationalKnowledge}`);

        title = "Foundational Knowledge Information";
        content = `Foundational knowledge first: ${newFoundationalKnowledge ? 'ON' : 'OFF'}\n\nWhen enabled, content will focus on building fundamental concepts before moving to more advanced topics. This ensures a strong foundation.\n\nRelated modules:\n- Basics First\n- Fundamental Concepts\n- Building Blocks\n\nTags: #foundational #basics #fundamentals`;
        break;
      case 'complexConcepts':
        const newComplexConcepts = !personalizationData.preferences.complexConcepts;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            complexConcepts: newComplexConcepts
          }
        });
        debugLog(`Updated complex concepts to: ${newComplexConcepts}`);

        title = "Complex Concepts Information";
        content = `Complex concepts introduction: ${newComplexConcepts ? 'ON' : 'OFF'}\n\nControls whether complex concepts are introduced gradually or presented in more advanced forms.\n\nRelated modules:\n- Advanced Theory\n- Complex Algorithms\n- Multi-System Integration\n\nTags: #complex #advanced #theory`;
        break;
      case 'progressTracking':
        const newProgressTracking = !personalizationData.preferences.progressTracking;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            progressTracking: newProgressTracking
          }
        });
        debugLog(`Updated progress tracking to: ${newProgressTracking}`);

        title = "Progress Tracking Information";
        content = `Progress tracking: ${newProgressTracking ? 'ON' : 'OFF'}\n\nWhen enabled, your learning progress is tracked to provide personalized recommendations and adjust content difficulty.\n\nRelated modules:\n- Progress Dashboard\n- Learning Analytics\n- Adaptive Recommendations\n\nTags: #tracking #analytics #progress`;
        break;
      case 'moduleSuggestions':
        const newModuleSuggestions = !personalizationData.preferences.moduleSuggestions;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            moduleSuggestions: newModuleSuggestions
          }
        });
        debugLog(`Updated module suggestions to: ${newModuleSuggestions}`);

        title = "Module Suggestions Information";
        content = `Module suggestions: ${newModuleSuggestions ? 'ON' : 'OFF'}\n\nWhen enabled, the system suggests relevant modules based on your learning progress and preferences.\n\nRelated modules:\n- Recommended Learning Paths\n- Personalized Curriculum\n- Adaptive Scheduling\n\nTags: #suggestions #recommendations #learning-path`;
        break;
      case 'contentRecommendation':
        const newContentRecommendation = !personalizationData.preferences.contentRecommendation;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            contentRecommendation: newContentRecommendation
          }
        });
        debugLog(`Updated content recommendation to: ${newContentRecommendation}`);

        title = "Content Recommendation Information";
        content = `Content recommendations: ${newContentRecommendation ? 'ON' : 'OFF'}\n\nWhen enabled, you receive personalized content recommendations based on your learning history and preferences.\n\nRelated modules:\n- Personalized Feed\n- Smart Recommendations\n- Adaptive Content\n\nTags: #recommendations #personalized #adaptive`;
        break;
      case 'learningSpeed':
        const newLearningSpeed = personalizationData.preferences.learningSpeed === 'slow' ? 'fast' :
                                personalizationData.preferences.learningSpeed === 'fast' ? 'normal' : 'slow';
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            learningSpeed: newLearningSpeed
          }
        });
        debugLog(`Updated learning speed to: ${newLearningSpeed}`);

        title = "Learning Speed Information";
        content = `Current learning speed: ${newLearningSpeed}\n\nYour learning speed preference affects the pacing of content delivery. Slower speeds allow more time for comprehension, while faster speeds cover material more quickly.\n\nRelated modules:\n- Adaptive Pacing\n- Personalized Timing\n- Speed Control\n\nTags: #${newLearningSpeed} #pacing #timing`;
        break;
      case 'progressiveDisclosure':
        const newProgressiveDisclosure = !personalizationData.preferences.progressiveDisclosure;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            progressiveDisclosure: newProgressiveDisclosure
          }
        });
        debugLog(`Updated progressive disclosure to: ${newProgressiveDisclosure}`);

        title = "Progressive Disclosure Information";
        content = `Progressive disclosure: ${newProgressiveDisclosure ? 'ON' : 'OFF'}\n\nWhen enabled, complex concepts are revealed gradually as you progress through the material, preventing cognitive overload.\n\nRelated modules:\n- Layered Learning\n- Progressive Concepts\n- Gradual Complexity\n\nTags: #progressive #layered #gradual`;
        break;
      case 'smartAdaptation':
        const newSmartAdaptation = !personalizationData.preferences.smartAdaptation;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            smartAdaptation: newSmartAdaptation
          }
        });
        debugLog(`Updated smart adaptation to: ${newSmartAdaptation}`);

        title = "AI-Powered Adaptation Information";

        if (newSmartAdaptation) {
          content = `AI-powered adaptation: ON\n\nWhen enabled, AI-powered adaptation adjusts content in real-time based on your interactions and performance.\n\nRecommended resources:\n- [Module 3: NVIDIA Isaac & VSLAM – Hardware-accelerated navigation aur Isaac Sim mein realistic environments banana](/module/m3-w9-isaac-ros-vslam-nav2)\n- [Module 4: Vision-Language-Action (VLA) – LLMs (GPT/Whisper) ko robot ke saath jorna taake wo voice commands par kaam kare](/module/m4-w13a-conversational-robotics)\n- [Week 8-12: Humanoid Mechanics – Bipedal locomotion (do pairon par chalna), balance control, aur grasping logic](/module/m4-w11-humanoid-kinematics-balance)\n\nRelated modules:\n- AI-Powered Learning\n- Real-Time Adaptation\n- Smart Content\n\nTags: #ai #adaptation #personalization`;
        } else {
          content = `AI-powered adaptation: OFF\n\nWhen disabled, content is delivered without AI-powered personalization.\n\nRecommended resources:\n- [Traditional Learning Approaches](/module/m0-w1-2-introduction-to-physical-ai)\n- [Static Content Delivery](/module/m1-w4-python-agents-rclpy)\n- [Manual Preference Setting](/module/m2-w6-gazebo-physics-simulation)\n\nRelated modules:\n- Fixed Content Delivery\n- Manual Adjustments\n- Static Materials\n\nTags: #traditional #static #manual`;
        }
        break;
      case 'quizResults':
        const newQuizResults = !personalizationData.preferences.quizResults;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            quizResults: newQuizResults
          }
        });
        debugLog(`Updated quiz results to: ${newQuizResults}`);

        title = "Quiz Results Information";
        content = `Learn from quiz results: ${newQuizResults ? 'ON' : 'OFF'}\n\nWhen enabled, the system analyzes your quiz performance to identify knowledge gaps and adjust content accordingly.\n\nRelated modules:\n- Performance Analysis\n- Gap Identification\n- Adaptive Feedback\n\nTags: #quiz #performance #analysis`;
        break;
      case 'contentAdjustment':
        const newContentAdjustment = !personalizationData.preferences.contentAdjustment;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            contentAdjustment: newContentAdjustment
          }
        });
        debugLog(`Updated content adjustment to: ${newContentAdjustment}`);

        title = "Content Adjustment Information";
        content = `Difficulty-based adjustment: ${newContentAdjustment ? 'ON' : 'OFF'}\n\nWhen enabled, content is automatically adjusted based on your demonstrated difficulty level and performance.\n\nRelated modules:\n- Dynamic Adjustment\n- Performance-Based Adaptation\n- Difficulty Calibration\n\nTags: #adjustment #dynamic #calibration`;
        break;
      case 'targetedHelp':
        const newTargetedHelp = !personalizationData.preferences.targetedHelp;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            targetedHelp: newTargetedHelp
          }
        });
        debugLog(`Updated targeted help to: ${newTargetedHelp}`);

        title = "Targeted Help Information";
        content = `Targeted help for challenges: ${newTargetedHelp ? 'ON' : 'OFF'}\n\nWhen enabled, you receive contextual help when facing difficulties with specific concepts.\n\nRelated modules:\n- Contextual Help\n- Challenge Support\n- Targeted Assistance\n\nTags: #help #support #contextual`;
        break;
      case 'interactionPatterns':
        const newInteractionPatterns = !personalizationData.preferences.interactionPatterns;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            interactionPatterns: newInteractionPatterns
          }
        });
        debugLog(`Updated interaction patterns to: ${newInteractionPatterns}`);

        title = "Interaction Patterns Information";
        content = `Learn from interactions: ${newInteractionPatterns ? 'ON' : 'OFF'}\n\nWhen enabled, the system learns from your interaction patterns to improve personalization.\n\nRelated modules:\n- Interaction Analytics\n- Behavioral Learning\n- Pattern Recognition\n\nTags: #interaction #behavior #analytics`;
        break;
      case 'language':
        // Cycle through available languages: en -> es -> fr -> ur -> en
        let newLanguage: 'en' | 'es' | 'fr' | 'ur' | string;
        switch(personalizationData.language) {
          case 'en':
            newLanguage = 'es'; // English to Spanish
            break;
          case 'es':
            newLanguage = 'fr'; // Spanish to French
            break;
          case 'fr':
            newLanguage = 'ur'; // French to Urdu
            break;
          case 'ur':
            newLanguage = 'en'; // Urdu to English
            break;
          default:
            newLanguage = 'en'; // Default to English
        }

        updatePersonalization({ language: newLanguage });
        debugLog(`Updated language to: ${newLanguage}`);

        title = "Language Selection Information";

        // Define content based on selected language
        let languageContent = '';
        let languageLinks = '';

        switch(newLanguage) {
          case 'ur':
            languageContent = `زبان: اردو\n\nآپ کی منتخب کردہ زبان سیکھنے کے پلیٹ فارم میں تمام مواد کی نمائش کو متاثر کرتی ہے۔`;
            languageLinks = `- [مقدمہ: روبوٹکس میں جسمانی AI](/module/m0-w1-2-introduction-to-physical-ai?lang=ur)\n- [ROS 2 کا تعارف](/module/module-1/ros2-fundamentals?lang=ur)\n- [گیزبو سیمولیشن](/module/m2-w6-gazebo-physics-simulation?lang=ur)`;
            break;
          case 'es':
            languageContent = `Idioma: Español\n\nEl idioma seleccionado afecta la presentación de todo el contenido en la plataforma de aprendizaje.`;
            languageLinks = `- [Introducción: IA Física en Robótica](/module/m0-w1-2-introduction-to-physical-ai?lang=es)\n- [Introducción a ROS 2](/module/module-1/ros2-fundamentals?lang=es)\n- [Simulación con Gazebo](/module/m2-w6-gazebo-physics-simulation?lang=es)`;
            break;
          case 'fr':
            languageContent = `Langue: Français\n\nLa langue sélectionnée affecte la présentation de tout le contenu de la plateforme d'apprentissage.`;
            languageLinks = `- [Introduction: IA Physique en Robotique](/module/m0-w1-2-introduction-to-physical-ai?lang=fr)\n- [Introduction à ROS 2](/module/module-1/ros2-fundamentals?lang=fr)\n- [Simulation Gazebo](/module/m2-w6-gazebo-physics-simulation?lang=fr)`;
            break;
          case 'en':
          default:
            languageContent = `Language: English\n\nYour selected language affects the presentation of all content in the learning platform.`;
            languageLinks = `- [Introduction: Physical AI in Robotics](/module/m0-w1-2-introduction-to-physical-ai?lang=en)\n- [Introduction to ROS 2](/module/module-1/ros2-fundamentals?lang=en)\n- [Gazebo Simulation](/module/m2-w6-gazebo-physics-simulation?lang=en)`;
            break;
        }

        content = `${languageContent}\n\nRecommended resources:\n${languageLinks}\n\nRelated modules:\n- Language Settings\n- Translation Services\n- Localization\n\nTags: #${newLanguage} #language #localization`;
        break;
      case 'contentAccess':
        const newAccess = personalizationData.preferences.contentAccess === 'simulation' ? 'realHardware' : 'simulation';
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            contentAccess: newAccess
          }
        });
        debugLog(`Updated content access to: ${newAccess}`);

        title = "Content Access Information";
        content = `Current access type: ${newAccess}\n\nYour content access preference determines whether you see simulation-based content or real hardware examples.\n\nRelated modules:\n- ${newAccess === 'simulation' ? 'Simulation Environment' : 'Real Hardware'}\n- ${newAccess === 'simulation' ? 'Gazebo Simulation' : 'Physical Robots'}\n\nTags: #${newAccess} #access #environment`;
        break;
      case 'learningGoals':
        const newGoals = personalizationData.preferences.learningGoals === 'research' ? 'hobby' : 'research';
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            learningGoals: newGoals
          }
        });
        debugLog(`Updated learning goals to: ${newGoals}`);

        title = "Learning Goals Information";
        content = `Current learning goals: ${newGoals}\n\nYour learning goals influence the type of content you see, focusing on research, hobby, or professional development.\n\nRelated modules:\n- ${newGoals === 'research' ? 'Academic Research' : newGoals === 'hobby' ? 'Hobby Projects' : 'Professional Skills'}\n\nTags: #${newGoals} #goals #focus`;
        break;
      case 'adaptiveDifficulty':
        const newAdaptive = !personalizationData.preferences.adaptiveDifficulty;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            adaptiveDifficulty: newAdaptive
          }
        });
        debugLog(`Updated adaptive difficulty to: ${newAdaptive}`);

        title = "Adaptive Difficulty Information";
        content = `Adaptive difficulty: ${newAdaptive ? 'ON' : 'OFF'}\n\nWhen enabled, difficulty levels adjust automatically based on your performance and engagement.\n\nRelated modules:\n- Dynamic Difficulty\n- Performance-Based Adaptation\n- Smart Leveling\n\nTags: #adaptive #difficulty #dynamic`;
        break;
      case 'contentContext':
        const newContentContext = personalizationData.preferences.contentContext === 'robotics' ? 'ai' : 'robotics';
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            contentContext: newContentContext
          }
        });
        debugLog(`Updated content context to: ${newContentContext}`);

        title = "Content Context Information";
        content = `Current context: ${newContentContext}\n\nYour content context preference focuses the material on either robotics or AI concepts.\n\nRelated modules:\n- ${newContentContext === 'robotics' ? 'Robotics Fundamentals' : 'AI Concepts'}\n\nTags: #${newContentContext} #context #focus`;
        break;
      case 'complexityLevel':
        const newComplexityLevel = personalizationData.preferences.complexityLevel === 'basic' ? 'complex' :
                                  personalizationData.preferences.complexityLevel === 'complex' ? 'standard' : 'basic';
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            complexityLevel: newComplexityLevel
          }
        });
        debugLog(`Updated complexity level to: ${newComplexityLevel}`);

        title = "Complexity Level Information";
        content = `Current complexity level: ${newComplexityLevel}\n\nYour complexity level preference determines how deep and complex the content explanations are.\n\nRelated modules:\n- ${newComplexityLevel === 'basic' ? 'Basic Concepts' : newComplexityLevel === 'standard' ? 'Intermediate Concepts' : 'Advanced Concepts'}\n\nTags: #${newComplexityLevel} #complexity #depth`;
        break;
      case 'preferredTopics':
        // For simplicity, we'll toggle between empty array and a sample array
        const newPreferredTopics = personalizationData.preferences.preferredTopics.length > 0 ? [] : ['ROS2', 'Gazebo', 'Navigation'];
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            preferredTopics: newPreferredTopics
          }
        });
        debugLog(`Updated preferred topics to: ${JSON.stringify(newPreferredTopics)}`);

        title = "Preferred Topics Information";
        content = `Current preferred topics: ${newPreferredTopics.join(', ')}\n\nYour preferred topics determine which subjects you see more content about.\n\nRelated modules:\n- ${newPreferredTopics.length > 0 ? newPreferredTopics.join('\n- ') : 'All Topics'}\n\nTags: #topics #preferences #interests`;
        break;
      case 'codeExamples':
        const newCodeExamples = !personalizationData.preferences.codeExamples;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            codeExamples: newCodeExamples
          }
        });
        debugLog(`Updated code examples to: ${newCodeExamples}`);

        title = "Code Examples Information";

        if (newCodeExamples) {
          content = `Code examples customization: ON\n\nWhen enabled, content includes more programming examples and code snippets.\n\nRecommended resources:\n- [Module 3: NVIDIA Isaac & VSLAM – Hardware-accelerated navigation aur Isaac Sim mein realistic environments banana](/module/m3-w9-isaac-ros-vslam-nav2?lang=${personalizationData.language})\n- [Module 4: Vision-Language-Action (VLA) – LLMs (GPT/Whisper) ko robot ke saath jorna taake wo voice commands par kaam kare](/module/m4-w13a-conversational-robotics?lang=${personalizationData.language})\n- [Week 8-12: Humanoid Mechanics – Bipedal locomotion (do pairon par chalna), balance control, aur grasping logic](/module/m4-w11-humanoid-kinematics-balance?lang=${personalizationData.language})\n\nRelated modules:\n- Programming Examples\n- Code Walkthroughs\n- Implementation Details\n\nTags: #code #examples #programming`;
        } else {
          content = `Code examples customization: OFF\n\nWhen disabled, content includes fewer programming examples and code snippets.\n\nRecommended resources:\n- [Conceptual Overview](/module/m0-w1-2-introduction-to-physical-ai?lang=${personalizationData.language})\n- [Theory Focus](/module/module-1/ros2-fundamentals?lang=${personalizationData.language})\n- [Non-Programming Content](/module/m2-w6-gazebo-physics-simulation?lang=${personalizationData.language})\n\nRelated modules:\n- Conceptual Learning\n- Theory Focus\n- Non-Programming Content\n\nTags: #theory #concepts #non-code`;
        }
        break;
      case 'quizzesAssessments':
        const newQuizzesAssessments = !personalizationData.preferences.quizzesAssessments;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            quizzesAssessments: newQuizzesAssessments
          }
        });
        debugLog(`Updated quizzes assessments to: ${newQuizzesAssessments}`);

        title = "Quizzes Assessments Information";

        if (newQuizzesAssessments) {
          content = `Personalized quizzes: ON\n\nWhen enabled, you receive personalized quizzes based on your learning progress and preferences.\n\nRecommended resources:\n- [Module 3: NVIDIA Isaac & VSLAM Quiz](/module/m3-w9-isaac-ros-vslam-nav2?lang=${personalizationData.language})\n- [Module 4: Vision-Language-Action Assessment](/module/m4-w13a-conversational-robotics?lang=${personalizationData.language})\n- [Week 8-12: Humanoid Mechanics Quiz](/module/m4-w11-humanoid-kinematics-balance?lang=${personalizationData.language})\n\nRelated modules:\n- Adaptive Quizzes\n- Personalized Assessments\n- Skill Evaluations\n\nTags: #quizzes #assessments #evaluation`;
        } else {
          content = `Personalized quizzes: OFF\n\nWhen disabled, you won't receive personalized quizzes based on your learning progress and preferences.\n\nRecommended resources:\n- [Learning Resources Overview](/module/m0-w1-2-introduction-to-physical-ai?lang=${personalizationData.language})\n- [Study Materials](/module/module-1/ros2-fundamentals?lang=${personalizationData.language})\n- [Reference Documents](/module/m2-w6-gazebo-physics-simulation?lang=${personalizationData.language})\n\nRelated modules:\n- Study Materials\n- Reference Documents\n- Learning Resources\n\nTags: #study #resources #reference`;
        }
        break;
      case 'practiceSessions':
        const newPracticeSessions = !personalizationData.preferences.practiceSessions;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            practiceSessions: newPracticeSessions
          }
        });
        debugLog(`Updated practice sessions to: ${newPracticeSessions}`);

        title = "Practice Sessions Information";
        content = `Adaptive practice sessions: ${newPracticeSessions ? 'ON' : 'OFF'}\n\nWhen enabled, practice sessions adapt to your skill level and learning pace.\n\nRelated modules:\n- Adaptive Practice\n- Skill-Building Exercises\n- Hands-On Labs\n\nTags: #practice #adaptive #exercises`;
        break;
      case 'customExercises':
        const newCustomExercises = !personalizationData.preferences.customExercises;
        updatePersonalization({
          preferences: {
            ...personalizationData.preferences,
            customExercises: newCustomExercises
          }
        });
        debugLog(`Updated custom exercises to: ${newCustomExercises}`);

        title = "Custom Exercises Information";
        content = `Custom exercises: ${newCustomExercises ? 'ON' : 'OFF'}\n\nWhen enabled, you get access to custom exercises tailored to your learning goals.\n\nRelated modules:\n- Custom Exercises\n- Personalized Challenges\n- Goal-Oriented Tasks\n\nTags: #custom #exercises #challenges`;
        break;
      default:
        debugLog(`Option not implemented yet: ${option}`);
        title = "Option Information";
        content = `Option ${option} is not yet fully implemented.`;
        break;
    }

    // Update the chat kit content
    setChatKitContent({ title, content });
    setShowChatKit(true);

    if (onPersonalize) {
      onPersonalize();
    } else {
      // If no specific onPersonalize function is provided, try to personalize the current page content
      // This would typically be handled by a parent component like ChapterPersonalization
      debugLog('No specific onPersonalize function provided, using default personalization');
    }

    // Refresh content to reflect personalization changes
    refreshContent();

    setIsOpen(false);
    setExpandedCategory(null);
    debugLog(`Closed dropdown after option selection`);
  };

  const categories = [
    {
      id: 'content-adaptation',
      title: '📚 Content Adaptation',
      icon: '📚',
      options: [
        {
          id: 'hardwareBackground',
          label: userBackground
            ? `Hardware: ${(userBackground.hardwareBackground as string).replace('_', ' ')}`
            : 'Hardware background level'
        },
        {
          id: 'softwareBackground',
          label: userBackground
            ? `Software: ${(userBackground.softwareBackground as string).replace('_', ' ')}`
            : 'Software background level'
        },
        { id: 'difficulty', label: 'Content difficulty' },

      ]
    },

    {
      id: 'adaptive-learning',
      title: '🎯 Adaptive Learning',
      icon: '🎯',
      options: [

        { id: 'smartAdaptation', label: 'AI-powered adaptation' },



      ]
    },

    {
      id: 'practice-assessment',
      title: '✅ Practice & Assessment',
      icon: '✅',
      options: [
        { id: 'codeExamples', label: 'Code examples customization' },
        { id: 'quizzesAssessments', label: 'Personalized quizzes' },

      ]
    }
  ];

  return (
    <div
      ref={dropdownRef}
      className={`${styles['personalize-dropdown']} ${className}`}
      style={{
        position: 'relative',
        zIndex: 9999,
        pointerEvents: 'auto'
      }}
    >
      <button
        className={`${styles['personalize-button']} ${!isAuthenticated ? styles['locked'] : ''}`}
        onClick={isAuthenticated ? toggleDropdown : () => window.location.href = '/signin'}
        onMouseDown={(e) => {
          debugLog('Mouse down on button');
        }}
        onMouseUp={(e) => {
          debugLog('Mouse up on button');
        }}
        onMouseEnter={(e) => {
          debugLog('Mouse entered button');
        }}
        onMouseLeave={(e) => {
          debugLog('Mouse left button');
        }}
        aria-haspopup="true"
        aria-expanded={isOpen}
        type="button"
        style={{
          pointerEvents: 'auto',
          cursor: isAuthenticated ? 'pointer' : 'not-allowed',
          position: 'relative',
          zIndex: 10000,
          opacity: isAuthenticated ? 1 : 0.6,
          filter: isAuthenticated ? 'none' : 'grayscale(30%)'
        }}
        title={isAuthenticated ? "Enable Personalization" : "Sign in to enable personalization"}
      >
        {isAuthenticated ? "Enable Personalization" : "🔒 Sign in to Enable Personalization"}
      </button>
      {isAuthenticated && isOpen && (
        <div
          className={styles['dropdown-menu']}
          style={{
            zIndex: 10001,
            pointerEvents: 'auto'
          }}
          onMouseEnter={(e) => {
            debugLog('Mouse entered dropdown menu');
          }}
          onMouseLeave={(e) => {
            debugLog('Mouse left dropdown menu');
          }}
        >
          <ul className={styles['category-list']}>
            {categories.map((category) => (
              <li key={category.id} className={styles['category-item']}>
                <button
                  className={`${styles['category-button']} ${expandedCategory === category.id ? styles['expanded'] : ''}`}
                  onClick={(e) => toggleCategory(category.id, e)}
                  type="button"
                  onMouseDown={(e) => {
                    e.preventDefault();
                    debugLog(`Category button clicked: ${category.id}`);
                  }}
                >
                  <span className={styles['category-title']}>
                    {category.icon} {category.title}
                  </span>
                  <span className={styles['arrow']}>
                    {expandedCategory === category.id ? '▼' : '▶'}
                  </span>
                </button>

                {expandedCategory === category.id && (
                  <ul className={styles['sub-options']}>
                    {category.options.map((option) => (
                      <li key={option.id}>
                        <button
                          onClick={(e) => handleOptionSelect(option.id, e)}
                          type="button"
                          className={styles['option-button']}
                          onMouseDown={(e) => {
                            e.preventDefault();
                            debugLog(`Option selected: ${option.id}`);
                          }}
                        >
                          {option.label}
                        </button>
                      </li>
                    ))}
                  </ul>
                )}
              </li>
            ))}
          </ul>
        </div>
      )}
      {showChatKit && chatKitContent && (
        <div
          style={{
            position: 'fixed',
            top: '50%',
            left: '50%',
            transform: 'translate(-50%, -50%)',
            width: '80%',
            maxWidth: '600px',
            height: '70vh',
            backgroundColor: 'white',
            border: '1px solid #ccc',
            borderRadius: '8px',
            zIndex: 10000,
            display: 'flex',
            flexDirection: 'column',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)'
          }}
        >
          <div
            style={{
              padding: '15px',
              backgroundColor: '#f0f0f0',
              borderBottom: '1px solid #ccc',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <h3>{chatKitContent.title}</h3>
            <button
              onClick={() => setShowChatKit(false)}
              style={{
                background: 'none',
                border: 'none',
                fontSize: '1.5rem',
                cursor: 'pointer',
                padding: '0 5px'
              }}
            >
              ×
            </button>
          </div>

          {/* Add difficulty navigation at the top for all options */}
          <div style={{
            padding: '15px',
            backgroundColor: '#f8f9fa',
            borderBottom: '1px solid #ddd',
            display: 'flex',
            justifyContent: 'space-around'
          }}>
            <button
              onClick={() => navigateToDifficulty('beginner')}
              style={{
                padding: '8px 12px',
                backgroundColor: personalizationData.preferences.difficulty === 'beginner' ? '#007bff' : '#e0e0e0',
                color: personalizationData.preferences.difficulty === 'beginner' ? 'white' : 'black',
                border: '1px solid #ccc',
                borderRadius: '4px',
                cursor: 'pointer',
                flex: 1,
                marginRight: '5px'
              }}
            >
              Beginner
            </button>
            <button
              onClick={() => navigateToDifficulty('intermediate')}
              style={{
                padding: '8px 12px',
                backgroundColor: personalizationData.preferences.difficulty === 'intermediate' ? '#007bff' : '#e0e0e0',
                color: personalizationData.preferences.difficulty === 'intermediate' ? 'white' : 'black',
                border: '1px solid #ccc',
                borderRadius: '4px',
                cursor: 'pointer',
                flex: 1,
                margin: '0 5px'
              }}
            >
              Intermediate
            </button>
            <button
              onClick={() => navigateToDifficulty('advanced')}
              style={{
                padding: '8px 12px',
                backgroundColor: personalizationData.preferences.difficulty === 'advanced' ? '#007bff' : '#e0e0e0',
                color: personalizationData.preferences.difficulty === 'advanced' ? 'white' : 'black',
                border: '1px solid #ccc',
                borderRadius: '4px',
                cursor: 'pointer',
                flex: 1,
                marginLeft: '5px'
              }}
            >
              Advanced
            </button>
          </div>

          {/* Add language selection buttons when language option is selected */}
          {chatKitContent.title === 'Language Selection Information' && (
            <div style={{
              padding: '15px',
              backgroundColor: '#f8f9fa',
              borderBottom: '1px solid #ddd',
              display: 'flex',
              justifyContent: 'space-around'
            }}>
              <button
                onClick={() => {
                  // Update language and refresh content
                  updatePersonalization({ language: 'en' });
                  // Update the chat kit content to reflect the new language
                  const newLanguageContent = `Language: English\n\nYour selected language affects the presentation of all content in the learning platform.`;
                  const newLanguageLinks = `- [Introduction: Physical AI in Robotics](/module/m0-w1-2-introduction-to-physical-ai?lang=en)\n- [Introduction to ROS 2](/module/module-1/ros2-fundamentals?lang=en)\n- [Gazebo Simulation](/module/m2-w6-gazebo-physics-simulation?lang=en)`;
                  setChatKitContent({
                    title: 'Language Selection Information',
                    content: `${newLanguageContent}\n\nRecommended resources:\n${newLanguageLinks}\n\nRelated modules:\n- Language Settings\n- Translation Services\n- Localization\n\nTags: en #language #localization`
                  });

                  // Store the selected language in localStorage
                  localStorage.setItem('selectedLanguage', 'en');

                  // Refresh the entire page content to reflect the new language
                  refreshContent();
                }}
                style={{
                  padding: '8px 12px',
                  backgroundColor: personalizationData.language === 'en' ? '#007bff' : '#e0e0e0',
                  color: personalizationData.language === 'en' ? 'white' : 'black',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  flex: 1,
                  marginRight: '5px'
                }}
              >
                English
              </button>
              <button
                onClick={() => {
                  // Update language and refresh content
                  updatePersonalization({ language: 'es' });
                  // Update the chat kit content to reflect the new language
                  const newLanguageContent = `Idioma: Español\n\nEl idioma seleccionado afecta la presentación de todo el contenido en la plataforma de aprendizaje.`;
                  const newLanguageLinks = `- [Introducción: IA Física en Robótica](/module/m0-w1-2-introduction-to-physical-ai?lang=es)\n- [Introducción a ROS 2](/module/module-1/ros2-fundamentals?lang=es)\n- [Simulación con Gazebo](/module/m2-w6-gazebo-physics-simulation?lang=es)`;
                  setChatKitContent({
                    title: 'Language Selection Information',
                    content: `${newLanguageContent}\n\nRecommended resources:\n${newLanguageLinks}\n\nRelated modules:\n- Language Settings\n- Translation Services\n- Localization\n\nTags: es #language #localization`
                  });

                  // Store the selected language in localStorage
                  localStorage.setItem('selectedLanguage', 'es');

                  // Refresh the entire page content to reflect the new language
                  refreshContent();
                }}
                style={{
                  padding: '8px 12px',
                  backgroundColor: personalizationData.language === 'es' ? '#007bff' : '#e0e0e0',
                  color: personalizationData.language === 'es' ? 'white' : 'black',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  flex: 1,
                  margin: '0 5px'
                }}
              >
                Spanish
              </button>
              <button
                onClick={() => {
                  // Update language and refresh content
                  updatePersonalization({ language: 'fr' });
                  // Update the chat kit content to reflect the new language
                  const newLanguageContent = `Langue: Français\n\nLa langue sélectionnée affecte la présentation de tout le contenu de la plateforme d'apprentissage.`;
                  const newLanguageLinks = `- [Introduction: IA Physique en Robotique](/module/m0-w1-2-introduction-to-physical-ai?lang=fr)\n- [Introduction à ROS 2](/module/module-1/ros2-fundamentals?lang=fr)\n- [Simulation Gazebo](/module/m2-w6-gazebo-physics-simulation?lang=fr)`;
                  setChatKitContent({
                    title: 'Language Selection Information',
                    content: `${newLanguageContent}\n\nRecommended resources:\n${newLanguageLinks}\n\nRelated modules:\n- Language Settings\n- Translation Services\n- Localization\n\nTags: fr #language #localization`
                  });

                  // Store the selected language in localStorage
                  localStorage.setItem('selectedLanguage', 'fr');

                  // Refresh the entire page content to reflect the new language
                  refreshContent();
                }}
                style={{
                  padding: '8px 12px',
                  backgroundColor: personalizationData.language === 'fr' ? '#007bff' : '#e0e0e0',
                  color: personalizationData.language === 'fr' ? 'white' : 'black',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  flex: 1,
                  margin: '0 5px'
                }}
              >
                French
              </button>
              <button
                onClick={() => {
                  // Update language and refresh content
                  updatePersonalization({ language: 'ur' });
                  // Update the chat kit content to reflect the new language
                  const newLanguageContent = `زبان: اردو\n\nآپ کی منتخب کردہ زبان سیکھنے کے پلیٹ فارم میں تمام مواد کی نمائش کو متاثر کرتی ہے۔`;
                  const newLanguageLinks = `- [مقدمہ: روبوٹکس میں جسمانی AI](/module/m0-w1-2-introduction-to-physical-ai?lang=ur)\n- [ROS 2 کا تعارف](/module/module-1/ros2-fundamentals?lang=ur)\n- [گیزبو سیمولیشن](/module/m2-w6-gazebo-physics-simulation?lang=ur)`;
                  setChatKitContent({
                    title: 'Language Selection Information',
                    content: `${newLanguageContent}\n\nRecommended resources:\n${newLanguageLinks}\n\nRelated modules:\n- Language Settings\n- Translation Services\n- Localization\n\nTags: ur #language #localization`
                  });

                  // Store the selected language in localStorage
                  localStorage.setItem('selectedLanguage', 'ur');

                  // Refresh the entire page content to reflect the new language
                  refreshContent();
                }}
                style={{
                  padding: '8px 12px',
                  backgroundColor: personalizationData.language === 'ur' ? '#007bff' : '#e0e0e0',
                  color: personalizationData.language === 'ur' ? 'white' : 'black',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  flex: 1,
                  marginLeft: '5px'
                }}
              >
                Urdu
              </button>
            </div>
          )}

          <div style={{
            flex: 1,
            overflowY: 'auto',
            padding: '20px',
            lineHeight: '1.6'
          }}>
            {chatKitContent.content.split('\n').map((line, index) => {
              // Check if the line contains a markdown link - use non-global regex to avoid state issues
              const markdownLinkRegex = /\[(.+?)\]\((.+?)\)/;
              const parts = [];
              let remainingLine = line;
              let processedLength = 0;

              // Process all links in the line
              while (true) {
                const match = remainingLine.match(markdownLinkRegex);
                if (!match) break;

                const matchIndex = match.index || 0;
                const fullMatch = match[0];
                const linkText = match[1];
                const linkUrl = match[2];

                // Add text before the link
                if (matchIndex > 0) {
                  parts.push(remainingLine.substring(0, matchIndex));
                }

                // Add the link
                parts.push(
                  <a
                    key={`${index}-${processedLength + matchIndex}`}
                    href={linkUrl}
                    style={{ color: '#007bff', textDecoration: 'underline' }}
                    onClick={(e) => {
                      e.preventDefault();
                      // Store the selected language in localStorage so the target page can access it
                      localStorage.setItem('selectedLanguage', personalizationData.language);

                      // Update the language in the URL or as a parameter for the target page
                      const newUrl = new URL(linkUrl, window.location.origin);
                      newUrl.searchParams.set('lang', personalizationData.language);

                      // Navigate to the documentation page with the selected language
                      window.location.href = newUrl.toString();
                      // Close the modal after navigation
                      setShowChatKit(false);
                    }}
                  >
                    {linkText}
                  </a>
                );

                // Update remaining line to process after this match
                const advance = matchIndex + fullMatch.length;
                remainingLine = remainingLine.substring(advance);
                processedLength += advance;
              }

              // Add remaining text after the last link
              if (remainingLine) {
                parts.push(remainingLine);
              }

              return (
                <div key={index} style={{ whiteSpace: 'pre-wrap' }}>
                  {parts}
                </div>
              );
            })}
          </div>

          <div
            style={{
              padding: '15px',
              backgroundColor: '#f0f0f0',
              borderTop: '1px solid #ccc',
              textAlign: 'right'
            }}
          >
            <button
              onClick={() => setShowChatKit(false)}
              style={{
                padding: '8px 16px',
                backgroundColor: '#007bff',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer'
              }}
            >
              Close
            </button>
          </div>
        </div>
      )}

      {showChatKit && chatKitContent && (
        <div
          style={{
            position: 'fixed',
            top: 0,
            left: 0,
            width: '100%',
            height: '100%',
            backgroundColor: 'rgba(0,0,0,0.5)',
            zIndex: 9999
          }}
          onClick={() => setShowChatKit(false)}
        />
      )}
      {debug && (
        <div
          className={styles['debug-info']}
          style={{
            position: 'absolute',
            top: '100%',
            left: 0,
            zIndex: 10002,
            backgroundColor: 'rgba(0, 0, 0, 0.8)',
            color: 'white',
            padding: '10px',
            borderRadius: '4px',
            fontSize: '12px',
            maxWidth: '300px',
            marginTop: '5px',
            pointerEvents: 'none'
          }}
        >
          <div style={{ fontWeight: 'bold', marginBottom: '5px' }}>Debug Info:</div>
          {debugInfo.map((log, index) => (
            <div key={index}>{log}</div>
          ))}
        </div>
      )}
    </div>
  );
};

export default PersonalizeButton;