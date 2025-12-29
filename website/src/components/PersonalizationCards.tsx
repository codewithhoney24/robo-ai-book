import React from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';
import { useAuth } from '../contexts/AuthContext';

const PersonalizationCards: React.FC = () => {
  const { personalizationData } = usePersonalization();
  const { user } = useAuth();
  const { preferences } = personalizationData;

  // Map hardware background to readable text
  const getHardwareBackgroundText = (background: string) => {
    switch(background) {
      case 'no_gpu':
        return 'No GPU - Standard laptop';
      case 'rtx_laptop':
        return 'RTX Laptop - RTX 2060-4060';
      case 'rtx_workstation':
        return 'RTX Workstation - RTX 3090/4090';
      case 'jetson_kit':
        return 'Jetson Kit - Jetson Orin';
      case 'cloud':
        return 'Cloud - AWS/Azure';
      default:
        return 'Not specified';
    }
  };

  // Map software background to readable text
  const getSoftwareBackgroundText = (background: string) => {
    switch(background) {
      case 'beginner':
        return 'Beginner - No Python/C++ experience';
      case 'python_intermediate':
        return 'Python Intermediate - Python basics';
      case 'ros2_developer':
        return 'ROS 2 Developer - ROS 1/2 experience';
      case 'ai_robotics_expert':
        return 'AI Robotics Expert - Professional level';
      default:
        return 'Not specified';
    }
  };

  // Map learning style to readable text
  const getLearningStyleText = (style: string) => {
    switch(style) {
      case 'visual':
        return 'Visual Learning';
      case 'textual':
        return 'Textual Learning';
      case 'hands-on':
        return 'Hands-on Learning';
      default:
        return 'Mixed Learning';
    }
  };

  // Map content access to readable text
  const getContentAccessText = (access: string) => {
    switch(access) {
      case 'simulation':
        return 'Simulation First';
      case 'real_hardware':
        return 'Real Hardware First';
      default:
        return 'Mixed Approach';
    }
  };

  return (
    <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(300px, 1fr))', gap: '20px', margin: '20px 0' }}>
      {/* Hardware Background Card */}
      <div style={{
        backgroundColor: 'rgba(15, 23, 42, 0.8)',
        border: '1px solid rgba(0, 255, 255, 0.3)',
        borderRadius: '8px',
        padding: '20px',
        boxShadow: '0 0 10px rgba(0, 255, 255, 0.1)',
        transition: 'transform 0.3s ease, box-shadow 0.3s ease'
      }}
      onMouseEnter={(e) => {
        e.currentTarget.style.transform = 'translateY(-5px)';
        e.currentTarget.style.boxShadow = '0 5px 15px rgba(0, 255, 255, 0.3)';
      }}
      onMouseLeave={(e) => {
        e.currentTarget.style.transform = 'translateY(0)';
        e.currentTarget.style.boxShadow = '0 0 10px rgba(0, 255, 255, 0.1)';
      }}>
        <h3 style={{ color: '#00FFFF', margin: '0 0 10px 0', fontSize: '1.2em' }}>Hardware Background</h3>
        <p style={{ color: '#a5f3fc', margin: '0' }}>
          Content tailored to your hardware background level: <strong>{getHardwareBackgroundText(preferences.hardwareBackground)}</strong>. 
          We provide appropriate explanations and examples based on your experience.
        </p>
      </div>

      {/* Software Background Card */}
      <div style={{
        backgroundColor: 'rgba(15, 23, 42, 0.8)',
        border: '1px solid rgba(0, 255, 255, 0.3)',
        borderRadius: '8px',
        padding: '20px',
        boxShadow: '0 0 10px rgba(0, 255, 255, 0.1)',
        transition: 'transform 0.3s ease, box-shadow 0.3s ease'
      }}
      onMouseEnter={(e) => {
        e.currentTarget.style.transform = 'translateY(-5px)';
        e.currentTarget.style.boxShadow = '0 5px 15px rgba(0, 255, 255, 0.3)';
      }}
      onMouseLeave={(e) => {
        e.currentTarget.style.transform = 'translateY(0)';
        e.currentTarget.style.boxShadow = '0 0 10px rgba(0, 255, 255, 0.1)';
      }}>
        <h3 style={{ color: '#00FFFF', margin: '0 0 10px 0', fontSize: '1.2em' }}>Software Background</h3>
        <p style={{ color: '#a5f3fc', margin: '0' }}>
          Content adapted to your software experience: <strong>{getSoftwareBackgroundText(preferences.softwareBackground)}</strong>. 
          We adjust complexity based on your programming knowledge.
        </p>
      </div>

      {/* Learning Style Card */}
      <div style={{
        backgroundColor: 'rgba(15, 23, 42, 0.8)',
        border: '1px solid rgba(0, 255, 255, 0.3)',
        borderRadius: '8px',
        padding: '20px',
        boxShadow: '0 0 10px rgba(0, 255, 255, 0.1)',
        transition: 'transform 0.3s ease, box-shadow 0.3s ease'
      }}
      onMouseEnter={(e) => {
        e.currentTarget.style.transform = 'translateY(-5px)';
        e.currentTarget.style.boxShadow = '0 5px 15px rgba(0, 255, 255, 0.3)';
      }}
      onMouseLeave={(e) => {
        e.currentTarget.style.transform = 'translateY(0)';
        e.currentTarget.style.boxShadow = '0 0 10px rgba(0, 255, 255, 0.1)';
      }}>
        <h3 style={{ color: '#00FFFF', margin: '0 0 10px 0', fontSize: '1.2em' }}>{getLearningStyleText(preferences.learningStyle)}</h3>
        <p style={{ color: '#a5f3fc', margin: '0' }}>
          Enhanced visual elements, diagrams, and interactive content to support your visual learning preference.
        </p>
      </div>

      {/* Content Access Card */}
      <div style={{
        backgroundColor: 'rgba(15, 23, 42, 0.8)',
        border: '1px solid rgba(0, 255, 255, 0.3)',
        borderRadius: '8px',
        padding: '20px',
        boxShadow: '0 0 10px rgba(0, 255, 255, 0.1)',
        transition: 'transform 0.3s ease, box-shadow 0.3s ease'
      }}
      onMouseEnter={(e) => {
        e.currentTarget.style.transform = 'translateY(-5px)';
        e.currentTarget.style.boxShadow = '0 5px 15px rgba(0, 255, 255, 0.3)';
      }}
      onMouseLeave={(e) => {
        e.currentTarget.style.transform = 'translateY(0)';
        e.currentTarget.style.boxShadow = '0 0 10px rgba(0, 255, 255, 0.1)';
      }}>
        <h3 style={{ color: '#00FFFF', margin: '0 0 10px 0', fontSize: '1.2em' }}>{getContentAccessText(preferences.contentAccess)}</h3>
        <p style={{ color: '#a5f3fc', margin: '0' }}>
          Content and examples focused on simulation environments for safer and more accessible learning.
        </p>
      </div>

      {/* Adaptive Learning Card */}
      <div style={{
        backgroundColor: 'rgba(15, 23, 42, 0.8)',
        border: '1px solid rgba(0, 255, 255, 0.3)',
        borderRadius: '8px',
        padding: '20px',
        boxShadow: '0 0 10px rgba(0, 255, 255, 0.1)',
        transition: 'transform 0.3s ease, box-shadow 0.3s ease'
      }}
      onMouseEnter={(e) => {
        e.currentTarget.style.transform = 'translateY(-5px)';
        e.currentTarget.style.boxShadow = '0 5px 15px rgba(0, 255, 255, 0.3)';
      }}
      onMouseLeave={(e) => {
        e.currentTarget.style.transform = 'translateY(0)';
        e.currentTarget.style.boxShadow = '0 0 10px rgba(0, 255, 255, 0.1)';
      }}>
        <h3 style={{ color: '#00FFFF', margin: '0 0 10px 0', fontSize: '1.2em' }}>Adaptive Learning</h3>
        <p style={{ color: '#a5f3fc', margin: '0' }}>
          Content difficulty automatically adjusts based on your progress and understanding to optimize your learning experience.
        </p>
      </div>

      {/* Personalized Content Card */}
      <div style={{
        backgroundColor: 'rgba(15, 23, 42, 0.8)',
        border: '1px solid rgba(0, 255, 255, 0.3)',
        borderRadius: '8px',
        padding: '20px',
        boxShadow: '0 0 10px rgba(0, 255, 255, 0.1)',
        transition: 'transform 0.3s ease, box-shadow 0.3s ease'
      }}
      onMouseEnter={(e) => {
        e.currentTarget.style.transform = 'translateY(-5px)';
        e.currentTarget.style.boxShadow = '0 5px 15px rgba(0, 255, 255, 0.3)';
      }}
      onMouseLeave={(e) => {
        e.currentTarget.style.transform = 'translateY(0)';
        e.currentTarget.style.boxShadow = '0 0 10px rgba(0, 255, 255, 0.1)';
      }}>
        <h3 style={{ color: '#00FFFF', margin: '0 0 10px 0', fontSize: '1.2em' }}>Personalized Content</h3>
        <p style={{ color: '#a5f3fc', margin: '0' }}>
          Showing {preferences.difficulty} content for your learning background.
        </p>
      </div>
    </div>
  );
};

export default PersonalizationCards;