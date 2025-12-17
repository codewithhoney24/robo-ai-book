import React from 'react';
import { useAuth } from '../contexts/AuthContext';

// Test component to demonstrate personalization based on user background
const PersonalizationTester: React.FC = () => {
  const { user } = useAuth();

  if (!user) {
    return (
      <div style={{ maxWidth: '800px', margin: '4rem auto', padding: '0 1rem', textAlign: 'center' }}>
        <h2 style={{ color: '#00bab7ff' }}>Personalization Test</h2>
        <p style={{ color: '#adb5bd' }}>Please sign in to test content personalization</p>
      </div>
    );
  }

  return (
    <div style={{ maxWidth: '800px', margin: '4rem auto', padding: '0 1rem' }}>
      <h2 style={{ color: '#00bab7ff', marginBottom: '2rem' }}>Personalization Test</h2>
      
      <div style={{ 
        backgroundColor: '#424949ff', 
        padding: '1.5rem', 
        borderRadius: '8px', 
        marginBottom: '2rem' 
      }}>
        <h3 style={{ color: '#e9ecef', marginBottom: '1rem' }}>Your Profile</h3>
        <p><strong style={{ color: '#00bab7ff' }}>Email:</strong> <span style={{ color: '#adb5bd' }}>{user.email}</span></p>
        <p><strong style={{ color: '#00bab7ff' }}>Name:</strong> <span style={{ color: '#adb5bd' }}>{user.name || 'Not provided'}</span></p>
        <p><strong style={{ color: '#00bab7ff' }}>Software Background:</strong> <span style={{ color: '#adb5bd' }}>{user.softwareBackground}</span></p>
        <p><strong style={{ color: '#00bab7ff' }}>Hardware Background:</strong> <span style={{ color: '#adb5bd' }}>{user.hardwareBackground}</span></p>
      </div>

      <div style={{ 
        backgroundColor: '#424949ff', 
        padding: '1.5rem', 
        borderRadius: '8px' 
      }}>
        <h3 style={{ color: '#e9ecef', marginBottom: '1rem' }}>Personalized Content Examples</h3>
        
        {/* Content for different software backgrounds */}
        <div style={{ marginBottom: '1.5rem' }}>
          <h4 style={{ color: '#00bab7ff', marginBottom: '0.5rem' }}>Software-Specific Content</h4>
          
          {user.softwareBackground === 'beginner' && (
            <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px' }}>
              <h5 style={{ color: '#20c997' }}>Beginner Content</h5>
              <p style={{ color: '#e9ecef' }}>As a beginner, we recommend starting with the basics. Check out our "Getting Started with Python" guide and "Introduction to ROS" series.</p>
            </div>
          )}
          
          {user.softwareBackground === 'python_intermediate' && (
            <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px' }}>
              <h5 style={{ color: '#20c997' }}>Intermediate Content</h5>
              <p style={{ color: '#e9ecef' }}>Since you have Python experience, we'll skip basic Python explanations and focus on ROS 2 API usage and advanced concepts.</p>
            </div>
          )}
          
          {user.softwareBackground === 'ros2_developer' && (
            <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px' }}>
              <h5 style={{ color: '#20c997' }}>Advanced ROS Developer Content</h5>
              <p style={{ color: '#e9ecef' }}>For experienced ROS developers, we provide content on custom message types, advanced navigation, and performance optimization.</p>
            </div>
          )}
          
          {user.softwareBackground === 'ai_robotics_expert' && (
            <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px' }}>
              <h5 style={{ color: '#20c997' }}>Expert Content</h5>
              <p style={{ color: '#e9ecef' }}>As an AI/robotics expert, we offer research papers, custom implementations, and cutting-edge techniques in embodied AI.</p>
            </div>
          )}
        </div>
        
        {/* Content for different hardware backgrounds */}
        <div>
          <h4 style={{ color: '#00bab7ff', marginBottom: '0.5rem' }}>Hardware-Specific Content</h4>
          
          {user.hardwareBackground === 'no_gpu' && (
            <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px' }}>
              <h5 style={{ color: '#20c997' }}>Standard Laptop Content</h5>
              <p style={{ color: '#e9ecef' }}>Based on your hardware, we recommend cloud-based alternatives to Isaac Sim and lightweight implementations for testing.</p>
            </div>
          )}
          
          {user.hardwareBackground === 'rtx_laptop' && (
            <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px' }}>
              <h5 style={{ color: '#20c997' }}>RTX Laptop Content</h5>
              <p style={{ color: '#e9ecef' }}>With your RTX laptop, you can run Isaac Sim locally. We'll provide VRAM optimization tips for your setup.</p>
            </div>
          )}
          
          {user.hardwareBackground === 'rtx_workstation' && (
            <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px' }}>
              <h5 style={{ color: '#20c997' }}>RTX Workstation Content</h5>
              <p style={{ color: '#e9ecef' }}>With your high-end workstation, you have full access to all GPU-intensive features and can run complex simulations.</p>
            </div>
          )}
          
          {user.hardwareBackground === 'jetson_kit' && (
            <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px' }}>
              <h5 style={{ color: '#20c997' }}>Jetson Kit Content</h5>
              <p style={{ color: '#e9ecef' }}>For your Jetson platform, we offer edge AI deployment guides and optimization techniques for embedded robotics.</p>
            </div>
          )}
          
          {user.hardwareBackground === 'cloud' && (
            <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px' }}>
              <h5 style={{ color: '#20c997' }}>Cloud Infrastructure Content</h5>
              <p style={{ color: '#e9ecef' }}>With cloud infrastructure, you can run massive simulations and training runs. We'll provide cost-optimization strategies.</p>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default PersonalizationTester;