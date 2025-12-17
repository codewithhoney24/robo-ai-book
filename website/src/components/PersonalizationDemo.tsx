import React from 'react';
import { useAuth } from '../contexts/AuthContext';
import PersonalizedContent from './PersonalizedContent';

const PersonalizationDemo: React.FC = () => {
  const { user } = useAuth();

  return (
    <div style={{ maxWidth: '800px', margin: '4rem auto', padding: '0 1rem' }}>
      <h1 style={{ color: '#00bab7ff', marginBottom: '2rem' }}>Content Personalization Demo</h1>
      
      {user ? (
        <>
          <h2 style={{ color: '#e9ecef', marginBottom: '1rem' }}>Hello, {user.name || user.email}!</h2>
          <p style={{ marginBottom: '2rem', color: '#adb5bd' }}>
            Based on your profile:
          </p>
          
          <div style={{ 
            backgroundColor: '#424949ff', 
            padding: '1.5rem', 
            borderRadius: '8px', 
            marginBottom: '2rem' 
          }}>
            <h3 style={{ color: '#00bab7ff', marginBottom: '1rem' }}>Software Background: {user.softwareBackground}</h3>
            
            <PersonalizedContent forSoftwareBackground={['beginner']}>
              <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px', marginBottom: '1rem' }}>
                <h4 style={{ color: '#20c997' }}>Content for Beginners</h4>
                <p>Here's a detailed explanation with basic concepts and step-by-step instructions to help you get started.</p>
              </div>
            </PersonalizedContent>
            
            <PersonalizedContent forSoftwareBackground={['python_intermediate']}>
              <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px', marginBottom: '1rem' }}>
                <h4 style={{ color: '#20c997' }}>Content for Python Intermediate</h4>
                <p>Content assuming basic Python knowledge. We'll skip basic explanations and focus on more advanced concepts.</p>
              </div>
            </PersonalizedContent>
            
            <PersonalizedContent forSoftwareBackground={['ros2_developer']}>
              <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px', marginBottom: '1rem' }}>
                <h4 style={{ color: '#20c997' }}>Content for ROS 2 Developers</h4>
                <p>Advanced ROS 2 concepts with deep technical details and practical examples for experienced users.</p>
              </div>
            </PersonalizedContent>
            
            <PersonalizedContent forSoftwareBackground={['ai_robotics_expert']}>
              <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px', marginBottom: '1rem' }}>
                <h4 style={{ color: '#20c997' }}>Content for AI Robotics Experts</h4>
                <p>Research-level content with cutting-edge techniques and advanced implementations in AI and robotics.</p>
              </div>
            </PersonalizedContent>
          </div>
          
          <div style={{ 
            backgroundColor: '#424949ff', 
            padding: '1.5rem', 
            borderRadius: '8px' 
          }}>
            <h3 style={{ color: '#00bab7ff', marginBottom: '1rem' }}>Hardware Background: {user.hardwareBackground}</h3>
            
            <PersonalizedContent forHardwareBackground={['no_gpu']}>
              <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px', marginBottom: '1rem' }}>
                <h4 style={{ color: '#20c997' }}>Content for No GPU Users</h4>
                <p>Cloud-based alternatives and lightweight implementations suitable for standard laptops without dedicated GPUs.</p>
              </div>
            </PersonalizedContent>
            
            <PersonalizedContent forHardwareBackground={['rtx_laptop']}>
              <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px', marginBottom: '1rem' }}>
                <h4 style={{ color: '#20c997' }}>Content for RTX Laptop Users</h4>
                <p>Optimized implementations that leverage your RTX capabilities with appropriate VRAM considerations.</p>
              </div>
            </PersonalizedContent>
            
            <PersonalizedContent forHardwareBackground={['rtx_workstation']}>
              <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px', marginBottom: '1rem' }}>
                <h4 style={{ color: '#20c997' }}>Content for RTX Workstation Users</h4>
                <p>Resource-intensive implementations that can fully utilize your high-end GPU capabilities.</p>
              </div>
            </PersonalizedContent>
            
            <PersonalizedContent forHardwareBackground={['jetson_kit']}>
              <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px', marginBottom: '1rem' }}>
                <h4 style={{ color: '#20c997' }}>Content for Jetson Kit Users</h4>
                <p>Edge AI and robotics deployment guides optimized for Jetson platforms with power and performance considerations.</p>
              </div>
            </PersonalizedContent>
            
            <PersonalizedContent forHardwareBackground={['cloud']}>
              <div style={{ backgroundColor: '#343a40', padding: '1rem', borderRadius: '4px', marginBottom: '1rem' }}>
                <h4 style={{ color: '#20c997' }}>Content for Cloud Users</h4>
                <p>Cloud-based AI and robotics workflows with cost optimization and scalable architecture recommendations.</p>
              </div>
            </PersonalizedContent>
          </div>
        </>
      ) : (
        <div style={{ textAlign: 'center', padding: '2rem' }}>
          <p style={{ color: '#adb5bd', marginBottom: '1rem' }}>Please sign in to see personalized content.</p>
        </div>
      )}
    </div>
  );
};

export default PersonalizationDemo;