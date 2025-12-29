import React from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';
import PersonalizeButton from '../components/PersonalizeButton';

const PersonalizationDemo: React.FC = () => {
  const { personalizationData, updatePersonalization } = usePersonalization();
  const { preferences, language } = personalizationData;

  return (
    <div style={{ padding: '20px', maxWidth: '800px', margin: '0 auto' }}>
      <h1>Personalization Demo</h1>
      <p>This page demonstrates how the personalization features work.</p>

      <div style={{ marginBottom: '30px' }}>
        <h2>Personalize Your Experience</h2>
        <PersonalizeButton
          debug={true}
          onPersonalize={() => {
            alert('Personalization updated! The page content will now reflect your preferences.');
          }}
        />
      </div>

      <div style={{ marginBottom: '30px' }}>
        <h2>Current Settings</h2>
        <div style={{ backgroundColor: '#f5f5f5', padding: '15px', borderRadius: '8px' }}>
          <p><strong>Language:</strong> {language}</p>
          <p><strong>Hardware Background:</strong> {preferences.hardwareBackground}</p>
          <p><strong>Difficulty Level:</strong> {preferences.difficulty}</p>
          <p><strong>Learning Style:</strong> {preferences.learningStyle}</p>
          <p><strong>Content Access:</strong> {preferences.contentAccess}</p>
          <p><strong>Learning Goals:</strong> {preferences.learningGoals}</p>
          <p><strong>Adaptive Difficulty:</strong> {preferences.adaptiveDifficulty ? 'Enabled' : 'Disabled'}</p>
          <p><strong>Learning Speed:</strong> {preferences.learningSpeed}</p>
          <p><strong>Progress Tracking:</strong> {preferences.progressTracking ? 'Enabled' : 'Disabled'}</p>
        </div>
      </div>

      <div>
        <h2>How Personalization Affects Content</h2>
        <p>
          Based on your settings, the content on this site adapts to your preferences.
          For example, if you prefer visual learning, we'll show more diagrams and
          interactive elements. If you prefer textual learning, we'll provide more
          detailed written explanations.
        </p>

        <h3>Learning Style: {preferences.learningStyle}</h3>
        {preferences.learningStyle === 'visual' && (
          <div>
            <p>Visual elements, diagrams, and interactive content will be emphasized.</p>
            <div style={{
              backgroundColor: '#e0f7fa',
              padding: '10px',
              borderRadius: '5px',
              border: '1px solid #00bcd4',
              textAlign: 'center',
              margin: '10px 0'
            }}>
              Visual content placeholder
            </div>
          </div>
        )}
        {preferences.learningStyle === 'textual' && (
          <div>
            <p>Detailed written explanations will be provided.</p>
            <div style={{
              backgroundColor: '#e8f5e9',
              padding: '10px',
              borderRadius: '5px',
              border: '1px solid #4caf50',
              margin: '10px 0'
            }}>
              <p>This is an example of a detailed textual explanation. In a real implementation,
              we would provide more comprehensive written content based on your preferences.</p>
            </div>
          </div>
        )}
        {preferences.learningStyle === 'handsOn' && (
          <div>
            <p>Practical exercises and interactive elements will be emphasized.</p>
            <button style={{
              backgroundColor: '#ff9800',
              color: 'white',
              padding: '10px 15px',
              border: 'none',
              borderRadius: '5px',
              cursor: 'pointer'
            }}>
              Interactive Exercise
            </button>
          </div>
        )}
      </div>
    </div>
  );
};

export default PersonalizationDemo;