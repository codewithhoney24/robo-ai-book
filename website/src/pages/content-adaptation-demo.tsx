import React from 'react';
import Layout from '@theme/Layout';
import ContentAdaptationSystem from '../components/ContentAdaptationSystem';
import { usePersonalization } from '../contexts/PersonalizationContext';

const ContentAdaptationDemoPage: React.FC = () => {
  const { personalizationData, updatePersonalization } = usePersonalization();
  const { preferences } = personalizationData;

  // Function to update learning style
  const updateLearningStyle = (style: 'visual' | 'textual' | 'handsOn') => {
    updatePersonalization({
      preferences: {
        ...preferences,
        learningStyle: style
      }
    });
  };

  // Function to update difficulty
  const updateDifficulty = (level: 'beginner' | 'intermediate' | 'advanced') => {
    updatePersonalization({
      preferences: {
        ...preferences,
        difficulty: level
      }
    });
  };

  // Function to update preferred topics
  const updatePreferredTopics = (topics: string[]) => {
    updatePersonalization({
      preferences: {
        ...preferences,
        preferredTopics: topics
      }
    });
  };

  return (
    <Layout title="Content Adaptation Demo" description="Demo of content adaptation system">
      <div style={{ padding: '2rem' }}>
        <div style={{ maxWidth: '800px', margin: '0 auto' }}>
          <h1>Content Adaptation System Demo</h1>
          
          <div style={{ 
            border: '1px solid #ccc', 
            padding: '1rem', 
            marginBottom: '2rem',
            backgroundColor: '#f9f9f9',
            borderRadius: '4px'
          }}>
            <h2>Your Current Preferences</h2>
            <p><strong>Learning Style:</strong> {preferences.learningStyle}</p>
            <p><strong>Difficulty Level:</strong> {preferences.difficulty}</p>
            <p><strong>Preferred Topics:</strong> {preferences.preferredTopics.join(', ') || 'None'}</p>
            
            <div style={{ marginTop: '1rem' }}>
              <h3>Update Preferences</h3>
              <div style={{ marginBottom: '1rem' }}>
                <strong>Learning Style:</strong>
                <button 
                  onClick={() => updateLearningStyle('visual')} 
                  style={{ margin: '0 0.5rem', padding: '0.5rem' }}
                >
                  Visual
                </button>
                <button 
                  onClick={() => updateLearningStyle('textual')} 
                  style={{ margin: '0 0.5rem', padding: '0.5rem' }}
                >
                  Textual
                </button>
                <button 
                  onClick={() => updateLearningStyle('handsOn')} 
                  style={{ margin: '0 0.5rem', padding: '0.5rem' }}
                >
                  Hands-On
                </button>
              </div>
              
              <div style={{ marginBottom: '1rem' }}>
                <strong>Difficulty Level:</strong>
                <button 
                  onClick={() => updateDifficulty('beginner')} 
                  style={{ margin: '0 0.5rem', padding: '0.5rem' }}
                >
                  Beginner
                </button>
                <button 
                  onClick={() => updateDifficulty('intermediate')} 
                  style={{ margin: '0 0.5rem', padding: '0.5rem' }}
                >
                  Intermediate
                </button>
                <button 
                  onClick={() => updateDifficulty('advanced')} 
                  style={{ margin: '0 0.5rem', padding: '0.5rem' }}
                >
                  Advanced
                </button>
              </div>
              
              <div>
                <strong>Preferred Topics:</strong>
                <button 
                  onClick={() => updatePreferredTopics(['robotics', 'AI'])} 
                  style={{ margin: '0 0.5rem', padding: '0.5rem' }}
                >
                  Robotics & AI
                </button>
                <button 
                  onClick={() => updatePreferredTopics(['simulation', 'control'])} 
                  style={{ margin: '0 0.5rem', padding: '0.5rem' }}
                >
                  Simulation & Control
                </button>
                <button 
                  onClick={() => updatePreferredTopics([])} 
                  style={{ margin: '0 0.5rem', padding: '0.5rem' }}
                >
                  Clear Topics
                </button>
              </div>
            </div>
          </div>

          <h2>Adaptive Content Examples</h2>
          
          <ContentAdaptationSystem 
            tags={['robotics', 'beginner', 'tutorial']} 
            difficulty="beginner" 
            contentType="visual" 
            topic="ros2-basics"
          >
            <div style={{
              border: '2px solid #007acc',
              borderRadius: '8px',
              padding: '15px',
              margin: '10px 0',
              backgroundColor: '#f0f8ff'
            }}>
              <h3>Beginner Visual Content</h3>
              <p>This content is tagged for beginners with a visual learning style.</p>
              <p>It covers ROS 2 basics and is tagged with 'robotics', 'beginner', and 'tutorial'.</p>
              <p>It should show if your preferences match these tags.</p>
            </div>
          </ContentAdaptationSystem>

          <ContentAdaptationSystem 
            tags={['advanced', 'simulation']} 
            difficulty="advanced" 
            contentType="handsOn" 
            topic="simulation"
          >
            <div style={{
              border: '2px solid #d32f2f',
              borderRadius: '8px',
              padding: '15px',
              margin: '10px 0',
              backgroundColor: '#ffebee'
            }}>
              <h3>Advanced Hands-On Content</h3>
              <p>This content is tagged as advanced with hands-on learning style.</p>
              <p>It covers simulation techniques and is tagged with 'advanced' and 'simulation'.</p>
              <p>It will only show if you prefer advanced content and hands-on learning.</p>
            </div>
          </ContentAdaptationSystem>

          <ContentAdaptationSystem 
            tags={['theory', 'textual']} 
            difficulty="intermediate" 
            contentType="textual" 
            topic="kinematics"
          >
            <div style={{
              border: '2px solid #388e3c',
              borderRadius: '8px',
              padding: '15px',
              margin: '10px 0',
              backgroundColor: '#e8f5e8'
            }}>
              <h3>Intermediate Textual Content</h3>
              <p>This content is tagged as intermediate with textual learning style.</p>
              <p>It covers kinematics theory and is tagged with 'theory' and 'textual'.</p>
              <p>It will appear based on your learning preferences.</p>
            </div>
          </ContentAdaptationSystem>
          
          <ContentAdaptationSystem 
            tags={['programming', 'code']} 
            difficulty="intermediate" 
            contentType="handsOn" 
            topic="programming"
          >
            <div style={{
              border: '2px solid #7b1fa2',
              borderRadius: '8px',
              padding: '15px',
              margin: '10px 0',
              backgroundColor: '#f3e5f5'
            }}>
              <h3>Programming Practice</h3>
              <p>This hands-on content focuses on programming exercises.</p>
              <p>It's tagged with 'programming' and 'code' and is intermediate level.</p>
            </div>
          </ContentAdaptationSystem>
        </div>
      </div>
    </Layout>
  );
};

export default ContentAdaptationDemoPage;