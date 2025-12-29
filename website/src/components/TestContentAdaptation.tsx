import React from 'react';
import ContentAdaptationSystem from './ContentAdaptationSystem';

const TestContentAdaptation: React.FC = () => {
  return (
    <div>
      <h2>Content Adaptation System Test</h2>

      <ContentAdaptationSystem
        tags={['robotics', 'beginner', 'tutorial']}
        difficulty="beginner"
        contentType="visual"
        topic="ros2-basics"
        className="test-content-block"
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
          <p><em>Click on this content to see tag information and personalization details.</em></p>
        </div>
      </ContentAdaptationSystem>

      <ContentAdaptationSystem
        tags={['advanced', 'simulation']}
        difficulty="advanced"
        contentType="handsOn"
        topic="simulation"
        className="test-content-block"
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
          <p><em>Click on this content to see tag information and personalization details.</em></p>
        </div>
      </ContentAdaptationSystem>

      <ContentAdaptationSystem
        tags={['theory', 'textual']}
        difficulty="intermediate"
        contentType="textual"
        topic="kinematics"
        className="test-content-block"
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
          <p><em>Click on this content to see tag information and personalization details.</em></p>
        </div>
      </ContentAdaptationSystem>
    </div>
  );
};

export default TestContentAdaptation;