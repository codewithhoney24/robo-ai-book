import React, { useState } from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';

interface ContentAdaptationSystemProps {
  children: React.ReactNode;
  tags: string[];
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
  contentType?: 'textual' | 'visual' | 'handsOn';
  topic?: string;
  className?: string;
}

const ContentAdaptationSystem: React.FC<ContentAdaptationSystemProps> = ({
  children,
  tags,
  difficulty = 'beginner',
  contentType = 'visual',
  topic = '',
  className = ''
  }) => {
  const { personalizationData } = usePersonalization();
  const { preferences } = personalizationData;
  const [showChatKit, setShowChatKit] = useState(false);

  // Determine content visibility based on personalization
  const isVisible = (): boolean => {
    // If user has specific learning goals and this content doesn't align
    if (preferences.learningGoals !== 'research' && topic.includes('research')) {
      return false;
    }

    // If content difficulty is beyond user's level and they don't want advanced content
    if (difficulty === 'advanced' && !preferences.advancedContent) {
      return false;
    }

    if (difficulty === 'intermediate' && preferences.difficulty === 'beginner' && !preferences.advancedContent) {
      return false;
    }

    // If user prefers a different learning style
    if (preferences.adaptiveDifficulty && preferences.learningStyle !== contentType) {
      return false;
    }

    // If user has preferred topics and this content doesn't match
    if (preferences.preferredTopics.length > 0 && topic && !preferences.preferredTopics.includes(topic)) {
      return false;
    }

    return true;
  };

  // Calculate how well this content matches user preferences
  const calculateMatchScore = (): number => {
    let score = 0;

    // Add points for matching difficulty
    if (preferences.difficulty === difficulty) {
      score += 3;
    } else if (
      (preferences.difficulty === 'intermediate' && difficulty === 'beginner') ||
      (preferences.difficulty === 'advanced' && difficulty === 'intermediate')
    ) {
      score += 1;
    }

    // Add points for matching content type
    if (preferences.learningStyle === contentType) {
      score += 2;
    }

    // Add points for matching topic
    if (topic && preferences.preferredTopics.includes(topic)) {
      score += 2;
    }

    // Add points for matching tags
    const matchingTags = tags.filter(tag => preferences.preferredTopics.includes(tag));
    score += matchingTags.length;

    return score;
  };

  if (!isVisible()) {
    return null;
  }

  const matchScore = calculateMatchScore();
  const contentClass = `${className} adapted-content match-score-${matchScore} ${contentType}-${difficulty}`;

  // Function to show the ChatKit with tag information
  const showTagInfo = () => {
    setShowChatKit(true);
  };

  // Function to handle navigation to different difficulty levels
  const navigateToDifficulty = (level: 'beginner' | 'intermediate' | 'advanced') => {
    // In a real implementation, this would navigate to content with the specified difficulty
    // For now, we'll just close the chatkit and potentially update preferences
    setShowChatKit(false);

    // Optionally update the difficulty preference
    if (preferences.difficulty !== level) {
      // This would require access to updatePersonalization function
      // For now, we'll just show an alert
      alert(`Navigating to ${level} level content. In a real implementation, this would take you to content tagged with #${level}`);
    }
  };

  return (
    <div className={contentClass} data-match-score={matchScore}>
      <div
        onClick={showTagInfo}
        style={{
          cursor: 'pointer',
          position: 'relative',
          display: 'inline-block'
        }}
        title="Click to see content tags and personalization info"
      >
        {children}
      </div>

      {showChatKit && (
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
            zIndex: 1000,
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
            <h3>Content Tags & Personalization Info</h3>
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

          <div style={{
            flex: 1,
            overflowY: 'auto',
            padding: '20px'
          }}>
            <div style={{ marginBottom: '20px' }}>
              <h4>Content Tags</h4>
              <div style={{
                display: 'flex',
                flexWrap: 'wrap',
                gap: '8px'
              }}>
                {tags.map((tag, index) => (
                  <span
                    key={index}
                    style={{
                      padding: '5px 10px',
                      backgroundColor: '#e3f2fd',
                      borderRadius: '16px',
                      fontSize: '0.9rem'
                    }}
                  >
                    #{tag}
                  </span>
                ))}
              </div>
            </div>

            <div style={{ marginBottom: '20px' }}>
              <h4>Content Details</h4>
              <p><strong>Difficulty:</strong> {difficulty}</p>
              <p><strong>Content Type:</strong> {contentType}</p>
              <p><strong>Topic:</strong> {topic || 'Not specified'}</p>
              <p><strong>Match Score:</strong> {matchScore}</p>
            </div>

            <div style={{ marginBottom: '20px' }}>
              <h4>Your Preferences</h4>
              <p><strong>Learning Style:</strong> {preferences.learningStyle}</p>
              <p><strong>Preferred Difficulty:</strong> {preferences.difficulty}</p>
              <p><strong>Preferred Topics:</strong> {preferences.preferredTopics.join(', ') || 'None'}</p>
            </div>

            <div style={{ marginBottom: '20px' }}>
              <h4>Navigate by Difficulty Level</h4>
              <div style={{ display: 'flex', gap: '10px' }}>
                <button
                  onClick={() => navigateToDifficulty('beginner')}
                  style={{
                    flex: 1,
                    padding: '10px',
                    backgroundColor: preferences.difficulty === 'beginner' ? '#007bff' : '#e0e0e0',
                    color: preferences.difficulty === 'beginner' ? 'white' : 'black',
                    border: '1px solid #ccc',
                    borderRadius: '4px',
                    cursor: 'pointer'
                  }}
                >
                  Beginner
                </button>
                <button
                  onClick={() => navigateToDifficulty('intermediate')}
                  style={{
                    flex: 1,
                    padding: '10px',
                    backgroundColor: preferences.difficulty === 'intermediate' ? '#007bff' : '#e0e0e0',
                    color: preferences.difficulty === 'intermediate' ? 'white' : 'black',
                    border: '1px solid #ccc',
                    borderRadius: '4px',
                    cursor: 'pointer'
                  }}
                >
                  Intermediate
                </button>
                <button
                  onClick={() => navigateToDifficulty('advanced')}
                  style={{
                    flex: 1,
                    padding: '10px',
                    backgroundColor: preferences.difficulty === 'advanced' ? '#007bff' : '#e0e0e0',
                    color: preferences.difficulty === 'advanced' ? 'white' : 'black',
                    border: '1px solid #ccc',
                    borderRadius: '4px',
                    cursor: 'pointer'
                  }}
                >
                  Advanced
                </button>
              </div>
            </div>

            <div>
              <h4>Adaptation Logic</h4>
              <ul>
                <li>Content difficulty matches your preference: {preferences.difficulty === difficulty ? '✓' : '✗'}</li>
                <li>Content type matches your preference: {preferences.learningStyle === contentType ? '✓' : '✗'}</li>
                <li>Content topic matches your preference: {topic && preferences.preferredTopics.includes(topic) ? '✓' : '✗'}</li>
                <li>Matching tags: {tags.filter(tag => preferences.preferredTopics.includes(tag)).length} out of {tags.length}</li>
              </ul>
            </div>
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

      {showChatKit && (
        <div
          style={{
            position: 'fixed',
            top: 0,
            left: 0,
            width: '100%',
            height: '100%',
            backgroundColor: 'rgba(0,0,0,0.5)',
            zIndex: 999
          }}
          onClick={() => setShowChatKit(false)}
        />
      )}
    </div>
  );
};

export default ContentAdaptationSystem;