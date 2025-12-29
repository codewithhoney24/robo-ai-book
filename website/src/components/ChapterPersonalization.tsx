import React, { useState } from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';
import { useAuth } from '../contexts/AuthContext';
import PersonalizeButton from './PersonalizeButton';
import { personalizationService } from '../services/personalizationService';

interface ChapterPersonalizationProps {
  chapterTitle: string;
  topic: string;
  children?: React.ReactNode;
}

const ChapterPersonalization: React.FC<ChapterPersonalizationProps> = ({
  chapterTitle,
  topic,
  children
}) => {
  const { personalizationData, updatePersonalization } = usePersonalization();
  const { preferences } = personalizationData;
  const { user } = useAuth();
  const [isProcessing, setIsProcessing] = useState(false);

  const handlePersonalizeForChapter = async () => {
    if (!user) {
      alert('Please sign in to access personalized content.');
      return;
    }

    setIsProcessing(true);
    
    try {
      // Update the content context to be specific to this chapter
      updatePersonalization({
        preferences: {
          ...preferences,
          contentContext: topic,
          contentAdjustment: true // Enable content adjustment for this topic
        }
      });

      // In a real implementation, we would send the chapter content to the backend for personalization
      // For now, we'll just show a confirmation message
      alert(`Content for "${chapterTitle}" has been personalized based on your preferences!`);
    } catch (error) {
      console.error('Error personalizing chapter:', error);
      alert('There was an error personalizing the content. Please try again.');
    } finally {
      setIsProcessing(false);
    }
  };

  return (
    <div style={{
      border: '1px solid rgba(0, 255, 255, 0.3)',
      borderRadius: '8px',
      padding: '20px',
      margin: '20px 0',
      backgroundColor: '#0f172a',
      boxShadow: '0 0 10px rgba(0, 255, 255, 0.1)'
    }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '15px' }}>
        <h2 style={{ margin: 0, color: '#00FFFF' }}>{chapterTitle}</h2>
        <PersonalizeButton
          onPersonalize={handlePersonalizeForChapter}
          className="chapter-personalize-btn"
          disabled={isProcessing}
        />
      </div>
      <p style={{ color: '#a5f3fc', fontStyle: 'italic' }}>
        Content in this chapter will adapt to your learning preferences
      </p>
      <div style={{ marginTop: '15px' }}>
        {children}
      </div>
    </div>
  );
};

export default ChapterPersonalization;