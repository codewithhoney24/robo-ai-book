import React from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';
import DynamicContentTagger from './DynamicContentTagger';

interface AdaptiveContentProps {
  children: React.ReactNode;
  contentType?: 'textual' | 'visual' | 'handsOn';
  topic: string;
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
  tags?: string[];
  fallbackContent?: React.ReactNode;
}

const AdaptiveContent: React.FC<AdaptiveContentProps> = ({
  children,
  contentType = 'visual',
  topic,
  difficulty = 'beginner',
  tags = [],
  fallbackContent
}) => {
  const { personalizationData } = usePersonalization();
  const { preferences, language } = personalizationData;

  // Check if this content should be shown based on user preferences
  const shouldShowBasedOnDifficulty = () => {
    if (difficulty === 'beginner') return true;
    if (difficulty === 'intermediate' &&
        (preferences.difficulty === 'intermediate' || preferences.difficulty === 'advanced')) {
      return true;
    }
    if (difficulty === 'advanced' && preferences.difficulty === 'advanced') {
      return true;
    }
    return false;
  };

  // Check if user prefers this type of content
  const userPrefersThisType = () => {
    if (contentType === 'visual' && preferences.learningStyle === 'visual') return true;
    if (contentType === 'textual' && preferences.learningStyle === 'textual') return true;
    if (contentType === 'handsOn' && preferences.learningStyle === 'handsOn') return true;
    return false;
  };

  // Render different content based on preferences
  const renderAdaptedContent = () => {
    // If user prefers this content type, show it prominently
    if (userPrefersThisType()) {
      return (
        <DynamicContentTagger
          tags={tags}
          contentType={contentType}
          difficulty={difficulty}
          topic={topic}
          fallback={fallbackContent}
        >
          <div className={`adaptive-content adaptive-content-${contentType} preferred`}>
            {children}
          </div>
        </DynamicContentTagger>
      );
    }

    // If user doesn't prefer this type but it's important content, show with less prominence
    if (shouldShowBasedOnDifficulty()) {
      return (
        <DynamicContentTagger
          tags={tags}
          contentType={contentType}
          difficulty={difficulty}
          topic={topic}
          fallback={fallbackContent}
        >
          <div className={`adaptive-content adaptive-content-${contentType} secondary`}>
            {children}
          </div>
        </DynamicContentTagger>
      );
    }

    // If user doesn't prefer this type and it's not important for their level, show fallback or nothing
    return fallbackContent || null;
  };

  return renderAdaptedContent();
};

export default AdaptiveContent;