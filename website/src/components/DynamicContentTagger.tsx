import React, { ReactNode } from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';

interface DynamicContentTaggerProps {
  children: ReactNode;
  tags: string[];
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
  contentType?: 'textual' | 'visual' | 'handsOn';
  topic?: string;
  fallback?: ReactNode;
  className?: string;
}

const DynamicContentTagger: React.FC<DynamicContentTaggerProps> = ({
  children,
  tags,
  difficulty = 'beginner',
  contentType = 'visual',
  topic = '',
  fallback = null,
  className = ''
}) => {
  const { personalizationData } = usePersonalization();
  const { preferences } = personalizationData;

  // Check if content should be shown based on tags and user preferences
  const shouldShowContent = (): boolean => {
    // If user has specific preferred topics and this topic is not in the list
    if (preferences.preferredTopics.length > 0 && topic && !preferences.preferredTopics.includes(topic)) {
      return false;
    }

    // If content difficulty is higher than user's level and user doesn't want advanced content
    if (difficulty === 'advanced' && !preferences.advancedContent) {
      return false;
    }

    if (difficulty === 'intermediate' && preferences.difficulty === 'beginner' && !preferences.advancedContent) {
      return false;
    }

    // If user prefers a specific learning style and this content doesn't match
    if (preferences.learningStyle !== 'visual' && contentType === 'visual' && preferences.adaptiveDifficulty) {
      return false;
    }

    if (preferences.learningStyle !== 'textual' && contentType === 'textual' && preferences.adaptiveDifficulty) {
      return false;
    }

    if (preferences.learningStyle !== 'handsOn' && contentType === 'handsOn' && preferences.adaptiveDifficulty) {
      return false;
    }

    // Check if any of the content tags match the user's preferred topics
    if (preferences.preferredTopics.length > 0) {
      return tags.some(tag => preferences.preferredTopics.includes(tag));
    }

    return true;
  };

  // Determine if this content matches user's learning preferences
  const matchesPreferences = (): boolean => {
    // Check if content type matches user preference
    const typeMatches: boolean = preferences.learningStyle === contentType;

    // Check if difficulty matches user preference
    const difficultyMatches: boolean = preferences.difficulty === difficulty;

    // Check if topic matches user preference
    const topicMatches: boolean = !!(topic && preferences.preferredTopics.includes(topic));

    return typeMatches || difficultyMatches || topicMatches;
  };

  if (!shouldShowContent()) {
    return <>{fallback}</>;
  }

  const contentClass = `${className} dynamic-content ${matchesPreferences() ? 'preferred' : 'standard'} ${contentType}-${difficulty}`;
  
  return (
    <div className={contentClass}>
      {children}
    </div>
  );
};

export default DynamicContentTagger;