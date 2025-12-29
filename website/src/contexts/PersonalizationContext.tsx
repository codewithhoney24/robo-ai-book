import React, { createContext, useContext, useState, ReactNode } from 'react';

interface PersonalizationPreferences {
  hardwareBackground: 'none' | 'beginner' | 'intermediate' | 'advanced' | 'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud';
  softwareBackground: 'beginner' | 'intermediate' | 'advanced' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert';
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  learningStyle: 'visual' | 'textual' | 'handsOn';
  advancedContent: boolean;
  foundationalKnowledge: boolean;
  complexConcepts: boolean;
  progressTracking: boolean;
  moduleSuggestions: boolean;
  contentRecommendation: boolean;
  learningSpeed: 'slow' | 'normal' | 'fast';
  progressiveDisclosure: boolean;
  adaptiveDifficulty: boolean;
  smartAdaptation: boolean;
  quizResults: boolean;
  contentAdjustment: boolean;
  targetedHelp: boolean;
  interactionPatterns: boolean;
  contentAccess: 'simulation' | 'realHardware';
  learningGoals: 'research' | 'hobby' | 'professional';
  contentContext: string;
  complexityLevel: 'basic' | 'standard' | 'complex';
  preferredTopics: string[];
  codeExamples: boolean;
  quizzesAssessments: boolean;
  practiceSessions: boolean;
  customExercises: boolean;
  learningGoal?: string;
}

interface PersonalizationData {
  theme: string;
  language: 'en' | 'es' | 'ur' | string;
  preferences: PersonalizationPreferences;
}

interface PersonalizationContextType {
  personalizationData: PersonalizationData;
  settings: PersonalizationData;
  updatePersonalization: (data: Partial<PersonalizationData>) => void;
  updateSettings: (data: Partial<PersonalizationData>) => void;
  refreshContent: () => void;
}

const PersonalizationContext = createContext<PersonalizationContextType | undefined>(undefined);

export const PersonalizationProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [personalizationData, setPersonalizationData] = useState<PersonalizationData>({
    theme: 'light',
    language: 'en',
    preferences: {
      hardwareBackground: 'no_gpu',
      softwareBackground: 'beginner',
      difficulty: 'beginner',
      learningStyle: 'visual',
      advancedContent: false,
      foundationalKnowledge: true,
      complexConcepts: false,
      progressTracking: true,
      moduleSuggestions: true,
      contentRecommendation: true,
      learningSpeed: 'normal',
      progressiveDisclosure: true,
      adaptiveDifficulty: true,
      smartAdaptation: true,
      quizResults: true,
      contentAdjustment: true,
      targetedHelp: true,
      interactionPatterns: true,
      contentAccess: 'simulation',
      learningGoals: 'research',
      contentContext: 'robotics',
      complexityLevel: 'standard',
      preferredTopics: [],
      codeExamples: true,
      quizzesAssessments: true,
      practiceSessions: true,
      customExercises: false,
    },
  });

  const [, setRefreshTrigger] = useState(0);

  const updatePersonalization = (data: Partial<PersonalizationData>) => {
    setPersonalizationData(prev => ({
      ...prev,
      ...data,
      preferences: {
        ...prev.preferences,
        ...(data.preferences || {})
      }
    }));
    setRefreshTrigger(prev => prev + 1);
  };

  const updateSettings = (data: Partial<PersonalizationData>) => {
    setPersonalizationData(prev => ({
      ...prev,
      ...data,
      preferences: {
        ...prev.preferences,
        ...(data.preferences || {})
      }
    }));
    setRefreshTrigger(prev => prev + 1);
  };

  const refreshContent = () => {
    console.log('Content refreshed based on personalization');
    setRefreshTrigger(prev => prev + 1);
  };

  return (
    <PersonalizationContext.Provider value={{
      personalizationData,
      settings: personalizationData,
      updatePersonalization,
      updateSettings,
      refreshContent
    }}>
      {children}
    </PersonalizationContext.Provider>
  );
};

export const usePersonalization = (): PersonalizationContextType => {
  const context = useContext(PersonalizationContext);
  if (!context) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
};

export default PersonalizationContext;