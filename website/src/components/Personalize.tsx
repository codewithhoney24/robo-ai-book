import React, { ReactNode } from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';

interface PersonalizeProps {
  forHardwareBackground?: string[];
  forSoftwareBackground?: string[];
  forDifficulty?: string[];
  fallback?: ReactNode;
  children: ReactNode;
}

const Personalize: React.FC<PersonalizeProps> = ({
  forHardwareBackground = [],
  forSoftwareBackground = [],
  forDifficulty = [],
  fallback = null,
  children
}) => {
  const { settings } = usePersonalization();
  const userHardwareBackground = settings.preferences.hardwareBackground;
  const userSoftwareBackground = settings.preferences.softwareBackground;
  const userDifficulty = settings.preferences.difficulty;

  // Check if any of the personalization criteria match
  const hardwareMatch = forHardwareBackground.length === 0 || forHardwareBackground.includes(userHardwareBackground);
  const softwareMatch = forSoftwareBackground.length === 0 || forSoftwareBackground.includes(userSoftwareBackground);
  const difficultyMatch = forDifficulty.length === 0 || forDifficulty.includes(userDifficulty);

  const shouldShowContent = hardwareMatch && softwareMatch && difficultyMatch;

  return <>{shouldShowContent ? children : fallback}</>;
};

export default Personalize;