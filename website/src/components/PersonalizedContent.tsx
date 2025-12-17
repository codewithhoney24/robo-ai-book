import React from 'react';
import { useAuth } from '../contexts/AuthContext';

interface PersonalizedContentProps {
  children: React.ReactNode;
  // Custom rendering based on user background
  forSoftwareBackground?: Array<'beginner' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert'>;
  forHardwareBackground?: Array<'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud'>;
  // Fallback content when user doesn't match criteria
  fallback?: React.ReactNode;
  // If true, content is shown to users who DON'T match the criteria
  inverse?: boolean;
}

const PersonalizedContent: React.FC<PersonalizedContentProps> = ({
  children,
  forSoftwareBackground,
  forHardwareBackground,
  fallback = null,
  inverse = false,
}) => {
  const { user, loading } = useAuth();

  if (loading || !user) {
    return <>{fallback}</>;
  }

  let shouldShow = true;

  // Check software background condition
  if (forSoftwareBackground && forSoftwareBackground.length > 0) {
    if (forSoftwareBackground.includes(user.softwareBackground)) {
      // If user's background is in the allowed list
      if (!inverse) {
        shouldShow = true;
      } else {
        shouldShow = false;
      }
    } else {
      // If user's background is NOT in the allowed list
      if (!inverse) {
        shouldShow = false;
      } else {
        shouldShow = true;
      }
    }
  }

  // Check hardware background condition (only if not already determined)
  if (shouldShow && forHardwareBackground && forHardwareBackground.length > 0) {
    if (forHardwareBackground.includes(user.hardwareBackground)) {
      // If user's hardware is in the allowed list
      if (!inverse) {
        shouldShow = true;
      } else {
        shouldShow = false;
      }
    } else {
      // If user's hardware is NOT in the allowed list
      if (!inverse) {
        shouldShow = false;
      } else {
        shouldShow = true;
      }
    }
  }

  // Apply inverse logic if needed
  if (inverse && forSoftwareBackground && forSoftwareBackground.length > 0) {
    shouldShow = !shouldShow;
  }

  return <>{shouldShow ? children : fallback}</>;
};

export default PersonalizedContent;