// Personalization logic utilities

export interface PersonalizationSettings {
  theme?: string;
  language?: string;
  preferences?: Record<string, any>;
}

// Function to save personalization settings to localStorage
export const savePersonalizationSettings = (settings: PersonalizationSettings): void => {
  try {
    const existingSettings = getPersonalizationSettings();
    const updatedSettings = { ...existingSettings, ...settings };
    localStorage.setItem('personalizationSettings', JSON.stringify(updatedSettings));
  } catch (error) {
    console.error('Error saving personalization settings:', error);
  }
};

// Function to retrieve personalization settings from localStorage
export const getPersonalizationSettings = (): PersonalizationSettings => {
  try {
    const settingsString = localStorage.getItem('personalizationSettings');
    return settingsString ? JSON.parse(settingsString) : {};
  } catch (error) {
    console.error('Error retrieving personalization settings:', error);
    return {};
  }
};

// Function to apply personalization settings
export const applyPersonalizationSettings = (settings: PersonalizationSettings): void => {
  // Only run on the client side
  if (typeof window !== 'undefined') {
    // Apply theme
    if (settings.theme) {
      document.documentElement.setAttribute('data-theme', settings.theme);
    }

    // Apply language
    if (settings.language) {
      document.documentElement.lang = settings.language;
    }

    // Apply other preferences
    if (settings.preferences) {
      Object.entries(settings.preferences).forEach(([key, value]) => {
        // Apply preferences as needed
        localStorage.setItem(`preference_${key}`, JSON.stringify(value));
      });
    }
  }
};

// Function to reset personalization settings
export const resetPersonalizationSettings = (): void => {
  try {
    if (typeof window !== 'undefined') {
      localStorage.removeItem('personalizationSettings');
      // Reset theme and language to defaults
      document.documentElement.removeAttribute('data-theme');
      document.documentElement.lang = 'en';
    }
  } catch (error) {
    console.error('Error resetting personalization settings:', error);
  }
};

// Function to get starting module path based on user's background
export const getStartingModulePath = (softwareBackground: string, hardwareBackground: string): string => {
  // This would typically get the starting module path based on user's background
  // For now, we'll return a default path, but in the future we could customize
  // based on the user's background
  if (softwareBackground === 'beginner') {
    return '/module/m0-w1-2-introduction-to-physical-ai';
  } else if (softwareBackground === 'intermediate') {
    return '/module/m1-w3-ros2-architecture'; // Using a more appropriate intermediate module
  } else {
    return '/module/m2-w6-gazebo-physics-simulation'; // Using a more appropriate advanced module
  }
};

// Function to get starting module title based on user's background
export const getStartingModuleTitle = (softwareBackground: string, hardwareBackground: string): string => {
  // This would typically get the starting module title based on user's background
  if (softwareBackground === 'beginner') {
    return 'Introduction to Physical AI';
  } else if (softwareBackground === 'intermediate') {
    return 'ROS 2 Architecture and Components';
  } else {
    return 'Gazebo Physics Simulation';
  }
};