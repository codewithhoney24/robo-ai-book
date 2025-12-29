// services/personalizationService.ts

import { API_CONFIG } from '../config/api';

interface PersonalizationRequest {
  userId: string;
  content: string;
  userBackground: {
    softwareBackground: 'beginner' | 'intermediate' | 'advanced' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert';
    hardwareBackground: 'none' | 'beginner' | 'intermediate' | 'advanced' | 'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud';
  };
  chapterTitle?: string;
}

interface PersonalizationResponse {
  personalizedContent: string;
}

export const personalizationService = {
  async personalizeContent(request: PersonalizationRequest): Promise<PersonalizationResponse> {
    try {
      const response = await fetch(API_CONFIG.ENDPOINTS.PERSONALIZATION.PERSONALIZE, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('better-auth-token') || ''}` // Using the correct token key
        },
        body: JSON.stringify(request)
      });

      if (!response.ok) {
        throw new Error(`Personalization API error: ${response.status} ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error in personalization service:', error);
      throw error;
    }
  }
};