// Example unit test for the personalization service
// This would be run with a testing framework like Jest

import { 
  getPersonalizationRules, 
  isContentVisible, 
  getPersonalizedContent,
  getRecommendations
} from '../frontend/src/services/PersonalizationService';

// Mock user data for testing
const mockBeginnerUser = {
  id: '1',
  email: 'beginner@example.com',
  name: 'Beginner User',
  softwareBackground: 'beginner' as const,
  hardwareBackground: 'no_gpu' as const,
};

const mockExpertUser = {
  id: '2',
  email: 'expert@example.com',
  name: 'Expert User',
  softwareBackground: 'ai_robotics_expert' as const,
  hardwareBackground: 'rtx_workstation' as const,
};

describe('PersonalizationService', () => {
  describe('getPersonalizationRules', () => {
    it('should return correct rules for beginner user with no_gpu', () => {
      const rules = getPersonalizationRules(mockBeginnerUser);
      
      expect(rules.showDetailedExplanations).toBe(true);
      expect(rules.showAdvancedContent).toBe(false);
      expect(rules.showGpuIntensiveContent).toBe(false);
      expect(rules.showCloudAlternatives).toBe(true);
      expect(rules.showJetsonSpecificContent).toBe(false);
      expect(rules.showCostOptimizationTips).toBe(false);
      expect(rules.showVRAMWarnings).toBe(false);
    });

    it('should return correct rules for expert user with rtx_workstation', () => {
      const rules = getPersonalizationRules(mockExpertUser);
      
      expect(rules.showDetailedExplanations).toBe(false);
      expect(rules.showAdvancedContent).toBe(true);
      expect(rules.showGpuIntensiveContent).toBe(true);
      expect(rules.showCloudAlternatives).toBe(false);
      expect(rules.showJetsonSpecificContent).toBe(false);
      expect(rules.showCostOptimizationTips).toBe(false);
      expect(rules.showVRAMWarnings).toBe(false);
    });
  });

  describe('isContentVisible', () => {
    it('should return true when no requirements are specified', () => {
      const isVisible = isContentVisible(mockBeginnerUser, undefined, undefined);
      expect(isVisible).toBe(true);
    });

    it('should respect software background requirements', () => {
      // Content for experts should not be visible to beginners
      const isVisible = isContentVisible(
        mockBeginnerUser,
        ['ai_robotics_expert'],
        undefined
      );
      expect(isVisible).toBe(false);
      
      // Content for beginners should be visible to beginners
      const isVisible2 = isContentVisible(
        mockBeginnerUser,
        ['beginner'],
        undefined
      );
      expect(isVisible2).toBe(true);
    });

    it('should respect hardware background requirements', () => {
      // GPU-intensive content should not be visible to no_gpu users
      const isVisible = isContentVisible(
        mockBeginnerUser,
        undefined,
        ['rtx_workstation']
      );
      expect(isVisible).toBe(false);
      
      // Content for no_gpu should be visible to no_gpu users
      const isVisible2 = isContentVisible(
        mockBeginnerUser,
        undefined,
        ['no_gpu']
      );
      expect(isVisible2).toBe(true);
    });
  });

  describe('getPersonalizedContent', () => {
    it('should add appropriate notes for beginners', () => {
      const baseContent = 'This is the basic content.';
      const personalized = getPersonalizedContent(mockBeginnerUser, baseContent);
      
      expect(personalized).toContain('Note: As a beginner');
    });

    it('should add advanced tips for experts', () => {
      const baseContent = 'This is the basic content.';
      const personalized = getPersonalizedContent(mockExpertUser, baseContent);
      
      expect(personalized).toContain('Advanced tip:');
    });
  });

  describe('getRecommendations', () => {
    it('should return appropriate recommendations for beginners', () => {
      const recommendations = getRecommendations(mockBeginnerUser);
      
      expect(recommendations).toContain('Start with the basics: Python and fundamental programming concepts');
      expect(recommendations).toContain('Focus on CPU-based implementations and cloud computing options');
    });

    it('should return appropriate recommendations for experts with high-end hardware', () => {
      const recommendations = getRecommendations(mockExpertUser);
      
      expect(recommendations).toContain('Check out the latest research papers in the field');
      expect(recommendations).toContain('You can run the most demanding simulations and training');
    });
  });
});

// Validation tests
import { 
  validateSignupData, 
  validateProfileUpdateData,
  isValidSoftwareBackground,
  isValidHardwareBackground
} from '../backend/src/validation/user';

describe('Validation Service', () => {
  describe('isValidSoftwareBackground', () => {
    it('should return true for valid software backgrounds', () => {
      expect(isValidSoftwareBackground('beginner')).toBe(true);
      expect(isValidSoftwareBackground('python_intermediate')).toBe(true);
      expect(isValidSoftwareBackground('ros2_developer')).toBe(true);
      expect(isValidSoftwareBackground('ai_robotics_expert')).toBe(true);
    });

    it('should return false for invalid software backgrounds', () => {
      expect(isValidSoftwareBackground('invalid')).toBe(false);
      expect(isValidSoftwareBackground('')).toBe(false);
    });
  });

  describe('isValidHardwareBackground', () => {
    it('should return true for valid hardware backgrounds', () => {
      expect(isValidHardwareBackground('no_gpu')).toBe(true);
      expect(isValidHardwareBackground('rtx_laptop')).toBe(true);
      expect(isValidHardwareBackground('rtx_workstation')).toBe(true);
      expect(isValidHardwareBackground('jetson_kit')).toBe(true);
      expect(isValidHardwareBackground('cloud')).toBe(true);
    });

    it('should return false for invalid hardware backgrounds', () => {
      expect(isValidHardwareBackground('invalid')).toBe(false);
      expect(isValidHardwareBackground('')).toBe(false);
    });
  });

  describe('validateSignupData', () => {
    it('should validate correct signup data', () => {
      const validData = {
        name: 'Test User',
        email: 'test@example.com',
        password: 'SecurePassword123',
        softwareBackground: 'beginner',
        hardwareBackground: 'no_gpu'
      };

      const result = validateSignupData(validData);
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('should detect validation errors', () => {
      const invalidData = {
        name: '', // Required
        email: 'invalid-email', // Invalid email
        password: '123', // Too short
        softwareBackground: 'invalid', // Not in enum
        hardwareBackground: 'no_gpu'
      };

      const result = validateSignupData(invalidData);
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Name is required');
      expect(result.errors).toContain('Valid email is required');
      expect(result.errors).toContain('Password must be at least 8 characters');
    });
  });

  describe('validateProfileUpdateData', () => {
    it('should validate correct profile update data', () => {
      const validData = {
        name: 'Updated Name',
        softwareBackground: 'python_intermediate'
      };

      const result = validateProfileUpdateData(validData);
      expect(result.isValid).toBe(true);
    });

    it('should reject invalid profile update data', () => {
      const invalidData = {
        name: 'Updated Name',
        softwareBackground: 'invalid_value'
      };

      const result = validateProfileUpdateData(invalidData);
      expect(result.isValid).toBe(false);
    });
  });
});