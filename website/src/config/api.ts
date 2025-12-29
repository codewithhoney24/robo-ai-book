// API Configuration
// Function to get the API base URL from the Docusaurus plugin
export const getApiConfig = () => {
  // Check if the API config is available globally (from the Docusaurus plugin)
  if (typeof window !== 'undefined' && (window as any).API_CONFIG) {
    return (window as any).API_CONFIG;
  }
  // Fallback to environment variable if available in browser context
  if (typeof process !== 'undefined' && process.env) {
    const baseUrl = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api';
    return {
      BASE_URL: baseUrl,
      ENDPOINTS: {
        AUTH: {
          SIGNUP_WITH_BACKGROUND: `${baseUrl}/v1/auth/signup-with-background`,
          SIGNIN: `${baseUrl}/v1/auth/session`,
          SESSION: `${baseUrl}/v1/auth/session`,
          SIGNOUT: `${baseUrl}/v1/auth/session`,
          UPDATE_BACKGROUND: `${baseUrl}/v1/auth/update-background`,
        },
        PERSONALIZATION: {
          PERSONALIZE: `${baseUrl}/v1/personalization/personalize`,
        },
        // Add other endpoints as needed
      },
    };
  }
  // Final fallback
  const baseUrl = 'http://localhost:8000/api';
  return {
    BASE_URL: baseUrl,
    ENDPOINTS: {
      AUTH: {
        SIGNUP_WITH_BACKGROUND: `${baseUrl}/v1/auth/signup-with-background`,
        SIGNIN: `${baseUrl}/v1/auth/session`,
        SESSION: `${baseUrl}/v1/auth/session`,
        SIGNOUT: `${baseUrl}/v1/auth/session`,
        UPDATE_BACKGROUND: `${baseUrl}/v1/auth/update-background`,
      },
      PERSONALIZATION: {
        PERSONALIZE: `${baseUrl}/v1/personalization/personalize`,
      },
      // Add other endpoints as needed
    },
  };
};

// Export a getter function instead of a static object
export const API_CONFIG = {
  get BASE_URL() {
    return getApiConfig().BASE_URL;
  },
  get ENDPOINTS() {
    return getApiConfig().ENDPOINTS;
  }
};