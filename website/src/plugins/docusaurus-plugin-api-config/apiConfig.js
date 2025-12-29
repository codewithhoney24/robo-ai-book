// website/src/plugins/docusaurus-plugin-api-config/apiConfig.js

// Get the API base URL from Docusaurus config
const API_BASE_URL =
  typeof window !== 'undefined' && window.__APP_CUSTOM_FIELDS__
    ? window.__APP_CUSTOM_FIELDS__.REACT_APP_API_BASE_URL
    : 'http://localhost:8000/api';

// Define API endpoints
const API_CONFIG = {
  BASE_URL: API_BASE_URL,
  ENDPOINTS: {
    AUTH: {
      SIGNUP_WITH_BACKGROUND: `${API_BASE_URL}/v1/auth/signup-with-background`,
      SIGNIN: `${API_BASE_URL}/v1/auth/session`,
      SESSION: `${API_BASE_URL}/v1/auth/session`,
      SIGNOUT: `${API_BASE_URL}/v1/auth/session`,
      UPDATE_BACKGROUND: `${API_BASE_URL}/v1/auth/update-background`,
    },
    PERSONALIZATION: {
      PERSONALIZE: `${API_BASE_URL}/v1/personalization/personalize`,
    },
    // Add other endpoints as needed
  },
};

// Make the API config available globally
if (typeof window !== 'undefined') {
  window.API_CONFIG = API_CONFIG;
}

export { API_CONFIG };