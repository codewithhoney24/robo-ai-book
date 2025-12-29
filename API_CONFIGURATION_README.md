# API Configuration Guide

## Problem Solved

The application was experiencing an "Unexpected token '<', '<!DOCTYPE '... is not valid JSON" error when making API calls from the frontend. This occurred because the frontend was making requests to relative URLs (e.g., `/api/v1/auth/signup`) which were resolving to the frontend server (localhost:3000) instead of the backend API server (localhost:8000).

## Solution Implemented

1. **Created a centralized API configuration** in `website/src/config/api.ts` that defines all API endpoints with the correct base URL.

2. **Created a Docusaurus plugin** in `website/src/plugins/docusaurus-plugin-api-config` to properly expose the API configuration to the client-side code.

3. **Updated all frontend components** to use the new API configuration instead of hardcoded URLs:
   - AuthContext.tsx (signup, signin, session, etc.)
   - personalizationService.ts
   - TextSelectionHandler.tsx
   - ChatbotWidget.tsx
   - theme/DocItem/index.js

4. **Added environment variable support** with a fallback to the default backend URL (http://localhost:8000/api).

## How to Configure

### For Development

1. The application uses the default backend URL `http://localhost:8000/api` by default
2. You can override this by setting the `REACT_APP_API_BASE_URL` environment variable
3. A `.env` file is included in the website directory with the default configuration

### For Production

1. Set the `REACT_APP_API_BASE_URL` environment variable to point to your production backend API
2. Example: `REACT_APP_API_BASE_URL=https://api.yourdomain.com`

## Running the Application

1. Start the backend server:
   ```bash
   cd backend
   python -m uvicorn src.api.main:app --reload --port 8000
   ```
   Or using the provided script:
   ```bash
   python server.py
   ```

2. Start the frontend:
   ```bash
   cd website
   npm start
   ```

## API Endpoints Configuration

The API configuration is handled through a getter function to ensure it's available when the Docusaurus plugin loads:

```typescript
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
          SIGNUP: `${baseUrl}/v1/auth/signup`,
          SIGNIN: `${baseUrl}/v1/auth/signin`,
          SESSION: `${baseUrl}/v1/auth/session`,
          SIGNOUT: `${baseUrl}/v1/auth/signout`,
          UPDATE_USER: `${baseUrl}/v1/auth/user`,
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
        SIGNUP: `${baseUrl}/v1/auth/signup`,
        SIGNIN: `${baseUrl}/v1/auth/signin`,
        SESSION: `${baseUrl}/v1/auth/session`,
        SIGNOUT: `${baseUrl}/v1/auth/signout`,
        UPDATE_USER: `${baseUrl}/v1/auth/user`,
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
```

## Components Updated

The following components were updated to use the new API configuration:

1. `AuthContext.tsx` - Authentication API calls
2. `personalizationService.ts` - Personalization API calls
3. `TextSelectionHandler.tsx` - Text translation/explanation API calls
4. `ChatbotWidget.tsx` - Chat and translation API calls
5. `theme/DocItem/index.js` - Document translation API calls

## Testing

To verify the fix works:

1. Start both the backend and frontend servers
2. Navigate to the signup page (`/signup`)
3. Create a new account
4. Verify that the signup request is successful and no "Unexpected token '<'" error occurs
5. Test other API-dependent functionality like sign-in, chat, and translation features