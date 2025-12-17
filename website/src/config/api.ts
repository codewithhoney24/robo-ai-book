// API Configuration for Better-Auth
const AUTH_SERVER_URL = typeof window !== 'undefined'
  ? 'http://localhost:8000' // Browser
  : process.env.AUTH_SERVER_URL || 'http://localhost:8000'; // SSR

export const API_ENDPOINTS = {
  // Better-Auth endpoints (correct paths)
  AUTH_SIGNUP: `${AUTH_SERVER_URL}/api/auth/sign-up/email`,
  AUTH_SIGNIN: `${AUTH_SERVER_URL}/api/auth/sign-in/email`,
  AUTH_SIGNOUT: `${AUTH_SERVER_URL}/api/auth/sign-out`,
  AUTH_SESSION: `${AUTH_SERVER_URL}/api/auth/get-session`,
  AUTH_GITHUB: `${AUTH_SERVER_URL}/api/auth/sign-in/social/github`,
  
  // Health check
  HEALTH: `${AUTH_SERVER_URL}/health`,
};

export const AUTH_CONFIG = {
  baseUrl: AUTH_SERVER_URL,
  credentials: 'include', // Important: Always send cookies
  headers: {
    'Content-Type': 'application/json',
  },
};

export default API_ENDPOINTS;