import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { API_CONFIG } from '../config/api';

interface AuthUser {
  id: string;
  email: string;
  name: string;
  emailVerified: boolean;
  softwareBackground?: 'beginner' | 'intermediate' | 'advanced' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert';
  hardwareBackground?: 'none' | 'beginner' | 'intermediate' | 'advanced' | 'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud';
  created_at?: string;
}

interface AuthContextType {
  user: AuthUser | null;
  loading: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (
    email: string,
    password: string,
    name: string,
    softwareBackground: 'beginner' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert',
    hardwareBackground: 'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud'
  ) => Promise<void>;
  signOut: () => Promise<void>;
  updateUser: (userData: Partial<AuthUser>) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

// API service for authentication
const authService = {
  // Sign up with background information
  signUpWithBackground: async (
    email: string,
    password: string,
    name: string,
    softwareBackground: 'beginner' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert',
    hardwareBackground: 'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud'
  ): Promise<AuthUser> => {
    const response = await fetch(API_CONFIG.ENDPOINTS.AUTH.SIGNUP_WITH_BACKGROUND, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email,
        password,
        name,
        softwareBackground,
        hardwareBackground
      }),
    });

    if (!response.ok) {
      // Check if the response is HTML (which would cause JSON parsing to fail)
      const contentType = response.headers.get('content-type');
      if (contentType && contentType.includes('text/html')) {
        // If it's HTML, we likely have a 404 or server error page
        const htmlText = await response.text();
        console.error('Received HTML instead of JSON:', htmlText);
        throw new Error(`Server error: ${response.status} - ${response.statusText}`);
      } else {
        // If it's JSON, parse it normally
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Signup failed');
      }
    }

    // Check if the response is HTML before parsing JSON
    const contentType = response.headers.get('content-type');
    if (contentType && contentType.includes('text/html')) {
      const htmlText = await response.text();
      console.error('Received HTML instead of JSON:', htmlText);
      throw new Error('Server returned HTML instead of JSON');
    }

    const data = await response.json();
    // Store the token in localStorage
    if (data.access_token) {
      localStorage.setItem('better-auth-token', data.access_token);
    }
    return data.user;
  },

  signIn: async (email: string, password: string): Promise<AuthUser> => {
    const response = await fetch(API_CONFIG.ENDPOINTS.AUTH.SIGNIN, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email,
        password,
      }),
    });

    if (!response.ok) {
      // Check if the response is HTML (which would cause JSON parsing to fail)
      const contentType = response.headers.get('content-type');
      if (contentType && contentType.includes('text/html')) {
        // If it's HTML, we likely have a 404 or server error page
        const htmlText = await response.text();
        console.error('Received HTML instead of JSON:', htmlText);
        throw new Error(`Server error: ${response.status} - ${response.statusText}`);
      } else {
        // If it's JSON, parse it normally
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Signin failed');
      }
    }

    // Check if the response is HTML before parsing JSON
    const contentType = response.headers.get('content-type');
    if (contentType && contentType.includes('text/html')) {
      const htmlText = await response.text();
      console.error('Received HTML instead of JSON:', htmlText);
      throw new Error('Server returned HTML instead of JSON');
    }

    const data = await response.json();
    // Store the token in localStorage
    if (data.access_token) {
      localStorage.setItem('better-auth-token', data.access_token);
    }
    return data.user;
  },

  getCurrentUser: async (): Promise<AuthUser | null> => {
    const token = localStorage.getItem('better-auth-token');
    if (!token) {
      return null;
    }

    try {
      const response = await fetch(API_CONFIG.ENDPOINTS.AUTH.SESSION, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      if (!response.ok) {
        // If the token is invalid, remove it
        localStorage.removeItem('better-auth-token');
        return null;
      }

      const data = await response.json();
      return data.user;
    } catch (error) {
      console.error('Error fetching user:', error);
      localStorage.removeItem('better-auth-token');
      return null;
    }
  },

  signOut: async (): Promise<void> => {
    const token = localStorage.getItem('better-auth-token');
    if (token) {
      // Call the signout endpoint
      await fetch(API_CONFIG.ENDPOINTS.AUTH.SIGNOUT, {
        method: 'DELETE',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });
    }
    // Clear the local storage
    localStorage.removeItem('better-auth-token');
  },

  updateCurrentUser: async (userData: Partial<AuthUser>): Promise<AuthUser> => {
    const token = localStorage.getItem('better-auth-token');
    if (!token) {
      throw new Error('Not authenticated');
    }

    const response = await fetch(API_CONFIG.ENDPOINTS.AUTH.UPDATE_BACKGROUND, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${token}`,
      },
      body: JSON.stringify({
        softwareBackground: userData.softwareBackground,
        hardwareBackground: userData.hardwareBackground
      }),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.detail || 'Update failed');
    }

    const data = await response.json();
    return data;
  }
};

export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<AuthUser | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Initialize user from storage or API
    const initializeUser = async () => {
      const storedUser = await authService.getCurrentUser();
      if (storedUser) {
        setUser(storedUser);
      }
      setLoading(false);
    };

    initializeUser();
  }, []);

  const signIn = async (email: string, password: string) => {
    const userData = await authService.signIn(email, password);
    setUser(userData);
  };

  const signUp = async (
    email: string,
    password: string,
    name: string,
    softwareBackground: 'beginner' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert',
    hardwareBackground: 'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud'
  ) => {
    const userData = await authService.signUpWithBackground(email, password, name, softwareBackground, hardwareBackground);
    setUser(userData);

    // Send confirmation email after successful signup
    import('../services/emailService').then(({ sendConfirmationEmail }) => {
      sendConfirmationEmail(email, name);
    }).catch(error => {
      console.error('Error importing email service:', error);
    });
  };

  const signOut = async () => {
    await authService.signOut();
    setUser(null);
  };

  const updateUser = async (userData: Partial<AuthUser>) => {
    const updatedUser = await authService.updateCurrentUser(userData);
    setUser(updatedUser);
  };

  const value = {
    user,
    loading,
    signIn,
    signUp,
    signOut,
    updateUser
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};