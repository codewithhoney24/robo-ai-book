import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface User {
  id: string;
  email: string;
  name: string;
  softwareBackground: string;
  hardwareBackground: string;
  emailVerified: boolean;
  createdAt: string;
  updatedAt: string;
  programmingLevel: string;
  knownLanguages: string;
  aiMlExperience: boolean;
  rosRoboticsExperience: boolean;
  hasJetson: boolean;
  hasRTXGPU: boolean;
  hasRobotHardware: boolean;
  simulationOnly: boolean;
}

interface SignupData {
  name: string;
  email: string;
  password: string;
  softwareBackground: string;
  hardwareBackground: string;
  programmingLevel: string;
  knownLanguages: string; // JSON string of languages
  aiMlExperience: boolean;
  rosRoboticsExperience: boolean;
  hasJetson: boolean;
  hasRTXGPU: boolean;
  hasRobotHardware: boolean;
  simulationOnly: boolean;
}

interface AuthContextType {
  user: User | null;
  loading: boolean;
  login: (email: string, password: string) => Promise<void>;
  signup: (data: SignupData) => Promise<void>;
  logout: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

const AUTH_API_URL = 'http://localhost:8000/api/auth';

export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    checkSession();
  }, []);

  const checkSession = async () => {
    try {
      const response = await fetch(`${AUTH_API_URL}/get-session`, {
        credentials: 'include',
      });

      if (response.ok) {
        const data = await response.json();
        if (data.session?.user) {
          setUser(data.session.user);
        }
      }
    } catch (error) {
      console.error('Session check failed:', error);
    } finally {
      setLoading(false);
    }
  };

  const signup = async (data: SignupData) => {
    const response = await fetch(`${AUTH_API_URL}/sign-up`, { // Using our custom endpoint
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify({
        email: data.email,
        password: data.password,
        name: data.name,
        softwareBackground: data.softwareBackground,
        hardwareBackground: data.hardwareBackground,
        programmingLevel: data.programmingLevel,
        knownLanguages: data.knownLanguages,
        aiMlExperience: data.aiMlExperience,
        rosRoboticsExperience: data.rosRoboticsExperience,
        hasJetson: data.hasJetson,
        hasRTXGPU: data.hasRTXGPU,
        hasRobotHardware: data.hasRobotHardware,
        simulationOnly: data.simulationOnly,
      }),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || 'Signup failed');
    }

    const result = await response.json();
    if (result.user) {
      setUser(result.user);
    }
  };

  const login = async (email: string, password: string) => {
    const response = await fetch(`${AUTH_API_URL}/sign-in`, { // Using our custom endpoint
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify({ email, password }),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message || 'Login failed');
    }

    const result = await response.json();
    if (result.user) {
      setUser(result.user);
    }
  };

  const logout = async () => {
    try {
      await fetch(`${AUTH_API_URL}/sign-out`, {
        method: 'POST',
        credentials: 'include',
      });
    } catch (error) {
      console.error('Logout error:', error);
    } finally {
      setUser(null);
    }
  };

  return (
    <AuthContext.Provider value={{ user, loading, login, signup, logout }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
};