import React from 'react';
import { useAuth } from '../contexts/AuthContext';

interface ProtectedRouteProps {
  children: React.ReactNode;
  // If requiredBackground is specified, user must have at least this level
  requiredBackground?: 'software' | 'hardware';
  // If hasSpecificBackground is specified, user must have this specific background
  hasSpecificBackground?: string;
  // Fallback component when user doesn't meet requirements
  fallback?: React.ReactNode;
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = ({
  children,
  requiredBackground,
  hasSpecificBackground,
  fallback = <div>Please sign in to access this content.</div>
}) => {
  const { user, loading } = useAuth();

  if (loading) {
    return <div>Loading...</div>;
  }

  // Check if user is authenticated
  if (!user) {
    return fallback;
  }

  // Check if user meets background requirements
  if (requiredBackground) {
    if (requiredBackground === 'software' && hasSpecificBackground) {
      if (user.softwareBackground !== hasSpecificBackground) {
        return fallback;
      }
    } else if (requiredBackground === 'hardware' && hasSpecificBackground) {
      if (user.hardwareBackground !== hasSpecificBackground) {
        return fallback;
      }
    }
  }

  // User has access to the content
  return <>{children}</>;
};

export default ProtectedRoute;