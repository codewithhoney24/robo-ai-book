import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';

export default function LayoutProviders({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}