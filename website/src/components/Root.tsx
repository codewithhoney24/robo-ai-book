import React from 'react';
import {AuthProvider} from '../contexts/AuthContext';

// This Root component wraps the entire Docusaurus app with AuthContext
export default function Root({children}: {children: React.ReactNode}) {
  return <AuthProvider>{children}</AuthProvider>;
}