import React from 'react';
import {AuthProvider} from '../contexts/AuthContext';
import {PersonalizationProvider} from '../contexts/PersonalizationContext';

// This Root component wraps the entire Docusaurus app with AuthContext and PersonalizationContext
export default function Root({children}: {children: React.ReactNode}) {
  return (
    <AuthProvider>
      <PersonalizationProvider>
        {children}
      </PersonalizationProvider>
    </AuthProvider>
  );
}