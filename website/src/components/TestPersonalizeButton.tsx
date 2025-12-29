import React, { useState } from 'react';
import PersonalizeButton from './PersonalizeButton';

// Mock AuthContext for testing purposes
const MockAuthContext = ({ children, isAuthenticated = false }: { children: React.ReactNode, isAuthenticated?: boolean }) => {
  const [user, setUser] = useState(isAuthenticated ? { 
    id: 'test-user-id', 
    email: 'test@example.com', 
    name: 'Test User',
    softwareBackground: 'beginner',
    hardwareBackground: 'rtx_laptop'
  } : null);

  const signIn = () => {
    setUser({ 
      id: 'test-user-id', 
      email: 'test@example.com', 
      name: 'Test User',
      softwareBackground: 'beginner',
      hardwareBackground: 'rtx_laptop'
    });
  };

  const signOut = () => {
    setUser(null);
  };

  return (
    <div>
      <div style={{ padding: '10px', backgroundColor: '#f0f0f0', marginBottom: '20px' }}>
        <strong>Auth Status:</strong> {user ? 'Authenticated' : 'Not Authenticated'}
        <button 
          onClick={user ? signOut : signIn} 
          style={{ marginLeft: '10px', padding: '5px 10px' }}
        >
          {user ? 'Sign Out' : 'Sign In'}
        </button>
      </div>
      <div>
        {children}
      </div>
    </div>
  );
};

const TestPersonalizeButton = () => {
  return (
    <div style={{ padding: '20px' }}>
      <h1>Personalize Button Test</h1>
      <p>This page demonstrates the locked personalization button functionality.</p>
      
      <h2>Test 1: Not Authenticated (Button should be locked)</h2>
      <MockAuthContext isAuthenticated={false}>
        <PersonalizeButton />
      </MockAuthContext>
      
      <h2>Test 2: Authenticated (Button should be unlocked)</h2>
      <MockAuthContext isAuthenticated={true}>
        <PersonalizeButton />
      </MockAuthContext>
      
      <h2>Test 3: Interactive Test</h2>
      <p>Use the sign in/out button above to test the locking/unlocking functionality</p>
      <MockAuthContext isAuthenticated={false}>
        <PersonalizeButton />
      </MockAuthContext>
    </div>
  );
};

export default TestPersonalizeButton;