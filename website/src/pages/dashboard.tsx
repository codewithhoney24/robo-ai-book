import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

import { useLocation } from '@docusaurus/router'; // URL params check karne ke liye
import { getStartingModulePath, getStartingModuleTitle } from '../utils/personalizationLogic';
import { usePersonalization } from '../contexts/PersonalizationContext';
import { useAuth } from '../contexts/AuthContext';
import PersonalizeButton from '../components/PersonalizeButton';

export default function Dashboard() {
  const { user, loading } = useAuth();
  const location = useLocation();
  const { settings } = usePersonalization();

  // URL se check karna ke user signup se aya hai ya signin se
  const params = new URLSearchParams(location.search);
  const isNewUser = params.get('type') === 'signup' || params.get('success') === 'signup';

  // Determine the appropriate starting module based on user's personalization settings
  const startingModulePath = getStartingModulePath(settings.preferences.softwareBackground, settings.preferences.hardwareBackground);
  const startingModuleTitle = getStartingModuleTitle(settings.preferences.softwareBackground, settings.preferences.hardwareBackground);

  if (loading) {
    return (
      <Layout title="Dashboard">
        <div style={{
          padding: '60px 20px',
          textAlign: 'center',
          backgroundColor: '#1b1b1d',
          minHeight: '70vh',
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center'
        }}>
          <p style={{ fontSize: '1.2rem', color: '#ccc' }}>Loading...</p>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Dashboard">
      <div style={{
        padding: '60px 20px',
        textAlign: 'center',
        backgroundColor: '#1b1b1d',
        minHeight: '70vh'
      }}>

        {/* Success Message Logic */}
        <div style={{ marginBottom: '20px' }}>
          {isNewUser ? (
            <div style={{ color: '#25c2a0', fontWeight: 'bold', fontSize: '1.4rem' }}>
              🎉 Welcome to the community.
            </div>
          ) : (
            <div style={{ color: '#25c2a0', fontWeight: 'bold', fontSize: '1.4rem' }}>
              Welcome! 🤖
            </div>
          )}
        </div>

        <h1 style={{ color: '#ffffff' }}>
          Welcome, Explorer! 🤖
        </h1>

        {/* English Translated Text */}
        <p style={{ fontSize: '1.2rem', marginBottom: '30px', color: '#ccc' }}>
          Your Robotics journey is still ongoing. Are you ready to start the next chapter?
        </p>

        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '20px' }}>
          {/* Main Learning Button - Only shown if user is email verified */}
          {user?.emailVerified ? (
            <Link
              className="button button--primary button--lg"
              to={startingModulePath}
              style={{
                padding: '15px 30px',
                fontSize: '1.2rem',
                boxShadow: '0 0 15px rgba(37, 194, 160, 0.4)'
              }}>
              🚀 Start Learning ({startingModuleTitle})
            </Link>
          ) : (
            <div style={{
              padding: '15px 30px',
              fontSize: '1.2rem',
              backgroundColor: '#444',
              borderRadius: '4px',
              color: '#ccc',
              cursor: 'not-allowed'
            }}>
              🚀 Start Learning ({startingModuleTitle})
            </div>
          )}

          {/* Show verification status message */}
          {!user?.emailVerified && (
            <div style={{
              padding: '10px',
              backgroundColor: '#ff6b6b',
              color: 'white',
              borderRadius: '4px',
              fontSize: '0.9rem',
              maxWidth: '500px',
              margin: '0 auto'
            }}>
              Please verify your email to access learning content.
            </div>
          )}

          {/* Secondary Home Button */}
          <Link
            className="button button--secondary button--lg"
            to="/"
            style={{ padding: '15px 30px', fontSize: '1.2rem', backgroundColor: '#00FFFF', boxShadow: '0 0 15px rgba(40, 38, 38, 0.2)' }}>
            Go to Home
          </Link>
        </div>

        {/* Personalization Section with Button */}
        <div style={{ marginTop: '50px', color: '#aaa', borderTop: '1px solid #333', paddingTop: '20px' }}>
          <h3 style={{ color: '#fff', marginBottom: '15px' }}>Your Learning Profile</h3>
          <p>Your Software Background: <b style={{color: '#fff'}}>{settings.preferences.softwareBackground || 'Not set'}</b></p>
          <p>Your Hardware Background: <b style={{color: '#fff'}}>{settings.preferences.hardwareBackground || 'Not set'}</b></p>
          <p>Your Learning Goal: <b style={{color: '#fff'}}>{settings.preferences.learningGoals || 'Not set'}</b></p>

          <div style={{ marginTop: '20px' }}>
            {user && <PersonalizeButton />}
          </div>
        </div>
      </div>
    </Layout>
  );
}