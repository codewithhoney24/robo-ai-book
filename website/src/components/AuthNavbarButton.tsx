import React, { useEffect, useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';

const AuthNavbarButton: React.FC = () => {
  const { user, loading, signOut } = useAuth();
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const history = useHistory();

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      const target = event.target as HTMLElement;
      if (isDropdownOpen && !target.closest('.auth-navbar-button')) {
        setIsDropdownOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isDropdownOpen]);

  if (loading) {
    return (
      <div className="auth-navbar-button" style={{
        color: '#00FFFF',
        fontWeight: 'bold',
        padding: '0.25rem 0.75rem',
        border: '1px solid #00FFFF',
        borderRadius: '20px',
        cursor: 'default'
      }}>
        Loading...
      </div>
    );
  }

  return (
    <div className="auth-navbar-button" style={{ position: 'relative' }}>
      <button
        onClick={user ? () => setIsDropdownOpen(!isDropdownOpen) : () => history.push('/signin')}
        style={{
          color: '#000',
          backgroundColor: '#00FFFF',
          border: '1px solid #00FFFF',
          borderRadius: '20px',
          padding: '0.25rem 0.75rem',
          fontWeight: 'bold',
          cursor: 'pointer',
          fontSize: '0.9rem',
          transition: 'all 0.3s ease',
          minWidth: '100px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center'
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.boxShadow = '0 0 10px rgba(0, 255, 255, 0.6)';
          e.currentTarget.style.backgroundColor = '#ccffff';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.boxShadow = 'none';
          e.currentTarget.style.backgroundColor = '#00FFFF';
        }}
      >
        {user ? `Hi, ${user.name}` : 'Sign In'}
      </button>

      {user && isDropdownOpen && (
        <div
          style={{
            position: 'absolute',
            top: '100%',
            right: 0,
            backgroundColor: '#0f172a',
            border: '1px solid #00FFFF',
            borderRadius: '8px',
            padding: '0.5rem 0',
            width: '150px',
            zIndex: 1000,
            boxShadow: '0 4px 12px rgba(0, 255, 255, 0.2)',
          }}
        >
          <button
            onClick={() => {
              history.push('/profile');
              setIsDropdownOpen(false);
            }}
            style={{
              width: '100%',
              padding: '0.5rem 1rem',
              textAlign: 'left',
              border: 'none',
              backgroundColor: 'transparent',
              color: '#00FFFF',
              cursor: 'pointer',
              fontSize: '0.9rem',
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.backgroundColor = 'rgba(0, 255, 255, 0.1)';
              e.currentTarget.style.color = '#ffffff';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.backgroundColor = 'transparent';
              e.currentTarget.style.color = '#00FFFF';
            }}
          >
            Profile
          </button>
          <button
            onClick={async () => {
              await signOut();
              history.push('/');
              setIsDropdownOpen(false);
            }}
            style={{
              width: '100%',
              padding: '0.5rem 1rem',
              textAlign: 'left',
              border: 'none',
              backgroundColor: 'transparent',
              color: '#00FFFF',
              cursor: 'pointer',
              fontSize: '0.9rem',
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.backgroundColor = 'rgba(0, 255, 255, 0.1)';
              e.currentTarget.style.color = '#ffffff';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.backgroundColor = 'transparent';
              e.currentTarget.style.color = '#00FFFF';
            }}
          >
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
};

export default AuthNavbarButton;