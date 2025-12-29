import React, { useState, useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import { usePersonalization } from '../contexts/PersonalizationContext';
import { API_CONFIG } from '../config/api';

interface AuthUser {
  id: string;
  email: string;
  name: string;
  softwareBackground?: 'beginner' | 'intermediate' | 'advanced' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert';
  hardwareBackground?: 'none' | 'beginner' | 'intermediate' | 'advanced' | 'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud';
  createdAt?: string;
}

const ProfilePage: React.FC = () => {
  const [user, setUser] = useState<AuthUser | null>(null);
  const [loading, setLoading] = useState(true);
  const { personalizationData, updatePersonalization } = usePersonalization();
  const [name, setName] = useState('');
  const history = useHistory();
  const [softwareBackground, setSoftwareBackground] = useState<'beginner' | 'intermediate' | 'advanced' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert'>('beginner');
  const [hardwareBackground, setHardwareBackground] = useState<'none' | 'beginner' | 'intermediate' | 'advanced' | 'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud'>('none');
  const [isEditing, setIsEditing] = useState(false);
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [message, setMessage] = useState('');

  // Fetch user data on component mount
  useEffect(() => {
    const fetchUser = async () => {
      try {
        const response = await fetch(`${API_CONFIG.BASE_URL}/v1/auth/session`, {
          method: 'GET',
          headers: {
            'Authorization': `Bearer ${localStorage.getItem('better-auth-token') || ''}`,
          },
        });

        if (response.ok) {
          const userData = await response.json();
          setUser(userData.user);
          setName(userData.user.name || '');
          setSoftwareBackground(userData.user.softwareBackground || 'beginner');
          setHardwareBackground(userData.user.hardwareBackground || 'none');
        } else {
          // If not authenticated, redirect to sign in
          history.push('/signin');
        }
      } catch (error) {
        console.error('Error fetching user data:', error);
        setMessage('Error loading user data');
      } finally {
        setLoading(false);
      }
    };

    fetchUser();
  }, [history]);

  const handleUpdateProfile = async (e: React.FormEvent) => {
    e.preventDefault();

    try {
      // Update user profile via API
      const response = await fetch(`${API_CONFIG.BASE_URL}/v1/auth/update-background`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('better-auth-token') || ''}`,
        },
        body: JSON.stringify({
          softwareBackground,
          hardwareBackground
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Failed to update profile');
      }

      const updatedUser = await response.json();
      setUser(updatedUser);

      // Update personalization preferences
      updatePersonalization({
        preferences: {
          ...personalizationData.preferences,
          softwareBackground: softwareBackground as any,
          hardwareBackground: hardwareBackground as any
        }
      });

      setMessage('Profile updated successfully!');
      setIsEditing(false);
    } catch (error: any) {
      setMessage(error.message || 'Failed to update profile. Please try again.');
      console.error('Update profile error:', error);
    }
  };

  const handleSignOut = async () => {
    try {
      // Call the signout endpoint
      await fetch(`${API_CONFIG.BASE_URL}/v1/auth/session`, {
        method: 'DELETE',
        headers: {
          'Authorization': `Bearer ${localStorage.getItem('better-auth-token') || ''}`,
        },
      });

      // Clear the local storage
      localStorage.removeItem('better-auth-token');

      // Redirect to sign in page
      history.push('/signin');
    } catch (error) {
      console.error('Sign out error:', error);
    }
  };

  const handleChangePassword = async (e: React.FormEvent) => {
    e.preventDefault();

    if (password !== confirmPassword) {
      setMessage('Passwords do not match');
      return;
    }

    // In a real implementation, this would call the backend API to change password
    // For now, we'll just show a success message
    setMessage('Password change functionality would be implemented here');
    setPassword('');
    setConfirmPassword('');
  };

  if (loading) {
    return <div>Loading...</div>;
  }

  if (!user) {
    return <div>Please sign in to view your profile.</div>;
  }

  return (
    <div style={{
      maxWidth: '700px',
      margin: '4rem auto',
      padding: '2rem',
      backgroundColor: '#0f172a',
      border: '1px solid rgba(0, 255, 255, 0.3)',
      borderRadius: '10px',
      boxShadow: '0 0 20px rgba(0, 255, 255, 0.2)'
    }}>
      <h1 style={{ color: '#00FFFF', textAlign: 'center', marginBottom: '1.5rem' }}>User Profile</h1>

      {message && <div style={{ color: '#20bf6b', marginBottom: '1rem', textAlign: 'center' }}>{message}</div>}

      <div style={{ marginBottom: '2rem' }}>
        <h2 style={{ color: '#00FFFF', marginBottom: '1rem' }}>Account Information</h2>
        <p style={{ color: '#a5f3fc', marginBottom: '1rem' }}><strong>Email:</strong> {user.email}</p>

        <form onSubmit={handleUpdateProfile}>
          <div style={{ marginBottom: '1.5rem' }}>
            <label htmlFor="name" style={{ display: 'block', marginBottom: '0.5rem', color: '#a5f3fc' }}>Name</label>
            <input
              id="name"
              type="text"
              value={name}
              onChange={(e) => setName(e.target.value)}
              disabled={!isEditing}
              style={{
                width: '100%',
                padding: '0.75rem',
                backgroundColor: isEditing ? 'rgba(15, 23, 42, 0.8)' : 'rgba(15, 23, 42, 0.5)',
                border: '1px solid rgba(0, 255, 255, 0.3)',
                borderRadius: '5px',
                color: '#00FFFF',
                fontSize: '1rem'
              }}
            />
          </div>

          <div style={{ marginBottom: '1.5rem' }}>
            <label htmlFor="softwareBackground" style={{ display: 'block', marginBottom: '0.5rem', color: '#a5f3fc' }}>Software Background</label>
            <select
              id="softwareBackground"
              value={softwareBackground}
              onChange={(e) => setSoftwareBackground(e.target.value as any)}
              disabled={!isEditing}
              style={{
                width: '100%',
                padding: '0.75rem',
                backgroundColor: isEditing ? 'rgba(15, 23, 42, 0.8)' : 'rgba(15, 23, 42, 0.5)',
                border: '1px solid rgba(0, 255, 255, 0.3)',
                borderRadius: '5px',
                color: '#00FFFF',
                fontSize: '1rem'
              }}
            >
              <option value="beginner" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>Beginner - No Python/C++ experience</option>
              <option value="python_intermediate" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>Python Intermediate - Python basics</option>
              <option value="ros2_developer" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>ROS 2 Developer - ROS 1/2 experience</option>
              <option value="ai_robotics_expert" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>AI Robotics Expert - Professional level</option>
            </select>
          </div>

          <div style={{ marginBottom: '1.5rem' }}>
            <label htmlFor="hardwareBackground" style={{ display: 'block', marginBottom: '0.5rem', color: '#a5f3fc' }}>Hardware Availability</label>
            <select
              id="hardwareBackground"
              value={hardwareBackground}
              onChange={(e) => setHardwareBackground(e.target.value as any)}
              disabled={!isEditing}
              style={{
                width: '100%',
                padding: '0.75rem',
                backgroundColor: isEditing ? 'rgba(15, 23, 42, 0.8)' : 'rgba(15, 23, 42, 0.5)',
                border: '1px solid rgba(0, 255, 255, 0.3)',
                borderRadius: '5px',
                color: '#00FFFF',
                fontSize: '1rem'
              }}
            >
              <option value="no_gpu" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>No GPU - Standard laptop</option>
              <option value="rtx_laptop" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>RTX Laptop - RTX 2060-4060</option>
              <option value="rtx_workstation" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>RTX Workstation - RTX 3090/4090</option>
              <option value="jetson_kit" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>Jetson Kit - Jetson Orin</option>
              <option value="cloud" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>Cloud - AWS/Azure</option>
            </select>
          </div>

          {isEditing ? (
            <>
              <button
                type="submit"
                style={{
                  marginRight: '0.5rem',
                  padding: '0.75rem 1.5rem',
                  backgroundColor: '#00FFFF',
                  color: '#000',
                  border: 'none',
                  borderRadius: '5px',
                  fontWeight: 'bold',
                  cursor: 'pointer',
                  fontSize: '1rem',
                  transition: 'all 0.3s ease'
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
                Save Changes
              </button>
              <button
                type="button"
                onClick={() => {
                  setIsEditing(false);
                  // Reset values if user cancels
                  setName(user.name || '');
                  setSoftwareBackground(user.softwareBackground || 'beginner');
                  setHardwareBackground(user.hardwareBackground || 'none');
                }}
                style={{
                  padding: '0.75rem 1.5rem',
                  backgroundColor: '#6c757d',
                  color: 'white',
                  border: 'none',
                  borderRadius: '5px',
                  fontWeight: 'bold',
                  cursor: 'pointer',
                  fontSize: '1rem',
                  transition: 'all 0.3s ease'
                }}
                onMouseEnter={(e) => {
                  e.currentTarget.style.backgroundColor = '#5a6268';
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.backgroundColor = '#6c757d';
                }}
              >
                Cancel
              </button>
            </>
          ) : (
            <button
              type="button"
              onClick={() => setIsEditing(true)}
              style={{
                padding: '0.75rem 1.5rem',
                backgroundColor: '#28a745',
                color: 'white',
                border: 'none',
                borderRadius: '5px',
                fontWeight: 'bold',
                cursor: 'pointer',
                fontSize: '1rem',
                transition: 'all 0.3s ease'
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.backgroundColor = '#218838';
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.backgroundColor = '#28a745';
              }}
            >
              Edit Profile
            </button>
          )}
        </form>
      </div>

      <div style={{ marginBottom: '2rem' }}>
        <h2 style={{ color: '#00FFFF', marginBottom: '1rem' }}>Change Password</h2>
        <form onSubmit={handleChangePassword}>
          <div style={{ marginBottom: '1.5rem' }}>
            <label htmlFor="newPassword" style={{ display: 'block', marginBottom: '0.5rem', color: '#a5f3fc' }}>New Password</label>
            <input
              id="newPassword"
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="Enter new password"
              style={{
                width: '100%',
                padding: '0.75rem',
                backgroundColor: 'rgba(15, 23, 42, 0.8)',
                border: '1px solid rgba(0, 255, 255, 0.3)',
                borderRadius: '5px',
                color: '#00FFFF',
                fontSize: '1rem'
              }}
            />
          </div>

          <div style={{ marginBottom: '1.5rem' }}>
            <label htmlFor="confirmPassword" style={{ display: 'block', marginBottom: '0.5rem', color: '#a5f3fc' }}>Confirm Password</label>
            <input
              id="confirmPassword"
              type="password"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              placeholder="Confirm new password"
              style={{
                width: '100%',
                padding: '0.75rem',
                backgroundColor: 'rgba(15, 23, 42, 0.8)',
                border: '1px solid rgba(0, 255, 255, 0.3)',
                borderRadius: '5px',
                color: '#00FFFF',
                fontSize: '1rem'
              }}
            />
          </div>

          <button
            type="submit"
            style={{
              padding: '0.75rem 1.5rem',
              backgroundColor: '#ffc107',
              color: '#000',
              border: 'none',
              borderRadius: '5px',
              fontWeight: 'bold',
              cursor: 'pointer',
              fontSize: '1rem',
              transition: 'all 0.3s ease'
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.boxShadow = '0 0 10px rgba(255, 193, 7, 0.6)';
              e.currentTarget.style.backgroundColor = '#e0a800';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.boxShadow = 'none';
              e.currentTarget.style.backgroundColor = '#ffc107';
            }}
          >
            Change Password
          </button>
        </form>
      </div>

      <div>
        <button
          onClick={handleSignOut}
          style={{
            padding: '0.75rem 1.5rem',
            backgroundColor: '#dc3545',
            color: 'white',
            border: 'none',
            borderRadius: '5px',
            fontWeight: 'bold',
            cursor: 'pointer',
            fontSize: '1rem',
            transition: 'all 0.3s ease'
          }}
          onMouseEnter={(e) => {
            e.currentTarget.style.boxShadow = '0 0 10px rgba(220, 53, 69, 0.6)';
            e.currentTarget.style.backgroundColor = '#c82333';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.boxShadow = 'none';
            e.currentTarget.style.backgroundColor = '#dc3545';
          }}
        >
          Sign Out
        </button>
      </div>
    </div>
  );
};

export default ProfilePage;