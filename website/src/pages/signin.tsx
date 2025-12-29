import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { API_CONFIG } from '../config/api';

const SigninPage: React.FC = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [rememberMe, setRememberMe] = useState(false);
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const history = useHistory();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      // Call the custom auth signin endpoint
      const response = await fetch(`${API_CONFIG.BASE_URL}/v1/auth/session`, {
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
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Failed to sign in. Please check your credentials and try again.');
      }

      const data = await response.json();

      // Store the session token
      if (data.access_token) {
        localStorage.setItem('better-auth-token', data.access_token);
      }

      // Redirect to dashboard or home page
      history.push('/dashboard?type=signin');
    } catch (err: any) {
      setError(err.message || 'Failed to sign in. Please check your credentials and try again.');
      console.error('Signin error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div style={{
      maxWidth: '500px',
      margin: '4rem auto',
      padding: '2rem',
      backgroundColor: '#0f172a',
      border: '1px solid rgba(0, 255, 255, 0.3)',
      borderRadius: '10px',
      boxShadow: '0 0 20px rgba(0, 255, 255, 0.2)'
    }}>
      <h1 style={{ color: '#00FFFF', textAlign: 'center', marginBottom: '1.5rem' }}>Sign In to Your Account</h1>
      {error && <div style={{ color: '#ff6b6b', marginBottom: '1rem', textAlign: 'center' }}>{error}</div>}

      <form onSubmit={handleSubmit}>
        <div style={{ marginBottom: '1.5rem' }}>
          <label htmlFor="email" style={{ display: 'block', marginBottom: '0.5rem', color: '#a5f3fc' }}>Email Address</label>
          <input
            id="email"
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
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
          <label htmlFor="password" style={{ display: 'block', marginBottom: '0.5rem', color: '#a5f3fc' }}>Password</label>
          <input
            id="password"
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
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

        <div style={{ marginBottom: '1.5rem', display: 'flex', alignItems: 'center', color: '#a5f3fc' }}>
          <input
            id="rememberMe"
            type="checkbox"
            checked={rememberMe}
            onChange={(e) => setRememberMe(e.target.checked)}
            style={{ marginRight: '0.5rem', width: '1rem', height: '1rem' }}
          />
          <label htmlFor="rememberMe">Remember me</label>
        </div>

        <button
          type="submit"
          disabled={loading}
          style={{
            width: '100%',
            padding: '0.75rem',
            backgroundColor: '#00FFFF',
            color: '#000',
            border: 'none',
            borderRadius: '5px',
            fontSize: '1.1rem',
            fontWeight: 'bold',
            cursor: loading ? 'not-allowed' : 'pointer',
            transition: 'all 0.3s ease'
          }}
          onMouseEnter={(e) => {
            if (!loading) {
              e.currentTarget.style.boxShadow = '0 0 15px rgba(0, 255, 255, 0.6)';
              e.currentTarget.style.backgroundColor = '#ccffff';
            }
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.boxShadow = 'none';
            e.currentTarget.style.backgroundColor = '#00FFFF';
          }}
        >
          {loading ? 'Signing In...' : 'Sign In'}
        </button>
      </form>

      <div style={{ marginTop: '1.5rem', textAlign: 'center' }}>
        <a href="/signup" style={{ color: '#00FFFF', textDecoration: 'underline' }}>Don't have an account? Sign up</a>
      </div>

      <div style={{ marginTop: '0.5rem', textAlign: 'center' }}>
        <a href="#" style={{ color: '#00FFFF', textDecoration: 'underline' }}>Forgot password?</a>
      </div>
    </div>
  );
};

export default SigninPage;