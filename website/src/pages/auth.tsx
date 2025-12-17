import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';



const AuthPage: React.FC = () => {
  const { login, signup } = useAuth();
  const [currentView, setCurrentView] = useState<'signin' | 'signup'>('signin');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Sign In form state
  const [signinData, setSigninData] = useState({
    email: '',
    password: '',
  });

  // Sign Up form state with background questions
  const [signupData, setSignupData] = useState({
    name: '',
    email: '',
    password: '',
    softwareBackground: 'beginner' as 'beginner' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert',
    hardwareBackground: 'no_gpu' as 'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud',
  });

  // GitHub OAuth handler
  const handleGitHubSignIn = async () => {
    try {
      setLoading(true);
      setError(null);

      // Check if window.location is available (client-side)
      if (typeof window !== 'undefined') {
        // Redirect to backend for GitHub OAuth flow
        // For now, we'll use a placeholder since GitHub OAuth needs specific setup
        window.location.href = '/api/auth/login/github';
      } else {
        // This shouldn't happen in this context, but just in case
        throw new Error('Window object is not available');
      }
    } catch (err) {
      console.error('GitHub sign-in error:', err);
      setError(err instanceof Error ? err.message : 'Sign in with GitHub failed. Please check your connection and try again.');
      setLoading(false);
    }
  };

  // Reset error when switching between views
  const switchView = (view: 'signin' | 'signup') => {
    setCurrentView(view);
    setError(null);
  };

  // Sign In form handlers
  const handleSigninChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setSigninData(prev => ({
      ...prev,
      [name]: value
    }));
    // Clear error when user starts typing
    if (error) setError(null);
  };

  const handleSigninSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      await login(signinData.email, signinData.password);
      // Redirect to dashboard or home page
      window.location.href = '/dashboard';
    } catch (error) {
      console.error('Sign in error:', error);
      setError(error instanceof Error ? error.message : 'Sign in failed. Please check your connection and try again.');
    } finally {
      setLoading(false);
    }
  };

  // Sign Up form handlers
  const handleSignupChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setSignupData(prev => ({
      ...prev,
      [name]: value
    }));
    // Clear error when user starts typing
    if (error) setError(null);
  };

  const handleSignupSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      // Register the user with custom fields
      await signup({
        name: signupData.name,
        email: signupData.email,
        password: signupData.password,
        softwareBackground: signupData.softwareBackground,
        hardwareBackground: signupData.hardwareBackground,
        programmingLevel: '',
        knownLanguages: '',
        aiMlExperience: false,
        rosRoboticsExperience: false,
        hasJetson: false,
        hasRTXGPU: false,
        hasRobotHardware: false,
        simulationOnly: false
      });

      // Redirect to dashboard or home page
      window.location.href = '/dashboard';
    } catch (error) {
      console.error('Sign up error:', error);
      setError(error instanceof Error ? error.message : 'Sign up failed. Please check your connection and try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div style={{ maxWidth: '400px', margin: '3rem auto', padding: '0 1rem' }}>
      <div style={{
        border: '1px solid #95d5d5ff',
        borderRadius: '8px',
        padding: '1rem',
        backgroundColor: '#424949ff'
      }}>
        <div style={{ display: 'flex', marginBottom: '1rem' }}>
          <button
            style={{
              flex: 1,
              padding: '0.75rem',
              border: '1px solid #065055ff',
              backgroundColor: currentView === 'signin' ? '#00bab7ff' : '#3c7175ff',
              color: currentView === 'signin' ? 'white' : '#212529',
              cursor: 'pointer',
              borderRadius: '4px 0 0 4px',
            }}
            onClick={() => switchView('signin')}
          >
            Sign In
          </button>
          <button
            style={{
              flex: 1,
              padding: '0.75rem',
              border: '1px solid #065055ff',
              backgroundColor: currentView === 'signup' ? '#00bab7ff' : '#3c7175ff',
              color: currentView === 'signup' ? 'white' : '#212529',
              cursor: 'pointer',
              borderRadius: '0 4px 4px 0',
            }}
            onClick={() => switchView('signup')}
          >
            Sign Up
          </button>
        </div>

        <div style={{ padding: '1rem 0' }}>
          {/* Error message display */}
          {error && (
            <div style={{
              padding: '0.75rem',
              marginBottom: '1rem',
              backgroundColor: '#dc3545',
              color: 'white',
              borderRadius: '4px',
              border: '1px solid #dc3545'
            }}>
              {error}
            </div>
          )}

          {currentView === 'signin' ? (
            <form onSubmit={handleSigninSubmit}>
              <div style={{ marginBottom: '1rem' }}>
                <label htmlFor="signinEmail" style={{ display: 'block', marginBottom: '0.25rem' }}>Email</label>
                <input
                  type="email"
                  id="signinEmail"
                  name="email"
                  value={signinData.email}
                  onChange={handleSigninChange}
                  required
                  style={{
                    width: '100%',
                    padding: '0.5rem',
                    border: '1px solid  #065055ff',
                    borderRadius: '4px',
                  }}
                />
              </div>

              <div style={{ marginBottom: '1rem' }}>
                <label htmlFor="signinPassword" style={{ display: 'block', marginBottom: '0.25rem' }}>Password</label>
                <input
                  type="password"
                  id="signinPassword"
                  name="password"
                  value={signinData.password}
                  onChange={handleSigninChange}
                  required
                  style={{
                    width: '100%',
                    padding: '0.5rem',
                    border: '1px solid #065055ff',
                    borderRadius: '4px',
                  }}
                />
              </div>

              <button
                type="submit"
                disabled={loading}
                style={{
                  width: '100%',
                  padding: '0.75rem',
                  backgroundColor: loading ? '#95d5d5' : '#00bab7ff',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: loading ? 'not-allowed' : 'pointer',
                  opacity: loading ? 0.6 : 1,
                  position: 'relative'
                }}
              >
                {loading ? (
                  <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                    <span className="spinner" style={{
                      width: '12px',
                      height: '12px',
                      border: '2px solid transparent',
                      borderLeft: '2px solid white',
                      borderRadius: '50%',
                      animation: 'spin 1s linear infinite',
                      marginRight: '8px'
                    }}></span>
                    Signing In...
                  </span>
                ) : 'Sign In'}
              </button>
            </form>
          ) : (
            <form onSubmit={handleSignupSubmit}>
              <div style={{ marginBottom: '1rem' }}>
                <label htmlFor="signupName" style={{ display: 'block', marginBottom: '0.25rem' }}>Full Name</label>
                <input
                  type="text"
                  id="signupName"
                  name="name"
                  value={signupData.name}
                  onChange={handleSignupChange}
                  required
                  style={{
                    width: '100%',
                    padding: '0.5rem',
                    border: '1px solid  #065055ff',
                    borderRadius: '4px',
                  }}
                />
              </div>

              <div style={{ marginBottom: '1rem' }}>
                <label htmlFor="signupEmail" style={{ display: 'block', marginBottom: '0.25rem' }}>Email</label>
                <input
                  type="email"
                  id="signupEmail"
                  name="email"
                  value={signupData.email}
                  onChange={handleSignupChange}
                  required
                  style={{
                    width: '100%',
                    padding: '0.5rem',
                    border: '1px solid  #065055ff',
                    borderRadius: '4px',
                  }}
                />
              </div>

              <div style={{ marginBottom: '1rem' }}>
                <label htmlFor="signupPassword" style={{ display: 'block', marginBottom: '0.25rem' }}>Password</label>
                <input
                  type="password"
                  id="signupPassword"
                  name="password"
                  value={signupData.password}
                  onChange={handleSignupChange}
                  minLength={8}
                  required
                  style={{
                    width: '100%',
                    padding: '0.5rem',
                    border: '1px solid #065055ff',
                    borderRadius: '4px',
                  }}
                />
                <div style={{ fontSize: '0.875rem', color: '#6c757d', marginTop: '0.25rem' }}>
                  Password must be at least 8 characters
                </div>
              </div>

              <div style={{ marginBottom: '1rem' }}>
                <label htmlFor="signupSoftwareBackground" style={{ display: 'block', marginBottom: '0.25rem' }}>
                  Software Background
                </label>
                <select
                  id="signupSoftwareBackground"
                  name="softwareBackground"
                  value={signupData.softwareBackground}
                  onChange={handleSignupChange}
                  required
                  style={{
                    width: '100%',
                    padding: '0.5rem',
                    border: '1px solid  #065055ff',
                    borderRadius: '4px',
                  }}
                >
                  <option value="beginner">No Python/C++ experience</option>
                  <option value="python_intermediate">Python basics</option>
                  <option value="ros2_developer">ROS 1/2 experience</option>
                  <option value="ai_robotics_expert">Professional level</option>
                </select>
              </div>

              <div style={{ marginBottom: '1rem' }}>
                <label htmlFor="signupHardwareBackground" style={{ display: 'block', marginBottom: '0.25rem' }}>
                  Hardware Availability
                </label>
                <select
                  id="signupHardwareBackground"
                  name="hardwareBackground"
                  value={signupData.hardwareBackground}
                  onChange={handleSignupChange}
                  required
                  style={{
                    width: '100%',
                    padding: '0.5rem',
                    border: '1px solid  #065055ff',
                    borderRadius: '4px',
                  }}
                >
                  <option value="no_gpu">Standard laptop</option>
                  <option value="rtx_laptop">RTX 2060–4060</option>
                  <option value="rtx_workstation">RTX 3090/4090</option>
                  <option value="jetson_kit">Jetson Orin</option>
                  <option value="cloud">AWS/Azure</option>
                </select>
                <div style={{ fontSize: '0.875rem', color: '#6c757d', marginTop: '0.25rem' }}>
                  We'll use this to customize content based on your available hardware
                </div>
              </div>

              <button
                type="submit"
                disabled={loading}
                style={{
                  width: '100%',
                  padding: '0.75rem',
                  backgroundColor: loading ? '#95d5d5' : '#00bab7ff',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: loading ? 'not-allowed' : 'pointer',
                  opacity: loading ? 0.6 : 1,
                  position: 'relative'
                }}
              >
                {loading ? (
                  <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                    <span style={{
                      width: '12px',
                      height: '12px',
                      border: '2px solid transparent',
                      borderLeft: '2px solid white',
                      borderRadius: '50%',
                      animation: 'spin 1s linear infinite',
                      marginRight: '8px'
                    }}></span>
                    Signing Up...
                  </span>
                ) : 'Sign Up'}
              </button>
            </form>
          )}

          {/* GitHub OAuth Button */}
          <div style={{ marginTop: '1rem', textAlign: 'center' }}>
            <button
              onClick={handleGitHubSignIn}
              disabled={loading}
              style={{
                width: '100%',
                padding: '0.75rem',
                backgroundColor: loading ? '#95d5d5' : '#333',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: loading ? 'not-allowed' : 'pointer',
                opacity: loading ? 0.6 : 1,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                marginBottom: '1rem'
              }}
            >
              {loading ? (
                <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                  <span style={{
                    width: '12px',
                    height: '12px',
                    border: '2px solid transparent',
                    borderLeft: '2px solid white',
                    borderRadius: '50%',
                    animation: 'spin 1s linear infinite',
                    marginRight: '8px'
                  }}></span>
                  Signing in with GitHub...
                </span>
              ) : (
                <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                  <svg width="18" height="18" viewBox="0 0 16 16" fill="none" style={{ marginRight: '8px' }}>
                    <path
                      d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.012 8.012 0 0 0 16 8c0-4.42-3.58-8-8-8z"
                      fill="currentColor"
                    />
                  </svg>
                  Sign in with GitHub
                </span>
              )}
            </button>
          </div>

          <div style={{ marginTop: '1rem', textAlign: 'center' }}>
            {currentView === 'signin' ? (
              <span>
                Don't have an account?{' '}
                <a
                  href="#"
                  onClick={(e) => { e.preventDefault(); switchView('signup'); }}
                  style={{ color: '#00bab7ff' }}
                >
                  Sign up
                </a>
              </span>
            ) : (
              <span>
                Already have an account?{' '}
                <a
                  href="#"
                  onClick={(e) => { e.preventDefault(); switchView('signin'); }}
                  style={{ color: '#00bab7ff' }}
                >
                  Sign in
                </a>
              </span>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default AuthPage;