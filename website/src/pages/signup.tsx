
import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { usePersonalization } from '../contexts/PersonalizationContext';
import { useAuth } from '../contexts/AuthContext';
import { API_CONFIG } from '../config/api';

interface AuthUser {
  id: string;
  email: string;
  name: string;
  emailVerified: boolean;
  softwareBackground?: 'beginner' | 'intermediate' | 'advanced' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert';
  hardwareBackground?: 'none' | 'beginner' | 'intermediate' | 'advanced' | 'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud';
  createdAt?: string;
}

const SignupPage: React.FC = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState<'beginner' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert'>('beginner');
  const [hardwareBackground, setHardwareBackground] = useState<'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud'>('no_gpu');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { updatePersonalization } = usePersonalization();
  const { signUp } = useAuth(); // Use the AuthContext for signup
  const history = useHistory();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      // Use the AuthContext signup method which handles the API call and token storage
      await signUp(
        email,
        password,
        name,
        softwareBackground,
        hardwareBackground
      );

      // Update personalization with the background information
      updatePersonalization({
        preferences: {
          softwareBackground: softwareBackground as any, // Type assertion to match PersonalizationContext
          hardwareBackground: hardwareBackground as any, // Type assertion to match PersonalizationContext
          difficulty: 'beginner',
          learningStyle: 'visual',
          advancedContent: false,
          foundationalKnowledge: false,
          complexConcepts: false,
          progressTracking: false,
          moduleSuggestions: false,
          contentRecommendation: false,
          learningSpeed: 'slow',
          progressiveDisclosure: false,
          adaptiveDifficulty: false,
          smartAdaptation: false,
          quizResults: false,
          contentAdjustment: false,
          targetedHelp: false,
          interactionPatterns: false,
          contentAccess: 'simulation',
          learningGoals: 'research',
          contentContext: '',
          complexityLevel: 'basic',
          preferredTopics: [],
          codeExamples: false,
          quizzesAssessments: false,
          practiceSessions: false,
          customExercises: false
        }
      });

      // Redirect to dashboard or home page after successful signup
      history.push('/dashboard?type=signup');
    } catch (err: any) {
      setError(err.message || 'Failed to create account. Please try again.');
      console.error('Signup error:', err);
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
      <h1 style={{ color: '#00FFFF', textAlign: 'center', marginBottom: '1.5rem' }}>Create Account</h1>
      {error && <div style={{ color: '#ff6b6b', marginBottom: '1rem', textAlign: 'center' }}>{error}</div>}

      <form onSubmit={handleSubmit}>
        <div style={{ marginBottom: '1.5rem' }}>
          <label htmlFor="name" style={{ display: 'block', marginBottom: '0.5rem', color: '#a5f3fc' }}>Full Name</label>
          <input
            id="name"
            type="text"
            value={name}
            onChange={(e) => setName(e.target.value)}
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
            minLength={8}
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
          <label htmlFor="softwareBackground" style={{ display: 'block', marginBottom: '0.5rem', color: '#a5f3fc' }}>Software Background</label>
          <select
            id="softwareBackground"
            value={softwareBackground}
            onChange={(e) => setSoftwareBackground(e.target.value as any)}
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
          >
            <option value="" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>Select your software background</option>
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
          >
            <option value="" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>Select your hardware availability</option>
            <option value="no_gpu" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>No GPU - Standard laptop</option>
            <option value="rtx_laptop" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>RTX Laptop - RTX 2060-4060</option>
            <option value="rtx_workstation" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>RTX Workstation - RTX 3090/4090</option>
            <option value="jetson_kit" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>Jetson Kit - Jetson Orin</option>
            <option value="cloud" style={{ backgroundColor: '#0f172a', color: '#00FFFF' }}>Cloud - AWS/Azure</option>
          </select>
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
          {loading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>

      <div style={{ marginTop: '1.5rem', textAlign: 'center' }}>
        Already have an account? <a href="/signin" style={{ color: '#00FFFF', textDecoration: 'underline' }}>Sign in</a>
      </div>
    </div>
  );
};

export default SignupPage;