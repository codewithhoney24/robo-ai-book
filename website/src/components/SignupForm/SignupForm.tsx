import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';

interface FormData {
  name: string;
  email: string;
  password: string;
  confirmPassword: string;
  softwareBackground: string;
  programmingLevel: string;
  knownLanguages: string[];
  aiMlExperience: boolean;
  rosRoboticsExperience: boolean;
  hasJetson: boolean;
  hasRTXGPU: boolean;
  hasRobotHardware: boolean;
  simulationOnly: boolean;
}

const SignupForm: React.FC = () => {
  const { signup } = useAuth();
  const [formData, setFormData] = useState<FormData>({
    name: '',
    email: '',
    password: '',
    confirmPassword: '',
    softwareBackground: 'beginner',
    programmingLevel: 'beginner',
    knownLanguages: [],
    aiMlExperience: false,
    rosRoboticsExperience: false,
    hasJetson: false,
    hasRTXGPU: false,
    hasRobotHardware: false,
    simulationOnly: true,
  });
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState(false);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value, type } = e.target;
    
    if (type === 'checkbox') {
      const target = e.target as HTMLInputElement;
      setFormData({
        ...formData,
        [name]: target.checked,
      });
    } else if (name === 'knownLanguages') {
      // Handle multi-select for programming languages
      const target = e.target as HTMLSelectElement;
      const selectedOptions = Array.from(target.selectedOptions).map(option => option.value);
      setFormData({
        ...formData,
        [name]: selectedOptions,
      });
    } else {
      setFormData({
        ...formData,
        [name]: value,
      });
    }
  };

  const validateForm = (): boolean => {
    const newErrors: Record<string, string> = {};

    if (!formData.name.trim()) {
      newErrors.name = 'Name is required';
    }

    if (!formData.email.trim()) {
      newErrors.email = 'Email is required';
    } else if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(formData.email)) {
      newErrors.email = 'Invalid email format';
    }

    if (!formData.password) {
      newErrors.password = 'Password is required';
    } else if (formData.password.length < 8) {
      newErrors.password = 'Password must be at least 8 characters';
    }

    if (formData.password !== formData.confirmPassword) {
      newErrors.confirmPassword = 'Passwords do not match';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setLoading(true);
    try {
      await signup({
        name: formData.name,
        email: formData.email,
        password: formData.password,
        softwareBackground: formData.softwareBackground,
        hardwareBackground: formData.hasJetson || formData.hasRTXGPU || formData.hasRobotHardware ? 'hardware' : 'simulation_only',
        programmingLevel: formData.programmingLevel,
        knownLanguages: JSON.stringify(formData.knownLanguages),
        aiMlExperience: formData.aiMlExperience,
        rosRoboticsExperience: formData.rosRoboticsExperience,
        hasJetson: formData.hasJetson,
        hasRTXGPU: formData.hasRTXGPU,
        hasRobotHardware: formData.hasRobotHardware,
        simulationOnly: formData.simulationOnly,
      });
      
      // Reset form
      setFormData({
        name: '',
        email: '',
        password: '',
        confirmPassword: '',
        softwareBackground: 'beginner',
        programmingLevel: 'beginner',
        knownLanguages: [],
        aiMlExperience: false,
        rosRoboticsExperience: false,
        hasJetson: false,
        hasRTXGPU: false,
        hasRobotHardware: false,
        simulationOnly: true,
      });
    } catch (error: any) {
      console.error('Signup error:', error);
      setErrors({ submit: error.message || 'An error occurred during signup' });
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signup-form-container">
      <h2>Create Your Account</h2>
      <p>Join our AI & Robotics learning platform. Please share your background to personalize your experience.</p>

      <form onSubmit={handleSubmit} className="signup-form">
        {/* Personal Information */}
        <div className="form-section">
          <h3>Personal Information</h3>
          <div className="form-group">
            <label htmlFor="name">Full Name</label>
            <input
              type="text"
              id="name"
              name="name"
              value={formData.name}
              onChange={handleChange}
              required
            />
            {errors.name && <span className="error">{errors.name}</span>}
          </div>

          <div className="form-group">
            <label htmlFor="email">Email Address</label>
            <input
              type="email"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              required
            />
            {errors.email && <span className="error">{errors.email}</span>}
          </div>

          <div className="form-group">
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              name="password"
              value={formData.password}
              onChange={handleChange}
              required
            />
            {errors.password && <span className="error">{errors.password}</span>}
          </div>

          <div className="form-group">
            <label htmlFor="confirmPassword">Confirm Password</label>
            <input
              type="password"
              id="confirmPassword"
              name="confirmPassword"
              value={formData.confirmPassword}
              onChange={handleChange}
              required
            />
            {errors.confirmPassword && <span className="error">{errors.confirmPassword}</span>}
          </div>
        </div>

        {/* Software Background */}
        <div className="form-section">
          <h3>Software Background</h3>
          
          <div className="form-group">
            <label htmlFor="programmingLevel">Programming Experience Level</label>
            <select
              id="programmingLevel"
              name="programmingLevel"
              value={formData.programmingLevel}
              onChange={handleChange}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="knownLanguages">Programming Languages You Know</label>
            <select
              id="knownLanguages"
              name="knownLanguages"
              multiple
              value={formData.knownLanguages}
              onChange={handleChange}
            >
              <option value="python">Python</option>
              <option value="javascript">JavaScript</option>
              <option value="cpp">C++</option>
              <option value="java">Java</option>
              <option value="csharp">C#</option>
              <option value="rust">Rust</option>
              <option value="go">Go</option>
            </select>
            <small>Hold Ctrl (Cmd on Mac) to select multiple languages</small>
          </div>

          <div className="form-group checkbox-group">
            <label>
              <input
                type="checkbox"
                name="aiMlExperience"
                checked={formData.aiMlExperience}
                onChange={handleChange}
              />
              Do you have experience with AI/ML?
            </label>
          </div>

          <div className="form-group checkbox-group">
            <label>
              <input
                type="checkbox"
                name="rosRoboticsExperience"
                checked={formData.rosRoboticsExperience}
                onChange={handleChange}
              />
              Do you have experience with ROS/Robotics?
            </label>
          </div>
        </div>

        {/* Hardware Background */}
        <div className="form-section">
          <h3>Hardware Background</h3>
          
          <div className="form-group checkbox-group">
            <label>
              <input
                type="checkbox"
                name="hasJetson"
                checked={formData.hasJetson}
                onChange={handleChange}
              />
              Do you have a Jetson (Orin/Nano)?
            </label>
          </div>

          <div className="form-group checkbox-group">
            <label>
              <input
                type="checkbox"
                name="hasRTXGPU"
                checked={formData.hasRTXGPU}
                onChange={handleChange}
              />
              Do you have an RTX GPU?
            </label>
          </div>

          <div className="form-group checkbox-group">
            <label>
              <input
                type="checkbox"
                name="hasRobotHardware"
                checked={formData.hasRobotHardware}
                onChange={handleChange}
              />
              Do you have robot hardware?
            </label>
          </div>

          <div className="form-group checkbox-group">
            <label>
              <input
                type="checkbox"
                name="simulationOnly"
                checked={formData.simulationOnly}
                onChange={handleChange}
              />
              Are you simulation-only?
            </label>
          </div>
        </div>

        {errors.submit && <div className="error submit-error">{errors.submit}</div>}

        <button type="submit" disabled={loading} className="submit-btn">
          {loading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;