import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';

const ProfilePage: React.FC = () => {
  const { user, loading } = useAuth();
  const [isEditing, setIsEditing] = useState(false);
  const [formData, setFormData] = useState({
    name: '',
    softwareBackground: 'beginner' as 'beginner' | 'python_intermediate' | 'ros2_developer' | 'ai_robotics_expert',
    hardwareBackground: 'no_gpu' as 'no_gpu' | 'rtx_laptop' | 'rtx_workstation' | 'jetson_kit' | 'cloud',
  });

  useEffect(() => {
    if (user) {
      setFormData({
        name: user.name,
        softwareBackground: user.softwareBackground as any,
        hardwareBackground: user.hardwareBackground as any,
      });
    }
  }, [user]);

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    console.log('Profile update would happen here');
    setIsEditing(false);
  };

  if (loading) {
    return <div>Loading...</div>;
  }

  if (!user) {
    return (
      <div style={{ padding: '2rem', textAlign: 'center' }}>
        <h1>Please sign in to view your profile</h1>
      </div>
    );
  }

  return (
    <div style={{ maxWidth: '600px', margin: '3rem auto', padding: '0 1rem' }}>
      <h1>User Profile</h1>

      {isEditing ? (
        <form onSubmit={handleSubmit}>
          <div style={{ marginBottom: '1rem' }}>
            <label htmlFor="name" style={{ display: 'block', marginBottom: '0.25rem' }}>Name</label>
            <input
              type="text"
              id="name"
              name="name"
              value={formData.name}
              onChange={handleInputChange}
              required
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #065055ff',
                borderRadius: '4px',
              }}
            />
          </div>

          <div style={{ marginBottom: '1rem' }}>
            <label htmlFor="softwareBackground" style={{ display: 'block', marginBottom: '0.25rem' }}>
              Software Background
            </label>
            <select
              id="softwareBackground"
              name="softwareBackground"
              value={formData.softwareBackground}
              onChange={handleInputChange}
              required
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #065055ff',
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
            <label htmlFor="hardwareBackground" style={{ display: 'block', marginBottom: '0.25rem' }}>
              Hardware Availability
            </label>
            <select
              id="hardwareBackground"
              name="hardwareBackground"
              value={formData.hardwareBackground}
              onChange={handleInputChange}
              required
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #065055ff',
                borderRadius: '4px',
              }}
            >
              <option value="no_gpu">Standard laptop</option>
              <option value="rtx_laptop">RTX 2060–4060</option>
              <option value="rtx_workstation">RTX 3090/4090</option>
              <option value="jetson_kit">Jetson Orin</option>
              <option value="cloud">AWS/Azure</option>
            </select>
          </div>

          <div style={{ display: 'flex', gap: '1rem' }}>
            <button
              type="submit"
              style={{
                padding: '0.5rem 1rem',
                backgroundColor: '#00bab7ff',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
              }}
            >
              Save Changes
            </button>
            <button
              type="button"
              onClick={() => {
                setIsEditing(false);
                // Reset form to original values
                setFormData({
                  name: user.name,
                  softwareBackground: user.softwareBackground as any,
                  hardwareBackground: user.hardwareBackground as any,
                });
              }}
              style={{
                padding: '0.5rem 1rem',
                backgroundColor: '#6c757d',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
              }}
            >
              Cancel
            </button>
          </div>
        </form>
      ) : (
        <div>
          <div style={{ marginBottom: '1rem' }}>
            <strong>Name:</strong> {user.name}
          </div>
          <div style={{ marginBottom: '1rem' }}>
            <strong>Email:</strong> {user.email}
          </div>
          <div style={{ marginBottom: '1rem' }}>
            <strong>Software Background:</strong> {user.softwareBackground}
          </div>
          <div style={{ marginBottom: '1rem' }}>
            <strong>Hardware Background:</strong> {user.hardwareBackground}
          </div>
          <div style={{ marginBottom: '1rem' }}>
            <strong>Member since:</strong> {new Date(user.createdAt).toLocaleDateString()}
          </div>

          <button
            onClick={() => setIsEditing(true)}
            style={{
              padding: '0.5rem 1rem',
              backgroundColor: '#00bab7ff',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer',
            }}
          >
            Edit Profile
          </button>
        </div>
      )}
    </div>
  );
};

export default ProfilePage;