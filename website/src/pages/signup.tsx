import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/SignupForm/SignupForm';

export default function SignupPage(): JSX.Element {
  return (
    <Layout title="Signup" description="Create your account for the AI & Robotics learning platform">
      <main>
        <div style={{ padding: '2rem', maxWidth: '800px', margin: '0 auto' }}>
          <SignupForm />
        </div>
      </main>
    </Layout>
  );
}