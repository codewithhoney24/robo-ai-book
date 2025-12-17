import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '../components/SigninForm/SigninForm';

export default function SigninPage(): JSX.Element {
  return (
    <Layout title="Signin" description="Sign in to your account for the AI & Robotics learning platform">
      <main>
        <div style={{ padding: '2rem', maxWidth: '800px', margin: '0 auto' }}>
          <SigninForm />
        </div>
      </main>
    </Layout>
  );
}