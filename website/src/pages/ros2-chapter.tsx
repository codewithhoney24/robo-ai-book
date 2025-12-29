import React from 'react';
import Layout from '@theme/Layout';
import ROS2FundamentalsContent from '../components/ROS2FundamentalsContent';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';

const ROS2ChapterPage: React.FC = () => {
  return (
    <Layout title="ROS 2 Fundamentals" description="Learn ROS 2 fundamentals with personalized content">
      <div style={{ padding: '20px', maxWidth: '1200px', margin: '0 auto' }}>
        <h1 style={{ textAlign: 'center', marginBottom: '30px' }}>ROS 2 Fundamentals Chapter</h1>
        <ROS2FundamentalsContent />
      </div>
    </Layout>
  );
};

export default ROS2ChapterPage;