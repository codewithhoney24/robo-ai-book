import React from 'react';
import Layout from '@theme/Layout';
import ChatKitPersonalization from '../components/ChatKitPersonalization';

const ChatKitPersonalizationPage: React.FC = () => {
  return (
    <Layout title="ChatKit Personalization" description="Interactive chat interface for personalization settings">
      <div style={{ padding: '2rem' }}>
        <div style={{ maxWidth: '800px', margin: '0 auto' }}>
          <h1>ChatKit Personalization Interface</h1>
          <p>
            This interactive chat interface allows you to manage your personalization settings.
            Type commands to view and update your learning preferences.
          </p>
          <div style={{ marginTop: '2rem' }}>
            <ChatKitPersonalization />
          </div>
          <div style={{ marginTop: '2rem' }}>
            <h2>How to Use</h2>
            <ul>
              <li>Type <code>help</code> to see all available commands</li>
              <li>Type <code>show preferences</code> to view current settings</li>
              <li>Type <code>show features</code> to see all personalization features</li>
              <li>Use <code>set difficulty [level]</code> to update difficulty</li>
              <li>Use <code>set style [type]</code> to update learning style</li>
              <li>Use <code>set topics [topic1,topic2]</code> to set preferred topics</li>
            </ul>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default ChatKitPersonalizationPage;