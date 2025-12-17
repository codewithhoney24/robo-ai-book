// File: src/theme/Layout/index.tsx

import React, { type ReactNode } from 'react'; 
import Layout from '@theme-original/Layout'; 
import ChatbotWidget from '@site/src/components/ChatbotWidget/ChatbotWidget'; 

type LayoutProps = {
  children: ReactNode;
  title?: string;
  description?: string;
};

// Layout component ko TypeScript ke saath export kiya
export default function LayoutWrapper(props: LayoutProps): ReactNode {
  return (
    // Yahan par humne Global Margin/Padding ko hatane ke liye ek DIV lagaya hai
    // Yeh DIV, Layout aur Chatbot dono ko wrap karega.
    <div style={{ 
      margin: 0, 
      padding: 0, 
      minHeight: '100vh', // Zaroori hai ke poori height le
      display: 'flex', // Zaroori hai ke content ko vertically align kare
      flexDirection: 'column' 
    }}>
      {/* Docusaurus Layout ko render kiya. */}
      <Layout {...props}>
        {/* 'props.children' mein aapki website ka content hai */}
        {props.children} 
      </Layout>
      
      {/* 🤖 Chatbot Widget: Isko Layout ke bahar render kiya gaya hai */}
      <ChatbotWidget /> 
    </div>
  );
}

