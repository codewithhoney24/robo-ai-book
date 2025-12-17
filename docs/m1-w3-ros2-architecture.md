<!-- ---
id: m1-w3-ros2-architecture
title: "Module 1: ROS 2 Architecture and Core Concepts"
sidebar_label: "Wk 3: ROS 2 Architecture"
---

import React, { useState } from 'react';
import { TranslateButton } from '@site/src/components/TranslateButton';

// Original Content ko yahan variable mein rakhen
const ORIGINAL_CONTENT = `
## ROS 2 Nodes, Topics, and Services

ROS 2 (Robot Operating System 2) provides a standardized middleware for robotic control.
The fundamental unit of computation in ROS 2 is the Node. Nodes communicate with each other
using Topics (asynchronous communication) and Services (synchronous communication).
This architecture allows for robust, modular, and distributed control of the Humanoid Robot's
nervous system, which is crucial for bipedal locomotion.

### Bridging Python Agents to ROS controllers

We use the rclpy client library in Python to interface our high-level AI agents (LLMs/VLA)
with the low-level ROS controllers that manage the physical actuators of the robot.

\`--- Hardware Context: This requires Ubuntu 22.04 LTS and a high-performance RTX GPU. ---\`
`;

// Component jo button aur content dono dikhaega
function TranslatedContentWrapper() {
  const [currentContent, setCurrentContent] = useState(ORIGINAL_CONTENT);
  const [isTranslated, setIsTranslated] = useState(false);

  // Translation handler function
  const handleTranslation = (translatedText) => {
    setCurrentContent(translatedText);
    setIsTranslated(true);
  };
  
  // Original content par wapas aane ke liye
  const resetContent = () => {
    setCurrentContent(ORIGINAL_CONTENT);
    setIsTranslated(false);
  };

  const contentToDisplay = currentContent;

  return (
    <div>
      {/* Buttons */}
      <div style={{ display: 'flex', alignItems: 'center', marginBottom: '20px' }}>
        <h2 style={{ marginRight: 'auto' }}>Module 1: ROS 2 Fundamentals</h2>
        <button 
            onClick={resetContent} 
            className="button button--secondary" 
            disabled={!isTranslated} 
            style={{ marginRight: '10px' }}
        >
            Show Original (English)
        </button>
        <TranslateButton content={ORIGINAL_CONTENT} onTranslation={handleTranslation} />
      </div>

      {/* Content Display */}
      {/* ContentDisplay ko simple div mein rakhen, aur dangerouslySetInnerHTML ko use karein */}
      <div 
        style={{ whiteSpace: 'pre-wrap' }}
        dangerouslySetInnerHTML={{ __html: contentToDisplay }} 
      />
      
    </div>
  );
}

{/* Aakhri mein is component ko page par display karein */}
<TranslatedContentWrapper /> -->