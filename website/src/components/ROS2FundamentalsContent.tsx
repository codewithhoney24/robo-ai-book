import React from 'react';
import ChapterPersonalization from '../components/ChapterPersonalization';
import AdaptiveContent from '../components/AdaptiveContent';
import { usePersonalization } from '../contexts/PersonalizationContext';

const ROS2FundamentalsContent: React.FC = () => {
  const { personalizationData } = usePersonalization();
  const { preferences } = personalizationData;

  // Sample ROS 2 content that adapts based on user preferences
  return (
    <div>
      <ChapterPersonalization 
        chapterTitle="ROS 2 Fundamentals" 
        topic="ros2-basics"
      >
        <p>
          Welcome to ROS 2 Fundamentals! This chapter will adapt to your learning preferences.
          You've selected {preferences.learningStyle} learning style and {preferences.difficulty} difficulty.
        </p>

        <AdaptiveContent contentType="visual" topic="ros2-basics" difficulty="beginner">
          <div style={{ 
            border: '1px solid #00bcd4', 
            borderRadius: '8px', 
            padding: '15px', 
            margin: '10px 0',
            backgroundColor: '#e0f7fa'
          }}>
            <h3>ROS 2 Architecture Diagram</h3>
            <div style={{ 
              textAlign: 'center', 
              padding: '15px', 
              backgroundColor: 'white', 
              borderRadius: '4px',
              margin: '10px 0'
            }}>
              <svg width="400" height="200" viewBox="0 0 400 200">
                <rect x="50" y="50" width="100" height="50" fill="#2196F3" stroke="#1976D2" strokeWidth="2" rx="5"/>
                <text x="100" y="80" textAnchor="middle" fill="white" fontWeight="bold" fontSize="14">Node A</text>
                
                <rect x="250" y="50" width="100" height="50" fill="#2196F3" stroke="#1976D2" strokeWidth="2" rx="5"/>
                <text x="300" y="80" textAnchor="middle" fill="white" fontWeight="bold" fontSize="14">Node B</text>
                
                <path d="M 150 75 L 250 75" stroke="#4CAF50" strokeWidth="2" fill="none" markerEnd="url(#arrowhead)"/>
                <text x="200" y="65" textAnchor="middle" fill="#4CAF50" fontWeight="bold" fontSize="12">Topic</text>
                
                <defs>
                  <marker id="arrowhead" markerWidth="10" markerHeight="7" 
                    refX="9" refY="3.5" orient="auto">
                    <polygon points="0 0, 10 3.5, 0 7" fill="#4CAF50"/>
                  </marker>
                </defs>
              </svg>
              <p style={{ marginTop: '10px' }}>This diagram shows two ROS 2 nodes communicating via a topic.</p>
            </div>
          </div>
        </AdaptiveContent>

        <AdaptiveContent contentType="textual" topic="ros2-basics" difficulty="beginner">
          <div style={{ 
            border: '1px solid #4CAF50', 
            borderRadius: '8px', 
            padding: '15px', 
            margin: '10px 0',
            backgroundColor: '#e8f5e9'
          }}>
            <h3>ROS 2 Publisher-Subscriber Pattern</h3>
            <p>
              In ROS 2, nodes communicate with each other using a publish-subscribe pattern. 
              A publisher node sends messages to a topic, and subscriber nodes receive those messages.
              This decouples the nodes, allowing for flexible and scalable system architectures.
            </p>
            <p>
              The communication is facilitated by the ROS 2 middleware, which handles message 
              serialization, network transport, and quality of service settings.
            </p>
          </div>
        </AdaptiveContent>

        <AdaptiveContent contentType="handsOn" topic="ros2-basics" difficulty="beginner">
          <div style={{ 
            border: '1px solid #FF9800', 
            borderRadius: '8px', 
            padding: '15px', 
            margin: '10px 0',
            backgroundColor: '#fff3e0'
          }}>
            <h3>Try It Yourself</h3>
            <p>
              Create a simple ROS 2 publisher and subscriber:
            </p>
            <ol>
              <li>Open a terminal and source ROS 2: <code>source /opt/ros/humble/setup.bash</code></li>
              <li>Create a new workspace: <code>mkdir -p ~/ros2_ws/src &amp;&amp; cd ~/ros2_ws</code></li>
              <li>Create a publisher node that sends "Hello ROS 2" messages</li>
              <li>Create a subscriber node that receives and prints the messages</li>
              <li>Run both nodes and observe the communication</li>
            </ol>
            <button style={{
              backgroundColor: '#FF9800',
              color: 'white',
              padding: '10px 15px',
              border: 'none',
              borderRadius: '5px',
              cursor: 'pointer',
              marginTop: '10px'
            }}>
              Launch ROS 2 Environment
            </button>
          </div>
        </AdaptiveContent>

        <AdaptiveContent contentType="visual" topic="ros2-basics" difficulty="intermediate">
          <div style={{ 
            border: '1px solid #9C27B0', 
            borderRadius: '8px', 
            padding: '15px', 
            margin: '10px 0',
            backgroundColor: '#f3e5f5'
          }}>
            <h3>ROS 2 QoS Profiles</h3>
            <p>Quality of Service (QoS) profiles allow fine-tuning communication behavior:</p>
            <ul>
              <li><strong>Reliability:</strong> Best effort vs Reliable</li>
              <li><strong>Durability:</strong> Volatile vs Transient local</li>
              <li><strong>History:</strong> Keep last N messages vs Keep all</li>
            </ul>
            <div style={{ 
              textAlign: 'center', 
              padding: '15px', 
              backgroundColor: 'white', 
              borderRadius: '4px',
              margin: '10px 0'
            }}>
              <svg width="400" height="250" viewBox="0 0 400 250">
                <rect x="50" y="50" width="300" height="150" fill="#f5f5f5" stroke="#9C27B0" strokeWidth="2" rx="5"/>
                <text x="200" y="80" textAnchor="middle" fill="#9C27B0" fontWeight="bold" fontSize="16">QoS Configuration</text>
                
                <rect x="70" y="100" width="120" height="30" fill="#E91E63" stroke="#C2185B" strokeWidth="1" rx="3"/>
                <text x="130" y="120" textAnchor="middle" fill="white" fontSize="12">Reliability</text>
                
                <rect x="210" y="100" width="120" height="30" fill="#E91E63" stroke="#C2185B" strokeWidth="1" rx="3"/>
                <text x="270" y="120" textAnchor="middle" fill="white" fontSize="12">Durability</text>
                
                <rect x="70" y="140" width="120" height="30" fill="#E91E63" stroke="#C2185B" strokeWidth="1" rx="3"/>
                <text x="130" y="160" textAnchor="middle" fill="white" fontSize="12">History</text>
                
                <rect x="210" y="140" width="120" height="30" fill="#E91E63" stroke="#C2185B" strokeWidth="1" rx="3"/>
                <text x="270" y="160" textAnchor="middle" fill="white" fontSize="12">Deadline</text>
              </svg>
            </div>
          </div>
        </AdaptiveContent>
      </ChapterPersonalization>
    </div>
  );
};

export default ROS2FundamentalsContent;