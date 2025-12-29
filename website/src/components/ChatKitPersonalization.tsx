import React, { useState, useRef, useEffect } from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';

interface ChatMessage {
  id: number;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
}

const ChatKitPersonalization: React.FC = () => {
  const { personalizationData, updatePersonalization } = usePersonalization();
  const { preferences } = personalizationData;
  const [messages, setMessages] = useState<ChatMessage[]>([
    {
      id: 1,
      text: "Hello! I'm your personalization assistant. I can help you customize your learning experience. Type 'help' to see options.",
      sender: 'bot',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSend = () => {
    if (!inputValue.trim()) return;

    // Add user message
    const userMessage: ChatMessage = {
      id: messages.length + 1,
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');

    // Process the command and generate bot response
    setTimeout(() => {
      processCommand(inputValue.toLowerCase());
    }, 500);
  };

  const processCommand = (command: string) => {
    let response = '';
    
    if (command.includes('help')) {
      response = `Available commands:
- "show preferences" - Display current settings
- "set difficulty [beginner/intermediate/advanced]" - Update difficulty level
- "set style [visual/textual/handsOn]" - Update learning style
- "set topics [topic1,topic2]" - Set preferred topics
- "show features" - List all personalization features`;
    } 
    else if (command.includes('show preferences')) {
      response = `Current preferences:
- Learning Style: ${preferences.learningStyle}
- Difficulty: ${preferences.difficulty}
- Preferred Topics: ${preferences.preferredTopics.join(', ') || 'None'}
- Learning Goals: ${preferences.learningGoals}
- Content Access: ${preferences.contentAccess}`;
    }
    else if (command.includes('show features')) {
      response = `Personalization Features Available:
- Adaptive Difficulty: ${preferences.adaptiveDifficulty ? 'ON' : 'OFF'}
- Smart Adaptation: ${preferences.smartAdaptation ? 'ON' : 'OFF'}
- Content Recommendation: ${preferences.contentRecommendation ? 'ON' : 'OFF'}
- Progressive Disclosure: ${preferences.progressiveDisclosure ? 'ON' : 'OFF'}
- Content Adjustment: ${preferences.contentAdjustment ? 'ON' : 'OFF'}
- Targeted Help: ${preferences.targetedHelp ? 'ON' : 'OFF'}`;
    }
    else if (command.includes('set difficulty')) {
      const difficulty = command.split(' ')[2] as 'beginner' | 'intermediate' | 'advanced';
      if (['beginner', 'intermediate', 'advanced'].includes(difficulty)) {
        updatePersonalization({
          preferences: {
            ...preferences,
            difficulty: difficulty
          }
        });
        response = `Difficulty level updated to ${difficulty}.`;
      } else {
        response = 'Please specify a valid difficulty: beginner, intermediate, or advanced.';
      }
    }
    else if (command.includes('set style')) {
      const style = command.split(' ')[2] as 'visual' | 'textual' | 'handsOn';
      if (['visual', 'textual', 'handsOn'].includes(style)) {
        updatePersonalization({
          preferences: {
            ...preferences,
            learningStyle: style
          }
        });
        response = `Learning style updated to ${style}.`;
      } else {
        response = 'Please specify a valid style: visual, textual, or handsOn.';
      }
    }
    else if (command.includes('set topics')) {
      const topicsStr = command.replace('set topics', '').trim();
      const topics = topicsStr.split(',').map(t => t.trim()).filter(t => t);
      updatePersonalization({
        preferences: {
          ...preferences,
          preferredTopics: topics
        }
      });
      response = `Preferred topics updated to: ${topics.join(', ')}.`;
    }
    else {
      response = "I didn't understand that command. Type 'help' to see available options.";
    }

    const botMessage: ChatMessage = {
      id: messages.length + 2,
      text: response,
      sender: 'bot',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, botMessage]);
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <div style={{ 
      display: 'flex', 
      flexDirection: 'column', 
      height: '600px', 
      border: '1px solid #ccc', 
      borderRadius: '8px',
      overflow: 'hidden'
    }}>
      <div style={{ 
        backgroundColor: '#f0f0f0', 
        padding: '15px', 
        borderBottom: '1px solid #ccc',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center'
      }}>
        <h3>Personalization Assistant</h3>
        <div style={{ fontSize: '0.9em', color: '#666' }}>
          Current: {preferences.learningStyle} | {preferences.difficulty}
        </div>
      </div>
      
      <div style={{ 
        flex: 1, 
        overflowY: 'auto', 
        padding: '15px', 
        backgroundColor: '#fafafa' 
      }}>
        {messages.map((message) => (
          <div 
            key={message.id} 
            style={{ 
              marginBottom: '15px', 
              textAlign: message.sender === 'user' ? 'right' : 'left' 
            }}
          >
            <div
              style={{
                display: 'inline-block',
                padding: '10px 15px',
                borderRadius: '18px',
                backgroundColor: message.sender === 'user' ? '#007bff' : '#e5e5ea',
                color: message.sender === 'user' ? 'white' : 'black',
                maxWidth: '70%'
              }}
            >
              {message.text}
            </div>
            <div style={{ fontSize: '0.7em', color: '#999', marginTop: '4px' }}>
              {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
        ))}
        <div ref={messagesEndRef} />
      </div>
      
      <div style={{ padding: '10px', backgroundColor: 'white', borderTop: '1px solid #ccc' }}>
        <div style={{ display: 'flex' }}>
          <textarea
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Type a command (e.g., 'help', 'show preferences', 'set difficulty intermediate')"
            style={{ 
              flex: 1, 
              padding: '10px', 
              border: '1px solid #ccc', 
              borderRadius: '18px',
              resize: 'none',
              height: '60px'
            }}
          />
          <button
            onClick={handleSend}
            style={{
              marginLeft: '10px',
              padding: '10px 20px',
              backgroundColor: '#007bff',
              color: 'white',
              border: 'none',
              borderRadius: '5px',
              cursor: 'pointer'
            }}
          >
            Send
          </button>
        </div>
        <div style={{ fontSize: '0.8em', color: '#666', marginTop: '5px' }}>
          Commands: help, show preferences, show features, set difficulty [level], set style [type], set topics [list]
        </div>
      </div>
    </div>
  );
};

export default ChatKitPersonalization;