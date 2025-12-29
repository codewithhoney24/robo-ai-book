// ChatbotWidget.tsx
import React, { useState, useRef, useEffect } from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';
import { API_CONFIG } from '../../config/api';
import styles from './ChatbotWidget.module.css';

// 1. Message ki type define ki (TypeScript ke liye) - UPDATED
interface Message {
  role: 'user' | 'bot';
  content: string;
  urdu?: string; // ✅ NEW: Urdu translation store karne ke liye
  id?: number; // ✅ NEW: Unique ID
}

const ChatbotWidget: React.FC = () => {
  const { personalizationData, updatePersonalization } = usePersonalization();
  const [isOpen, setIsOpen] = useState(false);

  // 2. Nayi States Add ki hain (Logic ke liye)
  const [messages, setMessages] = useState<Message[]>([]); // Chat history
  const [input, setInput] = useState(''); // Input box ka text
  const [loading, setLoading] = useState(false); // Loading animation ke liye
  const messagesEndRef = useRef<null | HTMLDivElement>(null); // Auto-scroll ke liye

  // ✅ NEW: Urdu Mode States
  const [urduMode, setUrduMode] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);

  // ✅ NEW: Text Selection States
  const [selectedText, setSelectedText] = useState<string | null>(null);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // ✅ NEW: Function to handle text selection
  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '') {
        const selectedText = selection.toString().trim();
        if (selectedText.length > 0 && selectedText.length < 500) { // Limit selection length
          setSelectedText(selectedText);
        }
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
    };
  }, []);

  // ✅ NEW: Function to send selected text to chat
  const sendSelectedTextToChat = () => {
    if (selectedText) {
      setInput(`Translate and explain this text: "${selectedText}"`);
      setSelectedText(null); // Clear the selected text after adding to input
    }
  };

  // Auto-scroll logic: Jab naya msg aaye to neeche scroll karo
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // ✅ NEW: Translation Function
  const translateMessage = async (text: string): Promise<string> => {
    try {
      const response = await fetch(`${API_CONFIG.BASE_URL}/v1/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          text: text,
          source_lang: 'en',
          target_lang: 'ur',
          use_ai: false
        }),
      });

      if (!response.ok) throw new Error('Translation failed');

      const data = await response.json();
      return data.translated_text;
    } catch (error) {
      console.error('Translation error:', error);
      return text; // Return original if translation fails
    }
  };

  // ✅ NEW: Toggle Urdu Mode
  const toggleUrduMode = async () => {
    if (!urduMode && messages.length > 0) {
      setIsTranslating(true);

      try {
        // Translate all messages
        const translatedMessages = await Promise.all(
          messages.map(async (msg) => {
            if (!msg.urdu) {
              const urduText = await translateMessage(msg.content);
              return { ...msg, urdu: urduText };
            }
            return msg;
          })
        );

        setMessages(translatedMessages);
      } catch (error) {
        console.error('Batch translation error:', error);
      } finally {
        setIsTranslating(false);
      }
    }

    setUrduMode(!urduMode);
  };

  // 3. Backend se baat karne wala Main Function - UPDATED
  const handleSendMessage = async () => {
    if (!input.trim()) return;

    // User ka message add kiya - UPDATED with ID
    const userMessage: Message = {
      role: 'user',
      content: input,
      id: Date.now(),
      urdu: undefined
    };
    setMessages((prev) => [...prev, userMessage]);
    setInput(''); // Input khali kiya
    setLoading(true);

    try {
      // Backend Call (Jo humne pehle test kiya tha)
      const response = await fetch(`${API_CONFIG.BASE_URL}/v1/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: userMessage.content,
          chat_history: [], // Filhal simple rakha hai
          personalization: personalizationData // Include personalization data
        }),
      });

      const data = await response.json();

      // Bot ka jawab add kiya - UPDATED with ID
      const botMessage: Message = {
        role: 'bot',
        content: data.content,
        id: Date.now() + 1,
        urdu: undefined
      };
      setMessages((prev) => [...prev, botMessage]);

      // ✅ NEW: If Urdu mode is ON, translate immediately
      if (urduMode) {
        const urduText = await translateMessage(botMessage.content);
        setMessages((prev) =>
          prev.map(msg =>
            msg.id === botMessage.id
              ? { ...msg, urdu: urduText }
              : msg
          )
        );
      }

    } catch (error) {
      console.error("Error:", error);
      const errorMessage: Message = {
        role: 'bot',
        content: "⚠️ Server Error: Backend se connect nahi ho raha.",
        id: Date.now() + 1
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      {/* Robot Icon Button */}
      <div
        className={styles.robotIcon}
        onClick={toggleChat}
        aria-label="Open Chatbot"
        role="button"
        tabIndex={0}
      >
        {/* Agar aapke CSS mein icon image nahi hai to yahan emoji daal sakte hain */}
        🤖
      </div>

      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header section - UPDATED with Urdu Toggle */}
          <div className={styles.chatHeader}>
            <div className={styles.headerInfo}>
              <h2>ROBOTIC CHAT {urduMode && '| اردو موڈ'}</h2>
              <p>Smart Friend for Robotics</p>
            </div>
            <div className={styles.headerControls}>
              {/* ✅ NEW: Urdu Toggle Button */}
              <button
                className={styles.controlButton}
                onClick={toggleUrduMode}
                disabled={isTranslating}
                title={urduMode ? "Switch to English" : "Switch to Urdu"}
                style={{
                  cursor: isTranslating ? 'not-allowed' : 'pointer',
                  opacity: isTranslating ? 0.6 : 1
                }}
              >
                {isTranslating ? '⏳' : urduMode ? '🇵🇰' : '🌐'}
              </button>
              <button className={styles.controlButton} onClick={() => setMessages([])}>⟳</button>
              <button className={styles.closeButton} onClick={toggleChat}>X</button>
            </div>
          </div>

          {/* 4. Body section (Yahan bari changing ki hai) */}
          <div className={styles.chatBody}>

            {/* Logic: Agar messages hain to Chat dikhao, warna Welcome Screen dikhao */}
            {messages.length === 0 ? (
              // --- WELCOME SCREEN (Purana Code) ---
              <div className={styles.welcomeScreen}>
                 <div className={styles.roboEye} />
                 <h1 className={styles.welcomeText}>Welcome to HUMANOID!</h1>
                 <p className={styles.guideText}>I provide Expert Guidance for Physical AI & Humanoid Robotics.</p>
                 <div className={styles.tipBox}>
                   <span className={styles.tipIcon}>💡</span>
                 <p>Tip: Ask about "Humanoid Sensors" or "AI Agents", or try the "Urdu Translation" feature.</p>
                 </div>
              </div>
            ) : (
              // --- CHAT MESSAGES LIST (Naya Code) - UPDATED with Urdu Support
              <div className={styles.messagesList}>
                {messages.map((msg, index) => (
                  <div key={msg.id || index} className={`${styles.messageRow} ${msg.role === 'user' ? styles.userRow : styles.botRow}`}>
                    <div className={msg.role === 'user' ? styles.userBubble : styles.botBubble}>
                      {/* ✅ NEW: Urdu Text Display Logic */}
                      {urduMode && msg.urdu ? (
                        <div
                          dir="rtl"
                          style={{
                            fontFamily: 'Noto Nastaliq Urdu, Jameel Noori Nastaleeq, Arial',
                            fontSize: '16px',
                            lineHeight: '1.8'

                          }}


                        >
                          {msg.urdu}
                        </div>
                      ) : (
                        <div>{msg.content}</div>
                      )}


                      {/* ✅ NEW: Translation Loading Indicator */}
                      {urduMode && !msg.urdu && msg.content && (
                        <div style={{
                          fontSize: '11px',
                          color: msg.role === 'user' ? 'rgba(240, 234, 234, 0.7)' : '#c7bdbdff',
                          marginTop: '5px',
                          fontStyle: 'italic'
                        }}>
                          Translating...
                        </div>
                      )}
                    </div>
                  </div>
                ))}
                {loading && <div className={styles.loadingBubble}>Thinking...</div>}
                <div ref={messagesEndRef} />
              </div>
            )}
          </div>



          {/* 5. Footer section (Input Logic connect ki) - UPDATED with Urdu Placeholder */}
          <div className={styles.chatFooter}>
            <input
              type="text"
              placeholder={urduMode ? "سوال پوچھیں..." : "Ask about the course..."}
              className={styles.footerInput}
              value={input} // State se connect kiya
              onChange={(e) => setInput(e.target.value)} // Typing handle ki
              onKeyDown={(e) => e.key === 'Enter' && handleSendMessage()} // Enter key support
              disabled={loading}
            />
            {selectedText && (
              <button
                className={styles.footerButton}
                onClick={sendSelectedTextToChat}
                title="Send selected text to chat"
                style={{ backgroundColor: '#28a745', marginRight: '5px' }}
              >
                📝
              </button>
            )}
            <button
              className={styles.footerButton}
              onClick={handleSendMessage}
              disabled={loading || !input.trim()}
            >
              {loading ? '⏳' : '✓'}
            </button>
          </div>
        </div>
      )}

    </div>
  );
};

export default ChatbotWidget;