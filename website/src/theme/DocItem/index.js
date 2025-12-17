import React, { useState, useEffect } from 'react';
import OriginalDocItem from '@theme-original/DocItem';

// =====================================================
// IMPROVED TEXT EXTRACTION
// =====================================================
function extractTextFromChildren(children) {
  if (!children) return '';
  
  // Handle string/number directly
  if (typeof children === 'string' || typeof children === 'number') {
    return String(children);
  }
  
  // Handle arrays
  if (Array.isArray(children)) {
    return children.map(extractTextFromChildren).join(' ');
  }
  
  // Handle React elements
  if (React.isValidElement(children)) {
    return extractTextFromChildren(children.props.children);
  }
  
  // Handle objects with children property
  if (children?.props?.children) {
    return extractTextFromChildren(children.props.children);
  }
  
  return '';
}

function extractDocumentText() {
  try {
    // Try multiple extraction strategies
    
    // Strategy 1: Get main article content
    const article = document.querySelector('article');
    if (article) {
      const clone = article.cloneNode(true);
      
      // Remove elements that shouldn't be translated
      const elementsToRemove = [
        'button',
        'script',
        'style',
        '.breadcrumbs',
        '.pagination-nav',
        '.theme-doc-footer'
      ];
      
      elementsToRemove.forEach(selector => {
        clone.querySelectorAll(selector).forEach(el => el.remove());
      });
      
      return clone.textContent.trim();
    }
    
    // Strategy 2: Fallback to body text
    return document.body.textContent.trim();
    
  } catch (error) {
    console.error('Text extraction failed:', error);
    return '';
  }
}

// =====================================================
// MAIN COMPONENT
// =====================================================
export default function DocItem(props) {
  const [urduText, setUrduText] = useState(null);
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState(null);

  // Clear error after 5 seconds
  useEffect(() => {
    if (error) {
      const timer = setTimeout(() => setError(null), 5000);
      return () => clearTimeout(timer);
    }
  }, [error]);

  const handleTranslate = async () => {
    setIsTranslating(true);
    setError(null);
    
    try {
      // Extract text from the document
      let textContent = extractDocumentText();
      
      // Fallback to props if DOM extraction fails
      if (!textContent || textContent.length < 50) {
        textContent = extractTextFromChildren(props.content);
      }
      
      // Validate extracted text
      if (!textContent || textContent.trim().length === 0) {
        throw new Error('No text found to translate');
      }
      
      console.log(`Extracted ${textContent.length} characters for translation`);
      console.log('First 100 chars:', textContent.slice(0, 100));
      
      // Make API request with proper error handling
      const response = await fetch('http://127.0.0.1:8000/api/v1/translate', {
        method: 'POST',
        headers: { 
          'Content-Type': 'application/json',
          'Accept': 'application/json'
        },
        body: JSON.stringify({ 
          text: textContent,
          source_lang: 'en',
          target_lang: 'ur'
        }),
      });

      // Handle non-OK responses
      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(
          errorData.detail || 
          `Server error: ${response.status} ${response.statusText}`
        );
      }

      const data = await response.json();
      
      if (!data.translated_text) {
        throw new Error('Invalid response from server');
      }
      
      setUrduText(data.translated_text);
      console.log('Translation successful');

    } catch (error) {
      console.error('Translation error:', error);
      setError(error.message);
      
      // Show user-friendly error
      const errorMessage = error.message.includes('Failed to fetch')
        ? 'Cannot connect to translation server. Make sure it is running on port 8000.'
        : error.message;
      
      alert(`Translation failed: ${errorMessage}`);
      
    } finally {
      setIsTranslating(false);
    }
  };

  const handleReset = () => {
    setUrduText(null);
    setError(null);
  };

  // =====================================================
  // TRANSLATION VIEW
  // =====================================================
  if (urduText) {
    return (
      <div style={{ 
        padding: '20px', 
        backgroundColor: '#232827ff', 
        borderRadius: '8px', 
        marginBottom: '20px', 
        border: '2px solid #25c2a0',
        boxShadow: '0 2px 8px rgba(37, 194, 160, 0.1)'
      }}>
        <div style={{ 
          marginBottom: '15px', 
          display: 'flex', 
          justifyContent: 'space-between',
          alignItems: 'center',
          borderBottom: '1px solid #88ead4ff',
          paddingBottom: '10px'
        }}>
          <strong style={{ color: '#15807cff', fontSize: '20px', border: '1px solid #00ffff',padding: '6px', borderRadius: '5px' }}>
            🇵🇰 Urdu Translation
          </strong>
          <button 
            onClick={handleReset} 
            style={{ 
              cursor: 'pointer', 
              background: '#20423dff', 
              border: 'none', 
              color: 'white',
              padding: '8px 16px',
              borderRadius: '5px',
              fontWeight: '600',
              transition: 'background 0.3s'
            }}
            onMouseOver={(e) => e.target.style.background = '#163f37ff'}
            onMouseOut={(e) => e.target.style.background = '#16312bff'}
          >
            Show Original
          </button>
        </div>
        <div 
          dir="rtl" 
          style={{ 
            textAlign: 'right', 
            fontFamily: 'Noto Nastaliq Urdu, Jameel Noori Nastaleeq, Arial', 
            fontSize: '20px', 
            lineHeight: '2.2',
            color: '#1e293b',
            whiteSpace: 'pre-wrap'
          }}
        >
          {urduText}
        </div>
      </div>
    );
  }

  // =====================================================
  // DEFAULT VIEW WITH TRANSLATE BUTTON
  // =====================================================
  return (
    <>
      <div style={{ marginBottom: '15px' }}>
        {error && (
          <div style={{
            padding: '12px',
            backgroundColor: 'rgba(113, 95, 95, 1)',
            border: '1px solid #fcc',
            borderRadius: '5px',
            color: '#c00',
            marginBottom: '10px'
          }}>
            ⚠️ {error}
          </div>
        )}
        
       <button 
  onClick={handleTranslate} 
  disabled={isTranslating}
  className="button button--primary button--lg"
  style={{
    width: '24%',
    fontSize: '16px',
    padding: '12px',
    cursor: isTranslating ? 'not-allowed' : 'pointer',
    opacity: isTranslating ? 0.6 : 1,
    
    // 👇 Yahan Changes Kiye Hain (Dark Theme)
    backgroundColor: '#1a1a2e',    // Deep Dark Blue
    color: '#ffffff',              // White Text
    border: '1px solid #00ffff',   // Neon Cyan Border (Chatbot se match)
    boxShadow: '0 0 10px rgba(0, 255, 255, 0.2)' // Halka sa Glow effect
  }}
>
  {isTranslating ? (
    <>
      <span className="spinner" style={{ 
        display: 'inline-block',
        width: '14px',
        height: '14px',
        border: '2px solid #fff',
        borderTopColor: 'transparent',
        borderRadius: '50%',
        animation: 'spin 0.8s linear infinite',
        marginRight: '8px'
      }} />
      Translating to Urdu...
    </>
  ) : (
    '🇵🇰 Translate Page to Urdu'
  )}
</button>
      </div>
      
      <OriginalDocItem {...props} />
      
      <style>{`
        @keyframes spin {
          to { transform: rotate(360deg); }
        }
      `}</style>
    </>
  );
}