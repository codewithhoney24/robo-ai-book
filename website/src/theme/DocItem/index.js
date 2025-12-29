import React, { useState, useEffect } from 'react';
import OriginalDocItem from '@theme-original/DocItem';
import { usePersonalization } from '@site/src/contexts/PersonalizationContext';
import PersonalizeButton from '@site/src/components/PersonalizeButton';

// Import the API configuration
import { API_CONFIG } from '@site/src/config/api';

// =====================================================
// TEXT EXTRACTION
// =====================================================
function extractTextFromChildren(children) {
  if (!children) return '';
  if (typeof children === 'string' || typeof children === 'number') return String(children);
  if (Array.isArray(children)) return children.map(extractTextFromChildren).join(' ');
  if (React.isValidElement(children)) return extractTextFromChildren(children.props.children);
  if (children?.props?.children) return extractTextFromChildren(children.props.children);
  return '';
}

function extractDocumentText() {
  try {
    const article = document.querySelector('article');
    if (article) {
      const clone = article.cloneNode(true);
      const elementsToRemove = ['button', 'script', 'style', '.breadcrumbs', '.pagination-nav', '.theme-doc-footer'];
      elementsToRemove.forEach(selector => {
        clone.querySelectorAll(selector).forEach(el => el.remove());
      });
      return clone.textContent.trim();
    }
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

  const { refreshContent, personalizationData } = usePersonalization();

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
      let textContent = extractDocumentText();
      if (!textContent || textContent.length < 50) {
        textContent = extractTextFromChildren(props.content);
      }
      if (!textContent || textContent.trim().length === 0) {
        throw new Error('No text found to translate');
      }

      const response = await fetch(`${API_CONFIG.BASE_URL}/v1/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'Accept': 'application/json' },
        body: JSON.stringify({
          text: textContent,
          source_lang: 'en',
          target_lang: 'ur'
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `Server error: ${response.status}`);
      }

      const data = await response.json();
      if (!data.translated_text) throw new Error('Invalid response from server');
      setUrduText(data.translated_text);

    } catch (error) {
      setError(error.message);
      alert(`Translation failed: ${error.message}`);
    } finally {
      setIsTranslating(false);
    }
  };

  const handleReset = () => {
    setUrduText(null);
    setError(null);
  };

  const handlePersonalize = () => {
    refreshContent();
  };

  // Get user background or default values
  const softwareBackground = personalizationData?.preferences.softwareBackground || 'General';

  // =====================================================
  // URDU VIEW
  // =====================================================
  if (urduText) {
    return (
      <div style={{
        padding: '40px',
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
          <strong style={{ color: '#15807cff', fontSize: '20px', border: '1px solid #00ffff', padding: '6px', borderRadius: '5px' }}>
            🇵🇰 Urdu Translation
          </strong>
          <button onClick={handleReset} style={{ cursor: 'pointer', background: '#20423dff', border: 'none', color: 'white', padding: '8px 16px', borderRadius: '5px' }}>
            Show Original
          </button>
        </div>
        <div dir="rtl" style={{ textAlign: 'right', fontFamily: 'Noto Nastaliq Urdu, Arial', fontSize: '20px', lineHeight: '2.2', color: '#ffffff', whiteSpace: 'pre-wrap' }}>
          {urduText}
        </div>
      </div>
    );
  }

  // =====================================================
// DEFAULT VIEW
// =====================================================
return (
  <>
    <div style={{ marginBottom: '20px', display: 'flex', flexDirection: 'column', gap: '12px' }}>

      <div style={{ display: 'flex', gap: '8px', flexWrap: 'wrap', alignItems: 'center' }}>

        {/* TRANSLATE BUTTON */}
        <button
          onClick={handleTranslate}
          disabled={isTranslating}
          className="button button--primary"
          style={{
            fontSize: '14px',
            padding: '8px 16px',
            backgroundColor: '#1a1a2e',
            color: '#ffffff',
            border: '1px solid #00ffff',
            borderRadius: '6px',
            cursor: isTranslating ? 'not-allowed' : 'pointer'
          }}
        >
          {isTranslating ? 'Translating...' : '🇵🇰 Urdu'}
        </button>

        {/* PERSONALIZE BUTTON */}
        <PersonalizeButton />
      </div>

        {/* USER STATUS */}
        <div style={{ fontSize: '0.75rem', color: '#25c2a0' }}>
          ✨ Profile Active: <b>{softwareBackground}</b>
        </div>

        {error && <div style={{ color: '#ff4d4d', fontSize: '13px' }}>⚠️ {error}</div>}
      </div>

      <OriginalDocItem {...props} />
    </>
  );
}
