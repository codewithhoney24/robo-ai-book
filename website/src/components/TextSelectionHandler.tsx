import React, { useEffect, useState } from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';
import { API_CONFIG } from '../config/api';

interface TextSelectionHandlerProps {
  children: React.ReactNode;
}

const TextSelectionHandler: React.FC<TextSelectionHandlerProps> = ({ children }) => {
  const { personalizationData } = usePersonalization();
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [showTranslation, setShowTranslation] = useState(false);
  const [translationResult, setTranslationResult] = useState<{ urdu: string; english: string; explanation: string } | null>(null);
  const [loading, setLoading] = useState(false);

  // Function to handle text selection
  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '') {
        const selectedText = selection.toString().trim();
        if (selectedText.length > 0 && selectedText.length < 500) { // Limit selection length
          setSelectedText(selectedText);
          setShowTranslation(true);
        }
      } else {
        setShowTranslation(false);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
    };
  }, []);

  // Function to translate and explain selected text
  const translateAndExplain = async () => {
    if (!selectedText) return;

    setLoading(true);
    try {
      // Call the backend API to translate and explain the selected text
      const response = await fetch(`${API_CONFIG.BASE_URL}/v1/translate-and-explain`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          text: selectedText,
          target_lang: 'ur',
          personalization: personalizationData
        }),
      });

      const data = await response.json();

      setTranslationResult({
        urdu: data.urdu_translation,
        english: data.english_translation,
        explanation: data.explanation
      });
    } catch (error) {
      console.error('Translation and explanation error:', error);
      setTranslationResult({
        urdu: 'ترجمہ دستیاب نہیں ہے',
        english: 'Translation not available',
        explanation: 'Explanation not available'
      });
    } finally {
      setLoading(false);
    }
  };

  // Trigger translation when text is selected
  useEffect(() => {
    if (selectedText && showTranslation) {
      translateAndExplain();
    }
  }, [selectedText, showTranslation]);

  return (
    <>
      {children}

      {/* Translation Popup */}
      {showTranslation && selectedText && (
        <div
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '400px',
            backgroundColor: '#fff',
            border: '1px solid #ccc',
            borderRadius: '8px',
            padding: '15px',
            zIndex: 10000,
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            fontFamily: 'Arial, sans-serif'
          }}
        >
          <div style={{ marginBottom: '10px' }}>
            <strong>Selected Text:</strong>
            <p style={{ fontStyle: 'italic', margin: '5px 0' }}>{selectedText}</p>
          </div>

          {loading ? (
            <p>Translating and explaining...</p>
          ) : translationResult ? (
            <>
              <div style={{ marginBottom: '10px' }}>
                <strong>Urdu Translation:</strong>
                <p
                  style={{
                    margin: '5px 0',
                    fontFamily: 'Noto Nastaliq Urdu, Jameel Noori Nastaleeq, Arial',
                    fontSize: '16px',
                    lineHeight: '1.8',
                    direction: 'rtl',
                    textAlign: 'right'
                  }}
                >
                  {translationResult.urdu}
                </p>
              </div>

              <div style={{ marginBottom: '10px' }}>
                <strong>English Translation:</strong>
                <p style={{ margin: '5px 0' }}>{translationResult.english}</p>
              </div>

              <div>
                <strong>Explanation:</strong>
                <p style={{ margin: '5px 0' }}>{translationResult.explanation}</p>
              </div>
            </>
          ) : null}

          <button
            onClick={() => setShowTranslation(false)}
            style={{
              marginTop: '10px',
              padding: '5px 10px',
              backgroundColor: '#007bff',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer'
            }}
          >
            Close
          </button>
        </div>
      )}
    </>
  );
};

export default TextSelectionHandler;