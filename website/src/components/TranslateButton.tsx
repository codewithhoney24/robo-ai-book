import React, { useState } from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';

interface TranslateButtonProps {
  content?: string;
  onTranslation?: (translatedText: string) => void;
  lang?: string;
}

const TranslateButton: React.FC<TranslateButtonProps> = ({
  content = '',
  onTranslation,
  lang = 'ur'
}) => {
  const { settings, updateSettings } = usePersonalization();
  const [isOpen, setIsOpen] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);

  const toggleMenu = () => setIsOpen(!isOpen);

  const languages = [
    { code: 'en', name: 'English' },
    { code: 'es', name: 'Spanish' },
    { code: 'fr', name: 'French' },
    { code: 'de', name: 'German' },
    { code: 'zh', name: 'Chinese' },
    { code: 'ja', name: 'Japanese' },
    { code: 'ko', name: 'Korean' },
    { code: 'ur', name: 'Urdu' },
    { code: 'hi', name: 'Hindi' },
  ];

  const handleTranslate = async (targetLang: string) => {
    setIsTranslating(true);
    setIsOpen(false);

    try {
      // Simulate API call to translation service
      // In a real implementation, this would call an actual translation API
      await new Promise(resolve => setTimeout(resolve, 1000));

      // For demo purposes, just return the content with "Translated to [lang]" prefix
      const translatedText = `Translated to ${languages.find(l => l.code === targetLang)?.name}: ${content}`;

      if (onTranslation) {
        onTranslation(translatedText);
      }
    } catch (error) {
      if (onTranslation) {
        onTranslation(`Error: ${error}`);
      }
    } finally {
      setIsTranslating(false);
    }
  };

  return (
    <div className="translate-button-container">
      {isTranslating ? (
        <button className="translate-button" disabled>
          Translating...
        </button>
      ) : (
        <button onClick={toggleMenu} className="translate-button">
          {settings.language.toUpperCase()}
        </button>
      )}
      {isOpen && (
        <div className="translate-menu">
          <div className="translate-options">
            {languages.map((lang) => (
              <button
                key={lang.code}
                className={`translate-option ${settings.language === lang.code ? 'active' : ''}`}
                onClick={() => handleTranslate(lang.code)}
              >
                {lang.name}
              </button>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};

export default TranslateButton;