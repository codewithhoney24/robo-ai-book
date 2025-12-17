import React, { useState } from 'react';

const API_ENDPOINT = "http://localhost:8000"; // FastAPI URL

interface TranslationResponse {
    [x: string]: string;
    translated_text: string;
}

// Props ko define karein taake TypeScript error na de
interface TranslateButtonProps {
    content: string; // Content string hoga
    onTranslation: (translatedText: string) => void; // Callback function
    lang?: string; // Optional string
}

// Button component jo translation initiate karega
export function TranslateButton({ content, onTranslation, lang = "Roman Urdu (Hinglish)" }: TranslateButtonProps) {
    const [isLoading, setIsLoading] = useState(false);

    const handleTranslate = async () => {
        setIsLoading(true);
        try {
            const response = await fetch(`${API_ENDPOINT}/api/v1/translate`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                // Poora content backend ko bhejein
                body: JSON.stringify({ text: content, target_language: lang }),
            });

            const result: TranslationResponse = await response.json();

            if (response.ok && onTranslation) {
                onTranslation(result.translated_text);
            } else {
                console.error("Translation API Error:", result);
                onTranslation(`Error: Translation failed from server: ${result.detail || 'Unknown error'}`);
            }
        } catch (error) {
            console.error("Network error during translation:", error);
            onTranslation(`Network Error: Could not connect to the translation service at ${API_ENDPOINT}.`);
        } finally {
            setIsLoading(false);
        }
    };

    return (
        <button
            onClick={handleTranslate}
            disabled={isLoading}
            className="button button--primary" // Docusaurus style
            style={{ marginBottom: '15px', marginLeft: '10px' }}
        >
            {isLoading ? 'Translating...' : `Translate to Urdu`}
        </button>
    );
}