"""
Utility functions for translation fallback when primary service is unavailable
"""
import logging
from typing import Optional


logger = logging.getLogger(__name__)


def simple_translate_fallback(text: str, target_language: str = "ur") -> Optional[str]:
    """
    Simple fallback translation using basic text replacement.
    This is a very basic implementation and should be replaced with a proper 
    translation service in production (like googletrans or Azure Translator).
    
    Args:
        text: Text to translate
        target_language: Target language code (default: 'ur' for Urdu)
        
    Returns:
        Translated text or None if translation fails
    """
    # This is a placeholder implementation.
    # In a real application, you would use a translation library like:
    # from googletrans import Translator
    # translator = Translator()
    # result = translator.translate(text, dest=target_language)
    # return result.text
    
    try:
        # For now, return a helpful message to indicate the service is unavailable
        fallback_message = (
            f"Translation service unavailable. Original text: {text[:100]}..."
            f" ({len(text)} chars total) - TRANSLATION SERVICE NOT CONFIGURED"
        )
        logger.warning("Using fallback translation - proper service needed for actual translation")
        return fallback_message
    except Exception as e:
        logger.error(f"Failed to generate fallback translation: {str(e)}")
        return None