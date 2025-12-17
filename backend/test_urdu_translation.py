import asyncio
import os
from src.services.translation_service import TranslationService


async def test_urdu_translation():
    """
    Test the Urdu translation functionality
    """
    # Create an instance of the translation service
    translation_service = TranslationService()

    # Sample text to translate
    sample_text = """
    The Robot Operating System 2 (ROS 2) provides a flexible framework for writing robot software. 
    It's a collection of tools, libraries, and conventions that aim to simplify the task of creating 
    complex and robust robot behaviors across a wide variety of robotic platforms. 
    Think of it as the "nervous system" for your robot, coordinating all its different parts.
    """

    print("Original text:")
    print(sample_text)
    print("\n" + "="*50 + "\n")

    try:
        # Translate the text to Urdu
        translated_text = await translation_service.translate_text(sample_text)
        
        print("Translated text (Urdu):")
        print(translated_text)
        
        # Verify that the translation contains Urdu characters
        urdu_chars = [char for char in translated_text if '\u0600' <= char <= '\u06FF' or '\u0750' <= char <= '\u077F']
        if urdu_chars:
            print(f"\n✅ Translation successful! Contains {len(urdu_chars)} Urdu characters.")
        else:
            print("\n⚠️  Translation may not contain Urdu characters.")
            
    except Exception as e:
        print(f"❌ Translation failed with error: {str(e)}")


if __name__ == "__main__":
    # Ensure OpenAI API key is set
    if not os.getenv("OPENAI_API_KEY"):
        print("⚠️  Warning: OPENAI_API_KEY environment variable not set.")
        print("Please set your OpenAI API key before running this test.")
    
    asyncio.run(test_urdu_translation())