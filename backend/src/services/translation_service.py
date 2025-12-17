from typing import Optional
import openai
import os
import logging
from ..config.settings import settings
from ..utils.translation_utils import simple_translate_fallback


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TranslationService:
    def __init__(self):
        # Load the OpenAI API key from environment variables or settings
        api_key = os.getenv("OPENAI_API_KEY") or settings.OPENAI_API_KEY
        if not api_key:
            raise ValueError("OpenAI API key is not configured")

        self.client = openai.AsyncOpenAI(api_key=api_key)

    async def translate_text(self, text: str, use_fallback: bool = True) -> Optional[str]:
        """
        Translate English text to Urdu while preserving technical terms in English.
        """
        if not text or len(text.strip()) == 0:
            raise ValueError("Text field is required and cannot be empty")

        # Call OpenAI API for translation
        try:
            system_prompt = "You are a technical translator. Translate the text to Urdu. Keep technical terms (ROS 2, Gazebo, Isaac Sim, Python, Nodes, Topics) in English. Output only the translated text."

            logger.info(f"Sending translation request for text length: {len(text)}")
            response = await self.client.chat.completions.create(
                model="gpt-4o-mini",  # Using a more capable model for translation
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": text}
                ],
                max_tokens=2000,  # Adjust based on expected output length
                temperature=0.3
            )

            translated_text = response.choices[0].message.content.strip()
            logger.info(f"Successfully translated text of length {len(text)}")

            return translated_text

        except openai.AuthenticationError as e:
            logger.error(f"OpenAI authentication error: {str(e)}")
            if use_fallback:
                logger.info("Falling back to basic translation service")
                return simple_translate_fallback(text)
            else:
                raise Exception(f"Authentication failed: {str(e)}")
        except openai.RateLimitError as e:
            logger.error(f"OpenAI rate limit error: {str(e)}")
            if use_fallback:
                logger.info("Falling back to basic translation service due to rate limit")
                return simple_translate_fallback(text)
            else:
                raise Exception(f"Rate limit exceeded: {str(e)}")
        except openai.APIError as e:
            logger.error(f"OpenAI API error: {str(e)}")
            if use_fallback:
                logger.info("Falling back to basic translation service due to API error")
                return simple_translate_fallback(text)
            else:
                raise Exception(f"API error: {str(e)}")
        except Exception as e:
            logger.error(f"Translation failed: {str(e)}")
            if use_fallback:
                logger.info("Falling back to basic translation service due to unexpected error")
                return simple_translate_fallback(text)
            else:
                # Re-raise the exception to be handled by the API endpoint
                raise