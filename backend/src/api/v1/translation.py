from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, validator
from typing import Optional
import logging

# =====================================================
# TRY MULTIPLE TRANSLATION LIBRARIES
# =====================================================
TRANSLATOR_TYPE = None

# Option 1: Try deep-translator first
try:
    from deep_translator import GoogleTranslator
    TRANSLATOR_TYPE = "deep_translator"
    print("[OK] Using deep-translator")
except ImportError:
    pass

# Option 2: Try googletrans as backup
if not TRANSLATOR_TYPE:
    try:
        from googletrans import Translator
        TRANSLATOR_TYPE = "googletrans"
        print("[OK] Using googletrans")
    except ImportError:
        pass

# Option 3: Try translate library as last resort
if not TRANSLATOR_TYPE:
    try:
        from translate import Translator as BasicTranslator
        TRANSLATOR_TYPE = "translate"
        print("[OK] Using translate library")
    except ImportError:
        pass

if not TRANSLATOR_TYPE:
    print("[WARN] No translation library found!")
    print("Install one of these:")
    print("  pip install deep-translator")
    print("  pip install googletrans==3.1.0a0")
    print("  pip install translate")

# Configure logger
logger = logging.getLogger("api")
router = APIRouter()

# =====================================================
# REQUEST/RESPONSE MODELS
# =====================================================
class TranslationRequest(BaseModel):
    text: str
    source_lang: Optional[str] = "en"
    target_lang: Optional[str] = "ur"
    use_ai: Optional[bool] = False

    @validator('text')
    def text_must_not_be_empty(cls, v):
        if not v or not v.strip():
            raise ValueError('Text cannot be empty')
        return v.strip()


class TranslationResponse(BaseModel):
    translated_text: str
    source_lang: str
    target_lang: str
    original_length: int
    translated_length: int
    translation_method: str


# =====================================================
# TRANSLATION FUNCTION (WORKS WITH ANY LIBRARY)
# =====================================================
def translate_text_universal(text: str, target_lang: str = "ur") -> str:
    """
    Universal translation function that works with multiple libraries
    """
    if not TRANSLATOR_TYPE:
        raise HTTPException(
            status_code=503,
            detail="No translation library installed. Run: pip install deep-translator"
        )

    try:
        # METHOD 1: deep-translator
        if TRANSLATOR_TYPE == "deep_translator":
            # Split into chunks (Google has 5000 char limit)
            max_chunk_size = 4500
            chunks = [text[i:i+max_chunk_size] for i in range(0, len(text), max_chunk_size)]

            translated_chunks = []
            translator = GoogleTranslator(source='en', target=target_lang)

            for chunk in chunks:
                translated = translator.translate(chunk)
                translated_chunks.append(translated)

            return " ".join(translated_chunks)

        # METHOD 2: googletrans
        elif TRANSLATOR_TYPE == "googletrans":
            translator = Translator()

            # Split into chunks if text is too long
            if len(text) > 4500:
                max_chunk_size = 4500
                chunks = [text[i:i+max_chunk_size] for i in range(0, len(text), max_chunk_size)]

                translated_chunks = []
                for chunk in chunks:
                    result = translator.translate(chunk, src='en', dest=target_lang)
                    translated_chunks.append(result.text)

                return " ".join(translated_chunks)
            else:
                result = translator.translate(text, src='en', dest=target_lang)
                return result.text

        # METHOD 3: translate library
        elif TRANSLATOR_TYPE == "translate":
            translator = BasicTranslator(to_lang=target_lang)

            # This library has limited support, so we'll do it in smaller chunks
            max_chunk_size = 500
            chunks = [text[i:i+max_chunk_size] for i in range(0, len(text), max_chunk_size)]

            translated_chunks = []
            for chunk in chunks:
                translated = translator.translate(chunk)
                translated_chunks.append(translated)

            return " ".join(translated_chunks)

    except Exception as e:
        logger.error(f"Translation error ({TRANSLATOR_TYPE}): {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Translation failed: {str(e)}"
        )


# =====================================================
# TRANSLATION ENDPOINT
# =====================================================
@router.post("/translate", response_model=TranslationResponse)
async def translate_text(request: Request, translation_req: TranslationRequest):
    """
    Translate text from source language to target language
    """
    try:
        # Log the incoming request
        user_agent = request.headers.get("user-agent", "Unknown")
        logger.debug(f"Request from user agent: {user_agent}")
        logger.info(f"Translation request - Length: {len(translation_req.text)} chars, AI: {translation_req.use_ai}")

        # Validate text length
        if len(translation_req.text) > 50000:
            raise HTTPException(
                status_code=413,
                detail="Text too long. Maximum 50,000 characters allowed."
            )

        # Translate using universal function
        translated_text = translate_text_universal(
            translation_req.text,
            translation_req.target_lang
        )

        logger.info(f"Translation successful using {TRANSLATOR_TYPE}")

        return TranslationResponse(
            translated_text=translated_text,
            source_lang=translation_req.source_lang,
            target_lang=translation_req.target_lang,
            original_length=len(translation_req.text),
            translated_length=len(translated_text),
            translation_method=TRANSLATOR_TYPE
        )

    except ValueError as ve:
        logger.error(f"Validation error: {str(ve)}")
        raise HTTPException(status_code=400, detail=str(ve))

    except HTTPException:
        raise

    except Exception as e:
        logger.error(f"Translation error: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"Translation service error: {str(e)}"
        )


# =====================================================
# BATCH TRANSLATION (for chatbot)
# =====================================================
@router.post("/translate/batch")
async def translate_batch(messages: list[dict]):
    """
    Translate multiple messages at once
    """
    try:
        if not TRANSLATOR_TYPE:
            raise HTTPException(
                status_code=503,
                detail="No translation library available"
            )

        translated_messages = []

        for msg in messages:
            translated_content = translate_text_universal(
                msg.get("content", ""),
                "ur"
            )
            translated_messages.append({
                "role": msg.get("role"),
                "content": translated_content,
                "original_content": msg.get("content")
            })

        return {
            "translated_messages": translated_messages,
            "count": len(translated_messages)
        }

    except Exception as e:
        logger.error(f"Batch translation error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


# =====================================================
# HEALTH CHECK
# =====================================================
@router.get("/translate/health")
async def translation_health():
    """Check translation service availability"""
    return {
        "status": "healthy" if TRANSLATOR_TYPE else "unavailable",
        "service": "translation",
        "active_library": TRANSLATOR_TYPE or "none",
        "supported_languages": {
            "source": ["en", "auto"],
            "target": ["ur", "hi", "ar", "fa"]
        },
        "installation_help": {
            "preferred": "pip install deep-translator",
            "alternative_1": "pip install googletrans==3.1.0a0",
            "alternative_2": "pip install translate"
        }
    }