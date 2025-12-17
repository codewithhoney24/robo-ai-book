from pydantic import BaseModel
from typing import Optional


class TranslationRequest(BaseModel):
    text: str
    selected_text: Optional[str] = None


class TranslationResponse(BaseModel):
    translated_text: str