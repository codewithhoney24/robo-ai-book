from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class UserSession(BaseModel):
    """
    Temporary data container for a user's interaction with the chatbot, managed without permanent storage
    """
    session_id: str = Field(..., description="Unique identifier for the session")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="When the session started")
    last_activity: datetime = Field(default_factory=datetime.utcnow, description="When the session was last active")
    user_agent: Optional[str] = Field(None, description="Information about the user's browser/device")

    class Config:
        from_attributes = True