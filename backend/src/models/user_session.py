from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class UserSession(BaseModel):
    """
    Represents a user's session in the application
    """
    session_id: str = Field(..., description="Unique identifier for the session")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="When the session was created")
    last_activity: datetime = Field(default_factory=datetime.utcnow, description="When the session was last active")
    user_agent: Optional[str] = Field(
        None,
        description="User agent string of the client",
        max_length=500
    )

    class Config:
        # Enable ORM mode to work with SQLAlchemy models if needed later
        from_attributes = True