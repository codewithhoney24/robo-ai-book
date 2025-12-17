from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class UserQuery(BaseModel):
    """
    A natural language question or statement submitted by a reader about the book content
    """
    id: str = Field(..., description="Unique identifier for the query")
    content: str = Field(
        ..., 
        description="The actual text of the user's query",
        min_length=1,
        max_length=2000
    )
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the query was submitted")
    user_session_id: str = Field(..., description="Reference to the user's session")
    selected_text: Optional[str] = Field(
        None, 
        description="Specific text selected by the user (if applicable)",
        max_length=5000
    )

    class Config:
        # Enable ORM mode to work with SQLAlchemy models if needed later
        from_attributes = True