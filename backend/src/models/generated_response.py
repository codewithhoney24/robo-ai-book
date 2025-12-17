from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime
from .retrieved_context import RetrievedContext


class GeneratedResponse(BaseModel):
    """
    AI-generated answer based on the retrieved context and the user's query
    """
    id: str = Field(..., description="Unique identifier for the response")
    content: str = Field(..., description="The text of the generated response", min_length=1)
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the response was generated")
    query_id: str = Field(..., description="Reference to the user query")
    relevance_score: float = Field(
        ..., 
        description="Confidence score of how relevant the response is to the query",
        ge=0.0,
        le=1.0
    )
    retrieved_contexts: List[RetrievedContext] = Field(
        default_factory=list,
        description="The contexts used to generate this response"
    )

    class Config:
        from_attributes = True