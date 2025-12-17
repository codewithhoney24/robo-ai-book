from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class RetrievedContext(BaseModel):
    """
    Book content segments retrieved from vector storage that are relevant to the user's query
    """
    id: str = Field(..., description="Unique identifier for the context")
    content: str = Field(..., description="The retrieved text from the book", min_length=1)
    relevance_score: float = Field(
        ..., 
        description="Confidence score of how relevant the content is to the query",
        ge=0.0,
        le=1.0
    )
    source_location: str = Field(
        ..., 
        description="Where in the book the content came from (e.g., chapter, page, section)",
        min_length=1
    )
    query_id: str = Field(..., description="Reference to the user query that triggered this retrieval")

    class Config:
        from_attributes = True