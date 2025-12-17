from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class BookContent(BaseModel):
    """
    The indexed textual information from the published book used as the knowledge base for the RAG system
    """
    id: str = Field(..., description="Unique identifier for the content block")
    content: str = Field(..., description="The text content", min_length=1)
    vector_embedding: List[float] = Field(
        default_factory=list,
        description="Vector representation for similarity search"
    )
    source_location: str = Field(
        ...,
        description="Where in the book this content came from (chapter, page, section)",
        min_length=1
    )
    book_id: str = Field(..., description="Identifier for the book this content belongs to")

    class Config:
        from_attributes = True