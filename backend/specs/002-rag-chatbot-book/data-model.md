# Data Model: Integrated RAG Chatbot for Published Book

## Overview

This document defines the data model for the RAG chatbot feature, including entities, their attributes, relationships, and validation rules derived from the functional requirements.

## Entities

### User Query

**Description**: A natural language question or statement submitted by a reader about the book content

**Attributes**:
- `id` (string): Unique identifier for the query
- `content` (string): The actual text of the user's query
- `timestamp` (datetime): When the query was submitted
- `user_session_id` (string): Reference to the user's session
- `selected_text` (string, optional): Specific text selected by the user (if applicable)

**Validation Rules**:
- `content` must not be empty
- `content` length must be between 1 and 2000 characters
- `user_session_id` must be a valid session identifier

**Relationships**:
- Belongs to one `User Session`
- Zero or more `Retrieved Context` objects are associated with the query during processing

### Retrieved Context

**Description**: Book content segments retrieved from vector storage that are relevant to the user's query

**Attributes**:
- `id` (string): Unique identifier for the context
- `content` (string): The retrieved text from the book
- `relevance_score` (float): Confidence score of how relevant the content is to the query
- `source_location` (string): Where in the book the content came from (e.g., chapter, page, section)
- `query_id` (string): Reference to the user query that triggered this retrieval

**Validation Rules**:
- `content` must not be empty
- `relevance_score` must be between 0 and 1
- `source_location` must not be empty

**Relationships**:
- Belongs to one `User Query`
- Associated with the `Book Content` from which it was retrieved

### Generated Response

**Description**: AI-generated answer based on the retrieved context and the user's query

**Attributes**:
- `id` (string): Unique identifier for the response
- `content` (string): The text of the generated response
- `timestamp` (datetime): When the response was generated
- `query_id` (string): Reference to the user query
- `relevance_score` (float): Confidence score of how relevant the response is to the query

**Validation Rules**:
- `content` must not be empty
- `relevance_score` must be between 0 and 1

**Relationships**:
- Belongs to one `User Query`
- Based on one or more `Retrieved Context` objects

### Book Content

**Description**: The indexed textual information from the published book used as the knowledge base for the RAG system

**Attributes**:
- `id` (string): Unique identifier for the content block
- `content` (string): The text content
- `vector_embedding` (array of floats): Vector representation for similarity search
- `source_location` (string): Where in the book this content came from (chapter, page, section)
- `book_id` (string): Identifier for the book this content belongs to

**Validation Rules**:
- `content` must not be empty
- `vector_embedding` must have a consistent dimension
- `source_location` must not be empty
- `book_id` must be a valid book identifier

**Relationships**:
- Zero or more `Retrieved Context` objects are derived from this content

### User Session

**Description**: Temporary data container for a user's interaction with the chatbot, managed without permanent storage

**Attributes**:
- `session_id` (string): Unique identifier for the session
- `created_at` (datetime): When the session started
- `last_activity` (datetime): When the session was last active
- `user_agent` (string): Information about the user's browser/device

**Validation Rules**:
- `session_id` must be unique
- `created_at` must be before `last_activity`
- Session data must be cleared after the session expires

**Relationships**:
- Contains zero or more `User Query` objects