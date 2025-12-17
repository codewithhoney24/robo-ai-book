# API Documentation: RAG Chatbot for Published Book

## Base URL
All API endpoints are prefixed with `/api/v1`

## Authentication
This API does not use authentication for simplicity. For production, consider implementing API keys or OAuth.

## Endpoints

### Chat
`POST /chat`

Process a natural language query and return a generated response based on book content.

#### Request Body
```json
{
  "query": "Your question about the book content",
  "session_id": "optional session identifier to maintain conversation context"
}
```

**Fields:**
- `query` (string, required): The user's natural language query about the book content. Must be 1-2000 characters.
- `session_id` (string, optional): Session identifier to maintain conversation context.

#### Response
```json
{
  "response": "Generated response to your query",
  "relevance_score": 0.95,
  "retrieved_contexts": [
    {
      "content": "Relevant book content used to generate the response",
      "source_location": "Chapter 3, Section 2, Page 45",
      "relevance_score": 0.89
    }
  ]
}
```

**Response Fields:**
- `response` (string): The generated answer to the user's query
- `relevance_score` (number): Confidence score of how relevant the response is to the query (0.0 to 1.0)
- `retrieved_contexts` (array): The contexts used to generate the response

#### Error Responses
- `400 Bad Request`: Invalid query parameters
- `422 Unprocessable Entity`: Missing required fields or validation errors
- `500 Internal Server Error`: Server-side error

---

### Chat with Selected Text
`POST /chat/selected-text`

Process a query specifically related to user-selected text within the book.

#### Request Body
```json
{
  "query": "Your question about the selected text",
  "selected_text": "The specific text selected by the user in the book",
  "session_id": "optional session identifier"
}
```

**Fields:**
- `query` (string, required): The user's query about the selected text
- `selected_text` (string, required): The specific text selected by the user in the book
- `session_id` (string, optional): Session identifier to maintain conversation context

#### Response
Same structure as the `/chat` endpoint.

#### Error Responses
- `400 Bad Request`: Invalid query parameters
- `422 Unprocessable Entity`: Missing required fields or validation errors
- `500 Internal Server Error`: Server-side error

---

### Index Book Content
`POST /api/v1/book-content/index`

Index book content for RAG retrieval.

#### Request Body
```json
{
  "book_id": "unique identifier for the book",
  "content_blocks": [
    {
      "id": "unique identifier for the content block",
      "content": "The text content of the block",
      "source_location": "Where in the book this content came from (e.g., chapter, page, section)",
      "book_id": "The same book ID as in the main request body"
    }
  ]
}
```

**Note:** Each content block must have all required fields:
- `id` (string, required): Unique identifier for the content block
- `content` (string, required): The text content of the block (at least 1 character)
- `source_location` (string, required): Where in the book this content came from (e.g., chapter, page, section)
- `book_id` (string, required): The same book ID as in the main request body

#### Response
```json
{
  "message": "Successfully indexed [number] content blocks for book [book_id]",
  "success": true
}
```

#### Error Responses
- `422 Unprocessable Entity`: Missing required fields (book_id or content_blocks), or content_blocks is not a non-empty list, or any content block is invalid
- `500 Internal Server Error`: Server-side error during indexing

---

### Index Text Content
`POST /api/v1/book-content/index-text`

Split book text into chunks and index them for RAG retrieval.

#### Request Body
```json
{
  "book_id": "unique identifier for the book",
  "full_text": "The complete text of the book to be split and indexed",
  "chunk_size": "optional size for text chunks (default: 1000)"
}
```

#### Response
```json
{
  "message": "Successfully indexed text for book [book_id]",
  "success": true
}
```

#### Error Responses
- `422 Unprocessable Entity`: Missing required fields (book_id or full_text)
- `500 Internal Server Error`: Server-side error during indexing

---

### Health Check
`GET /health`

Check the health status of the API.

#### Response
```json
{
  "status": "healthy",
  "timestamp": "2025-12-14T10:00:00Z"
}
```

## Error Format
All error responses follow this format:
```json
{
  "error": "ERROR_CODE",
  "message": "Human-readable error message"
}
```

## Rate Limiting
The API does not currently implement rate limiting. For production use, implement rate limiting at the application or infrastructure level.

## CORS Policy
The API allows requests from all origins (`*`). For production, configure specific origins as needed.

## Additional Endpoints

### Session Validation
`No dedicated endpoint` - Sessions are managed automatically. A new session is created if no `session_id` is provided. If a `session_id` is provided, it's validated automatically during API calls.

### Security Headers
All responses include the following security headers:
- `X-Content-Type-Options: nosniff`
- `X-Frame-Options: DENY`
- `X-XSS-Protection: 1; mode=block`

### Cross-Platform Compatibility
The API includes middleware to handle compatibility across different browsers and platforms, with special handling for potentially incompatible user agents.