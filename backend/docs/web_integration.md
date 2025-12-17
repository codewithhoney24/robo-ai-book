# Web Integration Guide for RAG Chatbot

## Overview
This guide provides instructions for integrating the RAG Chatbot API into web-based book formats. The chatbot is designed to work seamlessly across different browsers and platforms.

## API Endpoints
The chatbot provides several endpoints for different use cases:

### 1. Basic Chat
- **Endpoint**: `POST /api/v1/chat`
- **Purpose**: Submit a user query to the chatbot
- **Request Body**:
  ```json
  {
    "query": "Your question about the book content",
    "session_id": "optional session identifier",
    "user_agent": "optional user agent string"
  }
  ```
- **Response**:
  ```json
  {
    "response": "Generated answer to your query",
    "relevance_score": 0.95,
    "retrieved_contexts": [
      {
        "content": "Relevant book content used to generate the response",
        "source_location": "Chapter 1, Section 2",
        "relevance_score": 0.89
      }
    ]
  }
  ```

### 2. Selected Text Chat
- **Endpoint**: `POST /api/v1/chat/selected-text`
- **Purpose**: Submit a query specifically related to user-selected text
- **Request Body**:
  ```json
  {
    "query": "Your question about the selected text",
    "selected_text": "The text selected by the user in the book",
    "session_id": "optional session identifier",
    "user_agent": "optional user agent string"
  }
  ```
- **Response**: Same structure as basic chat

## Session Management
The chatbot uses session management to maintain conversation context across requests:

- If you don't provide a `session_id`, a new session will be created automatically
- If you provide an existing `session_id`, the system will validate it and continue the conversation
- Sessions are valid for 24 hours from the last activity

## Cross-Platform Compatibility
The API is designed to work across different platforms and browsers:
- Tested on Chrome, Firefox, Safari, and Edge
- Compatible with mobile browsers
- Follows web standards for requests and responses

## Error Handling
Common error responses:
- `400`: Bad Request - Invalid query format or session
- `422`: Unprocessable Entity - Missing required fields
- `500`: Internal Server Error - Server-side issue

## Performance Optimization
- Response times are optimized to be under 1 second for most queries
- Caching is implemented for frequently asked questions
- Content is indexed for fast retrieval

## Security
- All endpoints support HTTPS
- No sensitive user data is stored permanently
- Rate limiting is implemented to prevent abuse

## Implementation Tips
1. Always handle session IDs to maintain conversation context
2. Implement timeout handling for API calls
3. Cache responses for common queries to improve user experience
4. Format selected text appropriately when using the selected-text endpoint