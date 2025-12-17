# Quickstart Guide: Integrated RAG Chatbot for Published Book

## Overview

This guide provides step-by-step instructions to set up and run the RAG chatbot service for book integration. The service allows readers to ask questions about book content and receive AI-generated responses based on the book's information.

## Prerequisites

- Python 3.11 or higher
- pip package manager
- Access to Qdrant vector database (endpoint: https://9672a74d-60a3-423b-a6ad-5bb0c3b1b691.europe-west3-0.gcp.cloud.qdrant.io)
- Access to Neon Postgres database
- Cohere API key
- Qwen CLI installed and configured

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables

Create a `.env` file in the project root with the following variables:

```env
QDRANT_ENDPOINT=https://9672a74d-60a3-423b-a6ad-5bb0c3b1b691.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.Qq10rC6zEHq432Zp5p3LIv297R2Wz9F4J-MqXP0XHYY
QDRANT_CLIENT_URL=https://9672a74d-60a3-423b-a6ad-5bb0c3b1b691.europe-west3-0.gcp.cloud.qdrant.io:6333
NEON_DATABASE_URL=postgresql://neondb_owner:npg_gPuCILYeV73E@ep-square-glitter-a854b5sd-pooler.eastus2.azure.neon.tech/neondb?sslmode=require&channel_binding=require
COHERE_API_KEY=2ZmqzoGjZCpYb8GwEKn8kzIDtJluJ7dbV2mAVOXs
QWEN_CLI_PATH=/path/to/qwen-cli  # Update this to your Qwen CLI installation
```

### 5. Initialize Vector Database

Run the following command to initialize the Qdrant vector store with book content:

```bash
python -m scripts.initialize_vector_store --book-content-path /path/to/book/content
```

## Running the Service

### Development Mode

```bash
# Activate virtual environment
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Run the FastAPI server
uvicorn src.api.main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`.

### Production Mode

```bash
# Using gunicorn for production
gunicorn src.api.main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

## API Usage

### Submit a Query

```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the main concept behind RAG systems?",
    "session_id": "optional_session_id"
  }'
```

### Query with Selected Text

```bash
curl -X POST http://localhost:8000/api/v1/chat/selected-text \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept more deeply",
    "selected_text": "Retrieval-Augmented Generation (RAG) is a technique that combines information retrieval with text generation to provide more accurate and factual responses.",
    "session_id": "optional_session_id"
  }'
```

### Index Book Content

```bash
curl -X POST http://localhost:8000/api/v1/book-content/index \
  -H "Content-Type: application/json" \
  -d '{
    "book_id": "book_identifier",
    "content_blocks": [
      {
        "id": "block_1",
        "content": "This is the first content block of the book.",
        "source_location": "Chapter 1, Page 1"
      }
    ]
  }'
```

### Health Check

```bash
curl http://localhost:8000/health
```

## Testing

To run the unit tests:

```bash
pytest tests/unit/
```

To run the integration tests:

```bash
pytest tests/integration/
```

To run all tests with coverage:

```bash
pytest --cov=src/ --cov-report=html
```

## Project Structure

### Source Code

```
backend/
├── src/
│   ├── models/
│   │   ├── user_query.py           # Data model for user queries
│   │   ├── retrieved_context.py    # Data model for retrieved contexts
│   │   ├── generated_response.py   # Data model for generated responses
│   │   ├── book_content.py         # Data model for book content
│   │   └── user_session.py         # Data model for user sessions
│   ├── services/
│   │   ├── rag_service.py          # Core RAG logic implementation
│   │   ├── vector_store_service.py # Qdrant integration with selected text prioritization
│   │   ├── database_service.py     # Neon Postgres operations
│   │   ├── embedding_service.py    # Cohere API integration
│   │   ├── session_service.py      # Session management for web interface
│   │   └── optimization_service.py # Performance optimization features
│   ├── api/
│   │   ├── main.py                 # FastAPI application entry point with middleware
│   │   ├── chatbot_router.py       # Chatbot API endpoints with session handling
│   │   └── book_content_router.py  # Book content management endpoints
│   ├── config/
│   │   ├── settings.py             # Configuration management
│   │   ├── database_config.py      # Database connection settings
│   │   └── logging_config.py       # Logging infrastructure
│   └── exceptions/                 # Custom exception definitions
│       └── __init__.py
└── tests/
    ├── contract/                   # API contract tests
    ├── integration/                # Integration tests for cross-cutting functionality
    └── unit/                       # Unit tests for individual components
```

### Documentation

- `docs/api.md` - Complete API documentation
- `docs/web_integration.md` - Guide for web interface integration
- `docs/selected_text.md` - Documentation for selected text feature
- `README.md` - Project overview and setup guide

### Additional Features

- **Session Management**: Maintains conversation context with automatic session creation and activity tracking
- **Cross-Platform Compatibility**: Middleware to handle different browsers and platforms
- **Selected Text Processing**: Enhanced RAG that prioritizes contexts relevant to user-selected text
- **Performance Optimization**: Caching and batch processing for improved response times
- **Error Handling**: Comprehensive error responses with appropriate HTTP status codes
- **Logging**: Structured logging for debugging and monitoring

## Troubleshooting

### Common Issues

1. **Connection Errors**: Verify that your API keys and endpoints are correctly configured in the `.env` file.

2. **Vector Search Not Returning Results**: Check if the book content has been properly indexed in Qdrant.

3. **Slow Response Times**: Monitor your Qdrant and Neon usage to ensure you're within free tier limits.

4. **Qwen CLI Not Configured**: If the Qwen CLI is not configured, the system will fall back to a simple response. Set the correct path in your `.env` file with `QWEN_CLI_PATH`.

5. **Session Expiry**: Sessions expire after 24 hours of inactivity. If you get session-related errors, generate a new session.

6. **Selected Text Not Prioritized**: If the selected text feature isn't working as expected, ensure that the selected text is being sent with the query and that the content has been properly indexed in Qdrant.

### Checking Service Health

```bash
curl http://localhost:8000/health
```

This should return a health status indicating the service is operational.

### Session Validation

If you're experiencing issues with session management, ensure your client is properly sending and storing session IDs between requests. For new sessions, omit the session_id field and the server will generate one for you.

### Testing the Implementation

After completing the setup, run the following tests to verify the implementation:

```bash
# Run all unit tests
pytest tests/unit/

# Run all integration tests
pytest tests/integration/

# Run contract tests
pytest tests/contract/

# Run all tests with coverage (should be 80%+)
pytest --cov=src/ --cov-report=html
```

### Indexing Book Content

Before you can use the chatbot with your book content, you need to index it:

```bash
# Method 1: Index content blocks individually
curl -X POST http://localhost:8000/api/v1/book-content/index \
  -H "Content-Type: application/json" \
  -d '{
    "book_id": "my_book",
    "content_blocks": [
      {
        "id": "ch1_sec1",
        "content": "Chapter 1, Section 1 content goes here...",
        "source_location": "Chapter 1, Section 1"
      }
    ]
  }'

# Method 2: Index full text which will be automatically chunked
curl -X POST http://localhost:8000/api/v1/book-content/index-text \
  -H "Content-Type: application/json" \
  -d '{
    "book_id": "my_book",
    "full_text": "Full book text goes here...",
    "chunk_size": 1000
  }'
```