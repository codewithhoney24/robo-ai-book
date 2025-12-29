# Backend API

Backend API service for the Physical AI textbook platform with RAG chatbot functionality.

## Features

- RAG chatbot for querying textbook content
- Content personalization based on user preferences
- Translation services
- Logging and metrics
- Rate limiting for security

## Tech Stack

- **Framework**: FastAPI
- **Database**: PostgreSQL (Neon)
- **Vector Store**: Qdrant
- **Validation**: Pydantic
- **Type Safety**: Python type hints

## Setup

1. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Create a `.env` file based on `.env.example`:
   ```bash
   cp .env.example .env
   ```

4. Update the `.env` file with your configuration:
   - Set `DATABASE_URL` to your PostgreSQL connection string
   - Set API keys for Cohere, OpenAI, Qdrant, etc.
   - Update other configuration values as needed

5. Run the application:
   ```bash
   uvicorn src.api.main:app --reload  # For development
   uvicorn src.api.main:app --host 0.0.0.0 --port 8000  # For production
   ```

## API Endpoints

- `POST /api/v1/chat`: RAG chatbot endpoint
- `POST /api/v1/personalize`: Content personalization
- `POST /api/v1/translate`: Translation services
- `GET /api/v1/logs`: System logs
- `GET /health`: Health check

## Environment Variables

- `DATABASE_URL`: PostgreSQL connection string
- `NEON_DATABASE_URL`: Neon database URL (if different from DATABASE_URL)
- `QDRANT_ENDPOINT`: Qdrant vector store endpoint
- `QDRANT_API_KEY`: Qdrant API key
- `QDRANT_CLIENT_URL`: Qdrant client URL
- `COHERE_API_KEY`: Cohere API key
- `COHERE_EMBED_MODEL`: Cohere embedding model name
- `OPENAI_API_KEY`: OpenAI API key (optional)
- `CORS_ORIGIN`: Comma-separated list of allowed origins
- `PORT`: Port to run the server on (default: 8000)

## Security

- Input validation using Pydantic models
- Rate limiting to prevent abuse
- Secure API key handling