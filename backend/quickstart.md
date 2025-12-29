# Quickstart Guide: Backend API for Physical AI Textbook Platform

This guide will help you quickly verify that all backend services are working correctly.

## Prerequisites

- Python 3.11+
- PostgreSQL database
- Git

## Setup

1. Clone the repository:
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. Set up the backend:
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   cp .env.example .env
   # Update .env with your database URL and API keys
   uvicorn src.api.main:app --reload
   ```

## Validation Checklist

### RAG Chatbot Functionality

1. Test the chat endpoint:
   ```bash
   curl -X POST http://localhost:8000/api/v1/chat -H "Content-Type: application/json" -d '{"query": "What is ROS 2?"}'
   ```
2. Verify: You receive a relevant response about ROS 2

### Content Personalization

1. Test the personalization endpoint:
   ```bash
   curl -X POST http://localhost:8000/api/v1/personalize -H "Content-Type: application/json" -d '{"content": "Introduction to robotics", "expertise_level": "beginner"}'
   ```
2. Verify: You receive personalized content appropriate for the expertise level

### Translation Service

1. Test the translation endpoint:
   ```bash
   curl -X POST http://localhost:8000/api/v1/translate -H "Content-Type: application/json" -d '{"text": "Hello, world!", "target_lang": "ur"}'
   ```
2. Verify: You receive the text translated to Urdu

### Health Check

1. Navigate to http://localhost:8000/health or http://localhost:8000/docs
2. Verify: The health endpoint returns status information and the API documentation is accessible

## API Endpoints Verification

- `POST /api/v1/chat` - Verify RAG chatbot functionality
- `POST /api/v1/personalize` - Verify content personalization
- `POST /api/v1/translate` - Verify translation service
- `GET /api/v1/logs` - Verify logging service
- `GET /health` - Verify health check

## Security Checks

- Input validation is properly enforced
- Rate limiting is enforced
- API keys are required for protected endpoints
- CORS is properly configured

## Troubleshooting

If any of the above steps fail:

1. Check backend logs for error messages
2. Verify your `.env` configuration
3. Ensure your database is accessible
4. Confirm API keys are properly set
5. Check that all required services (Qdrant, PostgreSQL) are running

## Next Steps

After successful validation:

1. Configure production environment variables
2. Set up proper logging and monitoring
3. Test the deployed application
4. Integrate with the frontend application