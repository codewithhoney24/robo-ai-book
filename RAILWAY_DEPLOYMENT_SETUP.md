# Railway Deployment Setup

## Overview
This document describes the setup for deploying the AI Robotics Book application on Railway, including both the backend API and frontend Docusaurus website.

## Architecture
- **Backend**: FastAPI application hosted on Railway
- **Frontend**: Docusaurus website hosted on Railway
- **Database**: Neon PostgreSQL database
- **Vector Store**: Qdrant cloud instance
- **AI Services**: Cohere and OpenAI APIs

## Backend Configuration

### API Endpoints
The backend API is available at `https://web-production-3ced.up.railway.app/api/v1` with the following main endpoints:

- `/api/v1/health` - Health check
- `/api/v1/chat` - Main chat functionality
- `/api/v1/book-content` - Book content indexing
- `/api/v1/auth` - Authentication services
- `/api/v1/personalization` - Content personalization
- `/api/v1/translation` - Text translation
- `/api/v1/metrics` - Performance metrics
- `/api/v1/database` - Database operations

### Environment Variables
The backend uses the following key environment variables:
- `DATABASE_URL` - PostgreSQL database connection string
- `QDRANT_ENDPOINT` - Vector database endpoint
- `COHERE_API_KEY` - Cohere API key for embeddings (primary service)
- `OPENAI_API_KEY` - OpenAI API key (fallback service)
- `HOST` - Set to `0.0.0.0` for Railway deployment
- `PORT` - Port number (automatically set by Railway)

## Frontend Configuration

### API Connection
The frontend Docusaurus site is configured to connect to the backend API at:
`https://web-production-3ced.up.railway.app/api/v1`

This is configured in `website/docusaurus.config.ts` in the `customFields.REACT_APP_API_BASE_URL` setting.

### Build Process
The Docusaurus site is built using the standard command:
```bash
npm run build
```

### Deployment
The frontend is deployed to Railway and serves static files.

## Connection Flow

1. Frontend makes API calls to `https://web-production-3ced.up.railway.app/api/v1`
2. Backend receives requests and processes them
3. Backend connects to Neon PostgreSQL database for data storage
4. Backend connects to Qdrant for vector storage and similarity search
5. Backend uses Cohere and OpenAI APIs for AI functionality
6. Responses are sent back to the frontend

## Troubleshooting

### Common Issues
- If API calls fail, verify the backend is running at the correct URL
- Check that environment variables are properly set in Railway
- Verify database and vector store connections

### Health Checks
- Visit `/api/v1/health` to check backend status
- Visit `/api/v1/health/database` to check database connectivity
- Check the Railway logs for any errors

## Security Considerations
- API keys are stored as environment variables in Railway
- CORS is configured to allow necessary origins
- Authentication is handled via session management