# Robo-AI-Book Platform Deployment

This document explains how to deploy the Robo-AI-Book platform to Railway.

## Project Overview

The Robo-AI-Book platform is a Physical AI textbook platform with the following features:
- RAG (Retrieval Augmented Generation) chatbot for textbook content
- Content personalization based on user preferences
- Translation services (Urdu and other languages)
- System logging and monitoring

## Architecture

The platform consists of:
- **Backend**: FastAPI application in the `backend/` directory
- **Frontend**: Docusaurus website in the `website/` directory
- **Database**: PostgreSQL (Neon) for storing user queries and responses
- **Vector Store**: Qdrant for content retrieval
- **AI Services**: Cohere and OpenAI for embeddings and generation

## Deployment Steps

1. **Log in to Railway**:
   ```bash
   railway login
   ```
   
2. **Navigate to the project directory**:
   ```bash
   cd ai-robo-bk
   ```

3. **Link the project to a new or existing Railway service**:
   ```bash
   railway init
   ```

4. **Set environment variables**:
   ```bash
   railway var set COHERE_API_KEY=<your-cohere-api-key>
   railway var set OPENAI_API_KEY=<your-openai-api-key>
   railway var set DATABASE_URL=<your-postgresql-url>
   railway var set QDRANT_API_KEY=<your-qdrant-api-key>
   railway var set QDRANT_ENDPOINT=<your-qdrant-endpoint>
   ```

5. **Deploy the application**:
   ```bash
   railway up
   ```

## Environment Variables Required

The following environment variables need to be set for the application to work correctly:

- `COHERE_API_KEY`: API key for Cohere services
- `OPENAI_API_KEY`: API key for OpenAI services
- `DATABASE_URL`: PostgreSQL database connection string
- `QDRANT_API_KEY`: API key for Qdrant vector store
- `QDRANT_ENDPOINT`: Endpoint URL for Qdrant vector store

## Project Configuration

The project is configured for Railway deployment with:
- A `railway.json` file specifying build and deployment settings
- A `Dockerfile` for containerized deployment
- A `Procfile` for process management
- A `runtime.txt` specifying the Python version

## Monitoring

After deployment, you can monitor your application using:
- `railway logs` - View application logs
- `railway dashboard` - Open the Railway dashboard in your browser
- `railway status` - Check the status of your deployments

## Troubleshooting

If you encounter issues during deployment:

1. Check that all required environment variables are set
2. Verify that your API keys are valid and have the necessary permissions
3. Review the logs with `railway logs` for error messages
4. Ensure that your database connection string is properly formatted