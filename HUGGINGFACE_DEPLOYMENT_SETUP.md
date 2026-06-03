# Hugging Face Deployment Setup

This document describes the setup for deploying the AI Robotics Book application on Hugging Face Spaces, including both the backend API and the frontend Docusaurus website.

## 🏗️ Architecture Overview
- **Backend**: FastAPI application hosted on Hugging Face Spaces (using Docker SDK).
- **Frontend**: Docusaurus website hosted on Vercel or as a static site.

---

## ⚙️ Backend Setup (Hugging Face Spaces)

### API Endpoints
The backend API will be available at your Space's direct URL (e.g., `https://nousheenatif-robo-ai-backend.hf.space/api/v1`).

### Environment Variables (Secrets)
You must set these in the "Settings" tab of your Hugging Face Space:
- `COHERE_API_KEY`: API key for Cohere embeddings.
- `OPENAI_API_KEY`: API key for OpenAI LLM.
- `NEON_DATABASE_URL`: Connection string for Neon Postgres.
- `QDRANT_API_KEY`: API key for Qdrant Cloud.
- `QDRANT_ENDPOINT`: Endpoint URL for Qdrant.

---

## 🌐 Frontend Setup (Docusaurus)

The frontend needs to know where the backend is located. This is configured via the `REACT_APP_API_BASE_URL` environment variable.

### Configuration
In `website/docusaurus.config.ts`, the default fallback is set to:
`https://nousheenatif-robo-ai-backend.hf.space/api/v1`

---

## 🛠️ Deployment Steps

1.  **Backend**:
    - Create a new Docker Space on Hugging Face.
    - Push the repository to the Space.
    - Add secrets in the settings.
2.  **Frontend**:
    - Deploy to Vercel/Netlify.
    - Set `REACT_APP_API_BASE_URL` to your Space URL.
