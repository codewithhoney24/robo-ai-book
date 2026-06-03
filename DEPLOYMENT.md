# Deployment Guide for Physical AI & Humanoid Robotics Platform

This document explains how to deploy the Robo-AI-Book platform to Hugging Face Spaces.

## 🚀 Hugging Face Deployment (Backend)
Hugging Face Spaces is an excellent platform for hosting AI-powered backend services using Docker.

### Prerequisites
1.  A Hugging Face account.
2.  Hugging Face CLI installed (optional, but recommended).
3.  Docker installed for local testing.

### Steps to Deploy
1.  **Create a New Space**:
    - Go to Hugging Face and create a new Space.
    - Select **Docker** as the SDK.
2.  **Configure Environment Variables**:
    - In your Space settings, add the following secrets:
        - `COHERE_API_KEY`: Your Cohere API key.
        - `OPENAI_API_KEY`: Your OpenAI API key.
        - `NEON_DATABASE_URL`: Your Neon Postgres connection string.
        - `QDRANT_API_KEY`: Your Qdrant API key.
        - `QDRANT_ENDPOINT`: Your Qdrant cluster endpoint.
3.  **Push the Code**:
    - Push the `backend` folder content to the Hugging Face Space repository.
    - Ensure your `Dockerfile` is in the root of the Space repository.

---

## 🌐 Frontend Deployment (Docusaurus)
The frontend can be deployed to **Vercel**, **Netlify**, or even as a static Space on Hugging Face.

### Steps for Vercel/Netlify
1.  Connect your GitHub repository to Vercel/Netlify.
2.  Set the `Root Directory` to `website`.
3.  Set the build command: `npm run build`.
4.  Set the output directory: `build`.
5.  Add environment variable:
    - `REACT_APP_API_BASE_URL`: `https://<your-space-name>.hf.space/api/v1`

---

## 🏗️ Docker Configuration
The project includes a `Dockerfile` optimized for Hugging Face Spaces. It uses a non-root user (UID 1000) as required by the platform.

### Running Locally with Docker
```bash
docker build -t robo-ai-backend .
docker run -p 7860:7860 --env-file .env robo-ai-backend
```
Note: Hugging Face Spaces typically use port **7860**.
