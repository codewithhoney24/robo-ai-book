# Architecture of Physical AI & Humanoid Robotics Platform

This document describes the high-level architecture and directory structure of the project.

## 🚀 Overview
The project is a comprehensive educational platform for Physical AI and Humanoid Robotics. It combines a dynamic textbook (Docusaurus) with an interactive RAG-powered chatbot (FastAPI + Qdrant) and personalization features.

## 🏗️ High-Level Architecture
The system is divided into two main parts:

1.  **Frontend (Docusaurus/React):** A modern documentation site that provides the textbook content, user authentication, and an interactive chatbot widget.
2.  **Backend (FastAPI):** A robust API service handling RAG (Retrieval Augmented Generation), user personalization, translation services, and database management.

---

## 📂 Directory Structure

### 🌐 Frontend (`/website`)
Built with Docusaurus and React/TypeScript.
- `src/components`: UI components (ChatbotWidget, HomepageFeatures, PersonalizationCards, etc.).
- `src/contexts`: React Contexts for state management (e.g., `PersonalizationContext`).
- `src/pages`: Landing page, Sign Up, Sign In, and other custom pages.
- `docs`: The core textbook content in Markdown format.
- `src/theme`: Overridden Docusaurus components for custom look and feel.

### ⚙️ Backend (`/backend`)
Built with FastAPI (Python).
- `src/api`: REST API endpoints and routers.
- `src/services`: Business logic (RAG service, Embedding service, Translation, Session management).
- `src/models`: Pydantic models for data validation and database schemas.
- `src/database`: Database connection logic and repository patterns (Neon Postgres + Qdrant).
- `src/core`: Core configurations, exceptions, and security middleware.

### 📚 Course Content (`/docs`)
Contains the Markdown files for the modules:
- **Module 1:** ROS 2 Fundamentals
- **Module 2:** Digital Twin & Simulation
- **Module 3:** AI-Robot Brain (NVIDIA Isaac)
- **Module 4:** VLA & Humanoid Robotics

---

## 🤖 Core Features Architecture

### RAG Chatbot
1.  **Retrieval:** When a user asks a question, the backend generates an embedding of the query using **Cohere** or **OpenAI**.
2.  **Search:** It searches for relevant textbook chunks in **Qdrant** (Vector Store).
3.  **Generation:** It synthesizes a response based on the retrieved context. (Includes a fallback keyword-search mechanism for high reliability).

### Personalization System
- Users provide their "Software Background" and "Hardware Availability".
- The frontend passes these preferences to the backend.
- The content and chatbot responses are dynamically adapted to match the user's expertise level.

### Urdu Translation
- A dedicated service that provides on-the-fly translation of chatbot responses and course summaries into Urdu, making the content accessible to a wider audience.

---

## 🛠️ Tech Stack
- **Frontend:** React, Docusaurus, TypeScript, CSS Modules.
- **Backend:** FastAPI, Pydantic, Uvicorn.
- **AI/ML:** Cohere (Embeddings), OpenAI (LLM), Whisper (Speech).
- **Database:** Neon Postgres (Relational), Qdrant (Vector).
- **Deployment:** Railway (Production), Docker.
