from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import RedirectResponse

from typing import List
from datetime import datetime

import cohere

# Routers
from src.api.chatbot_router import router as chatbot_router
from src.api.book_content_router import router as book_content_router
from src.api.v1.health import router as health_router
from src.api.v1.metrics import router as metrics_router
from src.api.v1.database import router as database_router
from src.api.v1.chat import router as chat_v1_router
from src.api.v1.translation import router as translation_router
from src.api.v1.auth import router as auth_router

# Settings
from src.config.settings import settings

# Middleware
from src.api.middleware.compatibility_check import compatibility_check_middleware


# =====================================================
# APP INIT (old-style simple)
# =====================================================
app = FastAPI(
    title=settings.app_name,
    version=settings.version,
    description="API for interacting with the RAG chatbot embedded in published books",
    debug=settings.debug,
)


# =====================================================
# MIDDLEWARE
# =====================================================
app.add_middleware(GZipMiddleware, minimum_size=1000)
app.middleware("http")(compatibility_check_middleware)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # restrict in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Added for CORS preflight requests
    allow_origin_regex="https?://.*",
)

# app.add_middleware(
#     CORSMiddleware,
#     allow_origins="http://localhost:3000",  # allow only local frontend in development
#     allow_credentials=True,
#     allow_methods=["*"],
#     allow_headers=["*"],
# )

app.add_middleware(
    TrustedHostMiddleware,
    allowed_hosts=["*"],  # restrict in production
)


# =====================================================
# COHERE CLIENT (single instance)
# =====================================================
cohere_client = cohere.Client(settings.cohere_api_key)


# =====================================================
# TEXT CHUNKING (INLINE – like old code)
# =====================================================
def chunk_text(
    text: str,
    max_chars: int = 1200,
    overlap: int = 150
) -> List[str]:
    """
    Split text into overlapping chunks for RAG
    """
    if not text:
        return []

    chunks = []
    start = 0
    length = len(text)

    while start < length:
        end = start + max_chars

        if end < length:
            split_pos = text.rfind(". ", start, end)
            if split_pos != -1:
                end = split_pos + 1

        chunk = text[start:end].strip()
        if chunk:
            chunks.append(chunk)

        start = end - overlap
        if start < 0:
            start = 0

    return chunks


# =====================================================
# EMBEDDINGS (INLINE)
# =====================================================
def embed_document(text: str) -> List[float]:
    """
    Embedding for documents / chunks
    """
    response = cohere_client.embed(
        model=settings.cohere_embed_model,
        input_type="search_document",
        texts=[text],
    )
    return response.embeddings[0]


def embed_query(text: str) -> List[float]:
    """
    Embedding for user search queries
    """
    response = cohere_client.embed(
        model=settings.cohere_embed_model,
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]


# =====================================================
# ROUTERS
# =====================================================
app.include_router(chatbot_router, prefix="/api/v1", tags=["chatbot"])
app.include_router(book_content_router, prefix="/api/v1", tags=["book-content"])
app.include_router(health_router, prefix="/api/v1", tags=["health"])
app.include_router(metrics_router, prefix="/api/v1", tags=["metrics"])
app.include_router(database_router, prefix="/api/v1", tags=["database"])
app.include_router(chat_v1_router, prefix="/api/v1", tags=["chat"])
app.include_router(translation_router, prefix="/api/v1", tags=["translation"])
app.include_router(auth_router, prefix="/api/v1", tags=["auth"])


# =====================================================
# BASIC ENDPOINTS
# =====================================================
@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "app": settings.app_name,
        "version": settings.version,
        "timestamp": datetime.utcnow().isoformat(),
    }


@app.get("/")
async def root():
    return RedirectResponse(url="/docs")
