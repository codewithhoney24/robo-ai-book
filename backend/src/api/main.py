from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import RedirectResponse, JSONResponse

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
from src.api.v1.logs import router as logs_router
from src.api.v1.auth_better import router as better_auth_router
from src.api.v1.personalization import router as personalization_router
from src.api.v1.email import router as email_router

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
app.include_router(logs_router, prefix="/api/v1", tags=["logs"])
app.include_router(better_auth_router, prefix="/api/v1", tags=["auth"])  # BetterAuth-like routes with background collection
app.include_router(personalization_router, prefix="/api/v1", tags=["personalization"])
app.include_router(email_router, prefix="/api/v1", tags=["email"])


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


# =====================================================
# GLOBAL EXCEPTION HANDLERS
# =====================================================
@app.exception_handler(404)
async def not_found_handler(request: Request, exc):
    """
    Global handler for 404 Not Found errors.
    """
    from src.config.logging_config import api_logger
    api_logger.warning(f"404 error: {request.method} {request.url}")
    return JSONResponse(
        status_code=404,
        content={
            "error": "NOT_FOUND",
            "message": "The requested resource was not found",
            "path": str(request.url),
            "timestamp": datetime.utcnow().isoformat()
        }
    )


@app.exception_handler(422)
async def unprocessable_entity_handler(request: Request, exc):
    """
    Global handler for 422 Unprocessable Entity errors.
    This handles validation errors from Pydantic models.
    """
    from src.config.logging_config import api_logger
    api_logger.warning(f"422 error: {request.method} {request.url} - Validation error: {exc}")

    # Format the validation errors to return to the client
    errors = []
    if hasattr(exc, 'errors'):
        for error in exc.errors():
            errors.append({
                "loc": error.get("loc", []),
                "msg": error.get("msg", "Validation error"),
                "type": error.get("type", "validation_error")
            })

    return JSONResponse(
        status_code=422,
        content={
            "error": "VALIDATION_ERROR",
            "message": "The request contains invalid data",
            "details": errors,
            "path": str(request.url),
            "timestamp": datetime.utcnow().isoformat()
        }
    )


@app.exception_handler(500)
async def internal_error_handler(request: Request, exc):
    """
    Global handler for 500 Internal Server errors.
    """
    from src.config.logging_config import api_logger
    api_logger.error(f"500 error: {request.method} {request.url} - {str(exc)}")
    return JSONResponse(
        status_code=500,
        content={
            "error": "INTERNAL_ERROR",
            "message": "An internal server error occurred",
            "path": str(request.url),
            "timestamp": datetime.utcnow().isoformat()
        }
    )


@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """
    General exception handler for unhandled exceptions.
    """
    from src.config.logging_config import api_logger
    api_logger.error(f"Unhandled exception: {request.method} {request.url} - {str(exc)}")
    return JSONResponse(
        status_code=500,
        content={
            "error": "UNHANDLED_EXCEPTION",
            "message": "An unexpected error occurred",
            "path": str(request.url),
            "timestamp": datetime.utcnow().isoformat()
        }
    )
