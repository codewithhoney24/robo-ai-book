from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
import uuid

from ..services.rag_service import rag_service
from ..models.user_query import UserQuery
from ..models.generated_response import GeneratedResponse

from ..services.session_service import session_service
from ..config.logging_config import api_logger
from ..exceptions import RAGChatbotException, QueryTooLongException, InvalidSessionException
from ..config.settings import settings


router = APIRouter()


@router.post("/chat", response_model=GeneratedResponse)
async def chat_endpoint(query_request: dict):
    """
    Process a natural language query and return a generated response based on book content
    """
    try:
        # Extract and validate the query
        query_text = query_request.get("query", "").strip()
        if not query_text:
            # Check if we have selected_text to use instead (useful when frontend extraction fails)
            selected_text = query_request.get("selected_text", "").strip()
            if selected_text:
                query_text = "Explain this text: " + selected_text
            else:
                raise HTTPException(status_code=422, detail="Query parameter is required and cannot be empty")

        # Check query length
        if len(query_text) > settings.max_query_length:
            raise QueryTooLongException(f"Query exceeds maximum length of {settings.max_query_length} characters")

        # Extract book_id if provided
        book_id = query_request.get("book_id")

        # Handle session ID
        user_provided_session_id = query_request.get("session_id")
        if user_provided_session_id:
            # Validate existing session
            if not session_service.validate_session(user_provided_session_id):
                raise InvalidSessionException("Invalid or expired session ID")
            session_id = user_provided_session_id
        else:
            # Create new session
            session_id = f"session_{uuid.uuid4().hex}"
            user_agent = query_request.get("user_agent")  # Extract user agent if provided
            session_service.create_session(session_id, user_agent)

        # Create a UserQuery object
        user_query = UserQuery(
            id=f"query_{uuid.uuid4().hex}",
            content=query_text,
            user_session_id=session_id,
            selected_text=None  # No selected text for the basic chat endpoint
        )

        # Process the query using RAG with optional book_id filtering
        response = rag_service.process_query_with_rag(user_query, book_id=book_id)

        # Update session activity
        session_service.update_session_activity(session_id)

        # Log successful request
        api_logger.info(f"Processed chat query for session {session_id}")

        return response

    except QueryTooLongException as e:
        api_logger.warning(f"Query too long: {e.message}")
        raise HTTPException(status_code=400, detail={"error": e.error_code, "message": e.message})

    except InvalidSessionException as e:
        api_logger.warning(f"Invalid session: {e.message}")
        raise HTTPException(status_code=400, detail={"error": e.error_code, "message": e.message})

    except RAGChatbotException as e:
        api_logger.error(f"RAG chatbot error: {e.message}")
        raise HTTPException(status_code=400, detail={"error": e.error_code, "message": e.message})

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise

    except Exception as e:
        api_logger.error(f"Unexpected error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail={
            "error": "INTERNAL_ERROR",
            "message": "An unexpected error occurred while processing your query"
        })


@router.post("/chat/selected-text", response_model=GeneratedResponse)
async def chat_selected_text_endpoint(query_request: dict):
    """
    Process a query specifically related to user-selected text within the book
    """
    try:
        # Extract and validate the query
        query_text = query_request.get("query", "").strip()
        selected_text = query_request.get("selected_text", "").strip()

        if not query_text:
            raise HTTPException(status_code=422, detail="Query parameter is required and cannot be empty")

        if not selected_text:
            raise HTTPException(status_code=422, detail="Selected text parameter is required and cannot be empty")

        # Check query length
        if len(query_text) > settings.max_query_length:
            raise QueryTooLongException(f"Query exceeds maximum length of {settings.max_query_length} characters")

        # Extract book_id if provided
        book_id = query_request.get("book_id")

        # Handle session ID
        user_provided_session_id = query_request.get("session_id")
        if user_provided_session_id:
            # Validate existing session
            if not session_service.validate_session(user_provided_session_id):
                raise InvalidSessionException("Invalid or expired session ID")
            session_id = user_provided_session_id
        else:
            # Create new session
            session_id = f"session_{uuid.uuid4().hex}"
            user_agent = query_request.get("user_agent")  # Extract user agent if provided
            session_service.create_session(session_id, user_agent)

        # Create a UserQuery object with selected text
        user_query = UserQuery(
            id=f"query_{uuid.uuid4().hex}",
            content=query_text,
            user_session_id=session_id,
            selected_text=selected_text
        )

        # Process the query using RAG with optional book_id filtering
        response = rag_service.process_query_with_rag(user_query, book_id=book_id)

        # Update session activity
        session_service.update_session_activity(session_id)

        # Log successful request
        api_logger.info(f"Processed selected-text chat query for session {session_id}")

        return response

    except QueryTooLongException as e:
        api_logger.warning(f"Query too long: {e.message}")
        raise HTTPException(status_code=400, detail={"error": e.error_code, "message": e.message})

    except InvalidSessionException as e:
        api_logger.warning(f"Invalid session: {e.message}")
        raise HTTPException(status_code=400, detail={"error": e.error_code, "message": e.message})

    except RAGChatbotException as e:
        api_logger.error(f"RAG chatbot error: {e.message}")
        raise HTTPException(status_code=400, detail={"error": e.error_code, "message": e.message})

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise

    except Exception as e:
        api_logger.error(f"Unexpected error in chat selected text endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail={
            "error": "INTERNAL_ERROR",
            "message": "An unexpected error occurred while processing your query"
        })


@router.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "timestamp": "2025-12-14T10:00:00Z"}