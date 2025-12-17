from fastapi import APIRouter, Depends, HTTPException, status, Request
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
import time
import asyncio

from src.database.connection import get_db_session
from src.services.connection_pool import connection_pool_service
from src.services.error_handler import error_handler_service
from src.services.config_manager import config_manager_service
from src.services.connection_monitor import performance_monitor
from src.core.exceptions import DatabaseConnectionError, PerformanceThresholdExceededError


router = APIRouter(prefix="/chat", tags=["chat"])


class ChatQueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=2000, description="User's query to the chatbot")
    session_id: Optional[str] = Field(None, description="Optional session identifier")


class ChatQueryResponse(BaseModel):
    response: str = Field(..., description="The chatbot's response to the query")
    metadata: Dict[str, Any] = Field(..., description="Additional metadata about the response")


@router.post("/query", response_model=ChatQueryResponse)
async def chat_query(request: ChatQueryRequest) -> ChatQueryResponse:
    """
    Submit a query to the RAG chatbot which will access the textbook content.
    
    This endpoint implements the requirements from the spec:
    - Handles multiple concurrent users accessing textbook content
    - Maintains performance under 200ms for 95% of requests
    - Provides graceful degradation when connection limits are reached
    """
    start_time = time.time()
    
    # Validate query length
    max_query_length = config_manager_service.settings.max_query_length
    if len(request.query) > max_query_length:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Query exceeds maximum length of {max_query_length} characters"
        )
    
    try:
        # Execute query with error handling and fallback
        result = await error_handler_service.execute_with_fallback(
            primary_func=lambda: _execute_chat_query(request),
            fallback_func=lambda: _get_fallback_response(request.query)
        )
        
        # Add response time to performance monitoring
        response_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        performance_monitor.add_response_time(
            response_time, 
            endpoint="chat_query"
        )
        
        # Return the result
        return ChatQueryResponse(
            response=result.get("response", "I'm sorry, I couldn't process your request"),
            metadata={
                "response_time": response_time,
                "source_documents": result.get("source_documents", []),
                "session_id": request.session_id
            }
        )
    
    except PerformanceThresholdExceededError as e:
        # This shouldn't normally happen in this endpoint since we're monitoring after the call
        # But include for completeness
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Service temporarily unavailable due to performance issues"
        )
    except Exception as e:
        # Log the error
        from src.database.utils import db_logger
        db_logger.log_error(e, context="chat_query endpoint")
        
        # Check if we're in a degraded state
        if connection_pool_service.is_degraded():
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Service temporarily unavailable due to high load"
            )
        elif connection_pool_service.is_unavailable():
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Database temporarily unavailable"
            )
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="An unexpected error occurred"
            )


async def _execute_chat_query(request: ChatQueryRequest) -> Dict[str, Any]:
    """
    Internal function to execute the chat query with database access.
    """
    # This simulates the RAG functionality
    # In a real implementation, this would:
    # 1. Query the vector database for relevant textbook content
    # 2. Pass that content to an LLM to generate a response
    # 3. Return the response with source document references
    
    # For this implementation, we'll simulate a database operation
    # that might take some time
    await asyncio.sleep(0.1)  # Simulate processing time
    
    # Simulate retrieving information from textbook content
    simulated_response = f"I found some information regarding '{request.query[:50]}...'. " \
                        f"This is a simulated response for the RAG chatbot feature. " \
                        f"In a real implementation, this would connect to your textbook database."
    
    return {
        "response": simulated_response,
        "source_documents": ["textbook_chapter_1.pdf", "textbook_chapter_3.pdf"]
    }


async def _get_fallback_response(query: str) -> Dict[str, Any]:
    """
    Fallback response function when primary service is unavailable.
    """
    # In a real implementation, this might return cached responses
    # or a default message without requiring database access
    return {
        "response": "I'm currently experiencing high demand. Please try again in a moment. " +
                   f"Original query was about: {query[:50]}...",
        "source_documents": []
    }


@router.get("/health")
async def chat_health_check() -> Dict[str, Any]:
    """
    Health check endpoint specifically for the chat functionality.
    """
    try:
        # Get connection pool status
        pool_status = await connection_pool_service.get_pool_status()
        
        # Get performance status
        perf_status = performance_monitor.get_current_stats()
        
        # Check if performance is within thresholds
        is_healthy = (
            pool_status["status"] in ["active", "warming_up"] and
            perf_status["threshold_exceeded_percentage"] < 5  # Less than 5% threshold exceeded
        )
        
        return {
            "status": "healthy" if is_healthy else "degraded",
            "pool_status": pool_status,
            "performance": perf_status,
            "timestamp": time.time()
        }
    except Exception as e:
        from src.database.utils import db_logger
        db_logger.log_error(e, context="chat_health_check")
        return {
            "status": "unhealthy",
            "error": str(e),
            "timestamp": time.time()
        }