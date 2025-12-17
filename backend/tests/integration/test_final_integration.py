"""
Final integration test to verify all user stories work together
This test verifies that the components from all user stories integrate properly
"""
import pytest
import os
from fastapi.testclient import TestClient
from unittest.mock import patch, Mock
from pydantic import BaseModel

# Set required environment variables before importing app
os.environ.setdefault('QDRANT_ENDPOINT', 'http://example.com')
os.environ.setdefault('QDRANT_API_KEY', 'fake_key')
os.environ.setdefault('QDRANT_CLIENT_URL', 'http://example.com:6333')
os.environ.setdefault('NEON_DATABASE_URL', 'postgresql://user:password@localhost/db')
os.environ.setdefault('COHERE_API_KEY', 'fake_cohere_key')

from src.api.main import app
from src.services.rag_service import RAGService
from src.services.session_service import SessionService
from src.models.generated_response import GeneratedResponse
from src.models.retrieved_context import RetrievedContext
from src.models.user_query import UserQuery


client = TestClient(app)


def test_complete_user_flow_integration():
    """
    Test that simulates a complete user flow using all user stories:
    1. Basic chat functionality (US1)
    2. Web interface with session management (US2)
    3. Selected text processing (US3)
    """
    # Create mock objects for services
    mock_rag_service = Mock(spec=RAGService)
    mock_response = GeneratedResponse(
        id="response_1",
        content="RAG systems combine retrieval and generation for improved responses.",
        query_id="query_1",
        relevance_score=0.92,
        retrieved_contexts=[
            RetrievedContext(
                id="context_1",
                content="Retrieval-Augmented Generation (RAG) is a technique that combines information retrieval with text generation.",
                relevance_score=0.87,
                source_location="Chapter 3, Section 1",
                query_id="query_1"
            )
        ]
    )
    mock_rag_service.process_query_with_rag.return_value = mock_response

    # Mock the session service to simulate session management
    mock_session_service = Mock(spec=SessionService)
    mock_session_service.validate_session.return_value = True
    mock_session_service.update_session_activity.return_value = True

    # Test 1: Basic chat functionality (User Story 1)
    with patch('src.api.chatbot_router.rag_service', mock_rag_service), \
         patch('src.api.chatbot_router.session_service', mock_session_service):
        
        response = client.post(
            "/api/v1/chat",
            json={
                "query": "What is the main concept behind RAG systems?",
                "session_id": "test_session_123"
            }
        )

        assert response.status_code == 200
        data = response.json()
        assert "response" in data
        assert data["relevance_score"] == 0.92
        assert len(data["retrieved_contexts"]) == 1
        
        # Verify that RAG service was called
        assert mock_rag_service.process_query_with_rag.called

    # Test 2: Selected text functionality (User Story 3)
    with patch('src.api.chatbot_router.rag_service', mock_rag_service), \
         patch('src.api.chatbot_router.session_service', mock_session_service):
        
        response = client.post(
            "/api/v1/chat/selected-text",
            json={
                "query": "Explain this concept in more depth",
                "selected_text": "Retrieval-Augmented Generation (RAG) is a technique that combines information retrieval with text generation.",
                "session_id": "test_session_123"
            }
        )

        assert response.status_code == 200
        data = response.json()
        assert "response" in data
        assert data["relevance_score"] == 0.92
        assert len(data["retrieved_contexts"]) == 1

    # Test 3: Health check endpoint
    response = client.get("/health")
    assert response.status_code == 200
    health_data = response.json()
    assert health_data["status"] == "healthy"
    assert "timestamp" in health_data

    # Verify all services were called as expected during the tests
    assert mock_rag_service.process_query_with_rag.call_count == 2  # Called for both chat endpoints


def test_book_content_indexing_integration():
    """
    Test book content indexing functionality
    """
    with patch('src.api.book_content_router.book_content_service') as mock_book_service:
        mock_book_service.index_book_content.return_value = True
        
        response = client.post(
            "/api/v1/book-content/index",
            json={
                "book_id": "test_book_123",
                "content_blocks": [
                    {
                        "id": "block_1",
                        "content": "This is a test content block.",
                        "vector_embedding": [0.1, 0.2, 0.3],
                        "source_location": "Chapter 1, Page 1",
                        "book_id": "test_book_123"
                    }
                ]
            }
        )
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
        assert "Successfully indexed" in data["message"]
        
        # Verify the book content service was called
        assert mock_book_service.index_book_content.called


def test_error_handling_integration():
    """
    Test error handling across different user stories
    """
    # Mock an exception in the RAG service
    mock_rag_service = Mock(spec=RAGService)
    mock_rag_service.process_query_with_rag.side_effect = Exception("Test error")

    with patch('src.api.chatbot_router.rag_service', mock_rag_service):
        # Test error handling in basic chat endpoint
        response = client.post(
            "/api/v1/chat",
            json={
                "query": "This should fail",
                "session_id": "test_session_error"
            }
        )
        assert response.status_code == 500

        # Test error handling in selected text endpoint
        response = client.post(
            "/api/v1/chat/selected-text",
            json={
                "query": "This should also fail",
                "selected_text": "Some selected text",
                "session_id": "test_session_error"
            }
        )
        assert response.status_code == 500


if __name__ == "__main__":
    # Run the tests
    test_complete_user_flow_integration()
    test_book_content_indexing_integration()
    test_error_handling_integration()
    print("All integration tests passed!")