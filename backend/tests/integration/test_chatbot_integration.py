import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, Mock
from src.api.main import app
from src.services.rag_service import RAGService
from src.models.generated_response import GeneratedResponse
from src.models.retrieved_context import RetrievedContext


client = TestClient(app)


@pytest.fixture
def mock_rag_service():
    mock = Mock(spec=RAGService)
    mock.process_query_with_rag.return_value = GeneratedResponse(
        id="response_1",
        content="This is a test response based on the book content.",
        query_id="query_1",
        relevance_score=0.92,
        retrieved_contexts=[
            RetrievedContext(
                id="context_1",
                content="Relevant content from the book",
                relevance_score=0.85,
                source_location="Chapter 1, Page 10",
                query_id="query_1"
            )
        ]
    )
    return mock


def test_chatbot_integration_with_mocked_rag(mock_rag_service):
    """
    Integration test for chatbot API with mocked RAG service
    """
    # Mock the RAG service in the router
    with patch('src.api.chatbot_router.rag_service', mock_rag_service):
        response = client.post(
            "/api/v1/chat",
            json={
                "query": "What is the main concept behind RAG systems?",
                "session_id": "test_session_123"
            }
        )
        
        # Verify response
        assert response.status_code == 200
        data = response.json()
        
        # Verify structure
        assert "response" in data
        assert "relevance_score" in data
        assert "retrieved_contexts" in data
        
        # Verify content
        assert data["response"] == "This is a test response based on the book content."
        assert data["relevance_score"] == 0.92
        assert len(data["retrieved_contexts"]) == 1
        assert data["retrieved_contexts"][0]["content"] == "Relevant content from the book"
        
        # Verify RAG service was called
        mock_rag_service.process_query_with_rag.assert_called_once()


def test_chatbot_integration_with_selected_text(mock_rag_service):
    """
    Integration test for chatbot API with selected text
    """
    with patch('src.api.chatbot_router.rag_service', mock_rag_service):
        response = client.post(
            "/api/v1/chat/selected-text",
            json={
                "query": "Explain this concept more deeply",
                "selected_text": "Retrieval-Augmented Generation (RAG) is a technique that combines information retrieval with text generation.",
                "session_id": "test_session_123"
            }
        )
        
        # Verify response
        assert response.status_code == 200
        data = response.json()
        
        # Verify structure
        assert "response" in data
        assert "relevance_score" in data
        assert "retrieved_contexts" in data
        
        # Verify content
        assert data["response"] == "This is a test response based on the book content."
        
        # Verify RAG service was called
        mock_rag_service.process_query_with_rag.assert_called_once()


def test_chatbot_api_error_handling(mock_rag_service):
    """
    Integration test for chatbot API error handling
    """
    # Mock an exception in the RAG service
    mock_rag_service.process_query_with_rag.side_effect = Exception("Test error")
    
    with patch('src.api.chatbot_router.rag_service', mock_rag_service):
        response = client.post(
            "/api/v1/chat",
            json={
                "query": "What is the main concept behind RAG systems?",
                "session_id": "test_session_123"
            }
        )
        
        # Should return a 500 error
        assert response.status_code == 500