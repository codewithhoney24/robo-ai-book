import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch
from src.api.main import app
from src.models.generated_response import GeneratedResponse
from src.models.retrieved_context import RetrievedContext


client = TestClient(app)


@pytest.fixture
def mock_rag_service():
    with patch('src.api.chatbot_router.rag_service') as mock:
        mock_response = GeneratedResponse(
            id="response_1",
            content="Based on the selected text, RAG systems combine retrieval and generation for improved responses.",
            query_id="query_1",
            relevance_score=0.91,
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
        mock.process_query_with_rag.return_value = mock_response
        yield mock


def test_selected_text_integration_happy_path(mock_rag_service):
    """
    Integration test for selected text functionality - happy path
    """
    selected_text = "Retrieval-Augmented Generation (RAG) is a technique that combines information retrieval with text generation to provide more accurate and factual responses."
    
    response = client.post(
        "/api/v1/chat/selected-text",
        json={
            "query": "Explain this concept in more depth",
            "selected_text": selected_text,
            "session_id": "test_session_123"
        }
    )
    
    assert response.status_code == 200
    data = response.json()
    assert data["response"] == "Based on the selected text, RAG systems combine retrieval and generation for improved responses."
    assert data["relevance_score"] == 0.91
    assert len(data["retrieved_contexts"]) == 1
    assert mock_rag_service.process_query_with_rag.called


def test_selected_text_integration_without_session(mock_rag_service):
    """
    Integration test for selected text functionality without providing a session
    Should create a new session automatically
    """
    selected_text = "Machine learning is a subset of artificial intelligence that focuses on algorithms."
    
    response = client.post(
        "/api/v1/chat/selected-text",
        json={
            "query": "What does this mean?",
            "selected_text": selected_text
        }
    )
    
    assert response.status_code == 200
    data = response.json()
    assert "response" in data
    assert "relevance_score" in data
    assert "retrieved_contexts" in data


def test_selected_text_with_special_characters(mock_rag_service):
    """
    Integration test for selected text with special characters
    """
    selected_text = "The algorithm uses \"quoted\" text & special symbols like @#$%^*."
    
    response = client.post(
        "/api/v1/chat/selected-text",
        json={
            "query": "Explain the special elements",
            "selected_text": selected_text,
            "session_id": "test_session_456"
        }
    )
    
    assert response.status_code == 200
    data = response.json()
    assert "response" in data
    assert "relevance_score" in data
    assert "retrieved_contexts" in data


def test_selected_text_with_code_snippet(mock_rag_service):
    """
    Integration test for selected text containing code-like snippets
    """
    selected_text = "The function is defined as:\n\n```python\ndef rag_system(query, documents):\n    retrieved = retrieve(query, documents)\n    response = generate(query, retrieved)\n    return response\n```"
    
    response = client.post(
        "/api/v1/chat/selected-text",
        json={
            "query": "How does this function work?",
            "selected_text": selected_text,
            "session_id": "test_session_789"
        }
    )
    
    assert response.status_code == 200
    data = response.json()
    assert "response" in data
    assert "relevance_score" in data
    assert "retrieved_contexts" in data


def test_selected_text_multiple_paragraphs(mock_rag_service):
    """
    Integration test for selected text with multiple paragraphs
    """
    selected_text = """The first paragraph explains the basics of RAG systems.

The second paragraph delves into the technical implementation details. This might include information about vector embeddings, similarity search, and how the generation model is conditioned on the retrieved context.

The third paragraph discusses practical applications and benefits of using RAG systems over traditional language models."""
    
    response = client.post(
        "/api/v1/chat/selected-text",
        json={
            "query": "Summarize the concepts mentioned",
            "selected_text": selected_text,
            "session_id": "test_session_000"
        }
    )
    
    assert response.status_code == 200
    data = response.json()
    assert "response" in data
    assert "relevance_score" in data
    assert "retrieved_contexts" in data


def test_selected_text_error_handling(mock_rag_service):
    """
    Integration test to verify proper error handling in selected text endpoint
    """
    # Mock an exception in the RAG service
    mock_rag_service.process_query_with_rag.side_effect = Exception("Test error")
    
    response = client.post(
        "/api/v1/chat/selected-text",
        json={
            "query": "This should fail",
            "selected_text": "Some selected text",
            "session_id": "test_session_error"
        }
    )
    
    # Should return a 500 error
    assert response.status_code == 500