import pytest
from fastapi.testclient import TestClient
from src.api.main import app
from src.models.user_query import UserQuery
from src.models.generated_response import GeneratedResponse


client = TestClient(app)


def test_chat_endpoint_contract():
    """
    Contract test for /chat endpoint
    Verifies that the endpoint accepts the correct request format and returns expected response format
    """
    # Test valid request
    valid_request = {
        "query": "What is the main concept behind RAG systems?",
        "session_id": "test_session_123"
    }
    
    response = client.post("/api/v1/chat", json=valid_request)
    
    # Verify status code
    assert response.status_code == 200
    
    # Verify response structure
    response_data = response.json()
    assert "response" in response_data
    assert "relevance_score" in response_data
    assert "retrieved_contexts" in response_data
    
    # Verify data types
    assert isinstance(response_data["response"], str)
    assert isinstance(response_data["relevance_score"], float)
    assert isinstance(response_data["retrieved_contexts"], list)
    
    # Verify relevance score is within expected range
    assert 0.0 <= response_data["relevance_score"] <= 1.0


def test_chat_endpoint_missing_query():
    """
    Test /chat endpoint with missing query field
    Should return 422 (validation error)
    """
    invalid_request = {
        "session_id": "test_session_123"
    }
    
    response = client.post("/api/v1/chat", json=invalid_request)
    
    assert response.status_code == 422  # Validation error


def test_chat_endpoint_empty_query():
    """
    Test /chat endpoint with empty query
    Should return 422 (validation error)
    """
    invalid_request = {
        "query": "",
        "session_id": "test_session_123"
    }
    
    response = client.post("/api/v1/chat", json=invalid_request)
    
    assert response.status_code == 422  # Validation error