import pytest
from fastapi.testclient import TestClient
from src.api.main import app


client = TestClient(app)


def test_selected_text_endpoint_contract():
    """
    Contract test for /chat/selected-text endpoint
    Verifies that the endpoint accepts the correct request format and returns expected response format
    """
    # Test valid request with selected text
    valid_request = {
        "query": "Explain this concept more deeply",
        "selected_text": "Retrieval-Augmented Generation (RAG) is a technique that combines information retrieval with text generation.",
        "session_id": "test_session_456"
    }
    
    response = client.post("/api/v1/chat/selected-text", json=valid_request)
    
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


def test_selected_text_endpoint_missing_fields():
    """
    Test /chat/selected-text endpoint with missing required fields
    Should return 422 (validation error)
    """
    # Test without query
    invalid_request_1 = {
        "selected_text": "Sample selected text",
        "session_id": "test_session_789"
    }
    
    response_1 = client.post("/api/v1/chat/selected-text", json=invalid_request_1)
    assert response_1.status_code == 422

    # Test without selected_text
    invalid_request_2 = {
        "query": "What does this mean?",
        "session_id": "test_session_789"
    }
    
    response_2 = client.post("/api/v1/chat/selected-text", json=invalid_request_2)
    assert response_2.status_code == 422


def test_selected_text_endpoint_empty_fields():
    """
    Test /chat/selected-text endpoint with empty required fields
    Should return 422 (validation error)
    """
    # Test with empty query
    invalid_request_1 = {
        "query": "",
        "selected_text": "Sample selected text",
        "session_id": "test_session_789"
    }
    
    response_1 = client.post("/api/v1/chat/selected-text", json=invalid_request_1)
    assert response_1.status_code == 422

    # Test with empty selected_text
    invalid_request_2 = {
        "query": "What does this mean?",
        "selected_text": "",
        "session_id": "test_session_789"
    }
    
    response_2 = client.post("/api/v1/chat/selected-text", json=invalid_request_2)
    assert response_2.status_code == 422