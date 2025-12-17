import pytest
from fastapi.testclient import TestClient
from src.api.main import app


client = TestClient(app)


def test_book_content_index_endpoint_contract():
    """
    Contract test for /book-content/index endpoint
    """
    # Test valid request
    valid_request = {
        "book_id": "test_book_123",
        "content_blocks": [
            {
                "id": "block_1",
                "content": "This is the first content block of the book.",
                "vector_embedding": [0.1, 0.2, 0.3],
                "source_location": "Chapter 1, Page 1",
                "book_id": "test_book_123"
            }
        ]
    }
    
    response = client.post("/api/v1/book-content/index", json=valid_request)
    
    # Verify status code
    assert response.status_code == 200
    
    # Verify response structure
    response_data = response.json()
    assert "message" in response_data
    assert "success" in response_data
    assert response_data["success"] is True


def test_book_content_index_text_endpoint_contract():
    """
    Contract test for /book-content/index-text endpoint
    """
    # Test valid request
    valid_request = {
        "book_id": "test_book_456",
        "full_text": "This is the full text of the book. It contains multiple sentences and paragraphs that should be split into chunks for indexing.",
        "chunk_size": 500
    }
    
    response = client.post("/api/v1/book-content/index-text", json=valid_request)
    
    # Verify status code
    assert response.status_code == 200
    
    # Verify response structure
    response_data = response.json()
    assert "message" in response_data
    assert "success" in response_data
    assert response_data["success"] is True


def test_book_content_endpoints_missing_required_fields():
    """
    Test book content endpoints with missing required fields
    Should return 422 (validation error)
    """
    # Test /book-content/index without book_id
    invalid_request_1 = {
        "content_blocks": [
            {
                "id": "block_1",
                "content": "This is the first content block of the book.",
                "vector_embedding": [0.1, 0.2, 0.3],
                "source_location": "Chapter 1, Page 1",
                "book_id": "test_book_123"
            }
        ]
    }
    
    response_1 = client.post("/api/v1/book-content/index", json=invalid_request_1)
    assert response_1.status_code == 422

    # Test /book-content/index-text without book_id
    invalid_request_2 = {
        "full_text": "This is the full text of the book.",
    }
    
    response_2 = client.post("/api/v1/book-content/index-text", json=invalid_request_2)
    assert response_2.status_code == 422
    
    # Test /book-content/index-text without full_text
    invalid_request_3 = {
        "book_id": "test_book_789",
    }
    
    response_3 = client.post("/api/v1/book-content/index-text", json=invalid_request_3)
    assert response_3.status_code == 422


def test_health_endpoint_contract():
    """
    Contract test for /health endpoint
    """
    response = client.get("/health")
    
    # Verify status code
    assert response.status_code == 200
    
    # Verify response structure
    response_data = response.json()
    assert "status" in response_data
    assert "timestamp" in response_data
    assert response_data["status"] == "healthy"
    assert isinstance(response_data["timestamp"], str)