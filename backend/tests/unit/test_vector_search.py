import pytest
from unittest.mock import Mock, patch
from src.services.vector_store_service import VectorStoreService
from src.models.retrieved_context import RetrievedContext


@pytest.fixture
def mock_qdrant_client():
    mock = Mock()
    # Mock the query_points response (new client API)
    mock.query_points.return_value = Mock(points=[
        Mock(
            id="test_doc_1",
            payload={
                "content": "Test content for retrieval",
                "source_location": "Chapter 1, Section 1"
            },
            score=0.85
        ),
        Mock(
            id="test_doc_2", 
            payload={
                "content": "Another relevant content",
                "source_location": "Chapter 2, Section 3"
            },
            score=0.78
        )
    ])
    return mock


@pytest.fixture
def vector_store_service(mock_qdrant_client):
    with patch('src.services.vector_store_service.db_config') as mock_db_config:
        mock_db_config.get_qdrant_client.return_value = mock_qdrant_client
        return VectorStoreService()


def test_vector_search_basic(vector_store_service):
    """
    Unit test for basic vector search functionality
    """
    query_embedding = [0.1, 0.2, 0.3, 0.4]
    query_text = "What is RAG?"
    limit = 5
    
    results = vector_store_service.search(query_embedding, query_text, limit)
    
    # Verify the results
    assert len(results) == 2
    
    # Verify the structure of results
    for result in results:
        assert isinstance(result, RetrievedContext)
        assert result.content is not None
        assert 0.0 <= result.relevance_score <= 1.0
        assert result.source_location is not None
        assert result.query_id is not None
        
    # Verify that the Qdrant client was called
    vector_store_service.qdrant_client.query_points.assert_called_once()


def test_vector_search_with_filters(vector_store_service):
    """
    Unit test for vector search with filters
    """
    query_embedding = [0.1, 0.2, 0.3, 0.4]
    query_text = "What is RAG?"
    limit = 5
    filters = {"book_id": "book_123"}
    
    results = vector_store_service.search(query_embedding, query_text, limit, filters)
    
    # Verify the results
    assert len(results) == 2
    
    # Verify that the Qdrant client was called with filters
    args, kwargs = vector_store_service.qdrant_client.query_points.call_args
    # Verify that the call included the filters
    assert "query_filter" in kwargs


def test_vector_search_empty_embedding(vector_store_service):
    """
    Unit test for vector search with empty embedding
    """
    query_embedding = []
    query_text = "Empty embedding test"
    limit = 5
    
    with pytest.raises(ValueError):
        vector_store_service.search(query_embedding, query_text, limit)