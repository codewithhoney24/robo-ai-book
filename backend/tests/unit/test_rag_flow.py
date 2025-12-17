import pytest
from unittest.mock import Mock, patch, MagicMock
from src.services.rag_service import RAGService
from src.models.user_query import UserQuery
from src.models.retrieved_context import RetrievedContext


@pytest.fixture
def mock_embedding_service():
    mock = Mock()
    mock.generate_embedding.return_value = [0.1, 0.2, 0.3, 0.4]
    return mock


@pytest.fixture
def mock_vector_store_service():
    mock = Mock()
    mock.search.return_value = [
        RetrievedContext(
            id="test_context_1",
            content="Test content for retrieval",
            relevance_score=0.85,
            source_location="Chapter 1, Section 1",
            query_id="test_query_1"
        )
    ]
    return mock


@pytest.fixture
def rag_service(mock_embedding_service, mock_vector_store_service):
    with patch('src.services.rag_service.embedding_service', mock_embedding_service), \
         patch('src.services.rag_service.vector_store_service', mock_vector_store_service):
        return RAGService()


def test_rag_flow_with_selected_text(rag_service):
    """
    Unit test for RAG flow with user-selected text
    """
    query = UserQuery(
        id="test_query_1",
        content="What is RAG?",
        user_session_id="test_session_123",
        selected_text="Retrieval-Augmented Generation (RAG) is a technique that combines information retrieval with text generation."
    )
    
    result = rag_service.process_query_with_rag(query)
    
    # Verify the result structure
    assert result is not None
    assert hasattr(result, 'content')
    assert hasattr(result, 'relevance_score')
    assert hasattr(result, 'retrieved_contexts')
    
    # Verify vector store was called with the combined query
    rag_service.vector_store_service.search.assert_called_once()
    
    # Check that the search was called with the right parameters
    args, kwargs = rag_service.vector_store_service.search.call_args
    assert "What is RAG?" in args[0]  # Original query
    assert "Retrieval-Augmented Generation (RAG) is a technique" in args[0]  # Selected text


def test_rag_flow_without_selected_text(rag_service):
    """
    Unit test for RAG flow without user-selected text
    """
    query = UserQuery(
        id="test_query_2",
        content="Explain AI",
        user_session_id="test_session_123"
    )
    
    result = rag_service.process_query_with_rag(query)
    
    # Verify the result structure
    assert result is not None
    assert hasattr(result, 'content')
    assert hasattr(result, 'relevance_score')
    assert hasattr(result, 'retrieved_contexts')
    
    # Verify vector store was called
    rag_service.vector_store_service.search.assert_called_once()


def test_rag_flow_empty_query():
    """
    Unit test for RAG flow with empty query
    """
    query = UserQuery(
        id="test_query_3",
        content="",
        user_session_id="test_session_123"
    )
    
    with pytest.raises(ValueError):
        rag_service.process_query_with_rag(query)