import pytest
from unittest.mock import Mock, patch
from src.services.embedding_service import EmbeddingService


@pytest.fixture
def mock_cohere_client():
    mock = Mock()
    mock.embed.return_value = Mock()
    mock.embed.return_value.embeddings = [[0.1, 0.2, 0.3, 0.4]]
    return mock


@pytest.fixture
def embedding_service(mock_cohere_client):
    with patch('src.services.embedding_service.cohere.Client', return_value=mock_cohere_client):
        service = EmbeddingService()
        # Mock the settings to avoid needing actual API keys
        service.client = mock_cohere_client
        return service


def test_generate_embedding(embedding_service):
    """
    Test the generate_embedding method
    """
    text = "Test text for embedding"
    
    result = embedding_service.generate_embedding(text)
    
    # Check that Cohere client was called
    embedding_service.client.embed.assert_called_once()
    
    # Check that the result is a list of floats
    assert isinstance(result, list)
    assert len(result) == 4  # Based on our mock


def test_generate_embedding_for_combined_query_without_selected_text(embedding_service):
    """
    Test the generate_embedding_for_combined_query method without selected text
    """
    query = "Test query"
    selected_text = None
    
    result = embedding_service.generate_embedding_for_combined_query(query, selected_text)
    
    # Check that Cohere client was called with just the query
    embedding_service.client.embed.assert_called_once()
    args, kwargs = embedding_service.client.embed.call_args
    assert query in args[0][0]  # First argument should be the texts list
    
    assert isinstance(result, list)


def test_generate_embedding_for_combined_query_with_selected_text(embedding_service):
    """
    Test the generate_embedding_for_combined_query method with selected text
    """
    query = "Test query"
    selected_text = "Test selected text"
    
    result = embedding_service.generate_embedding_for_combined_query(query, selected_text)
    
    # Check that Cohere client was called with the combined text
    embedding_service.client.embed.assert_called_once()
    args, kwargs = embedding_service.client.embed.call_args
    combined_text = args[0][0]  # First argument should be the texts list
    
    assert query in combined_text
    assert selected_text in combined_text
    
    assert isinstance(result, list)


def test_calculate_similarity():
    """
    Test the calculate_similarity method
    """
    embedding_service = EmbeddingService()
    
    embedding1 = [1.0, 0.0, 0.0]  # Unit vector
    embedding2 = [1.0, 0.0, 0.0]  # Same as embedding1
    
    result = embedding_service.calculate_similarity(embedding1, embedding2)
    
    # Should be 1.0 for identical vectors
    assert result == 1.0
    assert 0.0 <= result <= 1.0


def test_calculate_similarity_different():
    """
    Test the calculate_similarity method with different embeddings
    """
    embedding_service = EmbeddingService()
    
    embedding1 = [1.0, 0.0, 0.0]
    embedding2 = [0.0, 1.0, 0.0]  # Orthogonal to embedding1
    
    result = embedding_service.calculate_similarity(embedding1, embedding2)
    
    # Should be 0.0 for orthogonal vectors
    assert result == 0.0
    assert 0.0 <= result <= 1.0