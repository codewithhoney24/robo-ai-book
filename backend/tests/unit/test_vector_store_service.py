import pytest
from unittest.mock import Mock, patch
from src.services.vector_store_service import VectorStoreService
from src.models.retrieved_context import RetrievedContext


@pytest.fixture
def mock_qdrant_client():
    mock = Mock()
    return mock


@pytest.fixture
def mock_embedding_service():
    mock = Mock()
    mock.generate_embedding.return_value = [0.1, 0.2, 0.3, 0.4]
    mock.calculate_similarity.return_value = 0.8
    return mock


@pytest.fixture
def vector_store_service(mock_qdrant_client):
    with patch('src.services.vector_store_service.db_config.get_qdrant_client', return_value=mock_qdrant_client):
        return VectorStoreService()


def test_calculate_combined_score():
    """
    Test the combined score calculation
    """
    service = VectorStoreService()
    
    # Test case: original score 0.7, similarity score 0.9
    # Combined should be 0.6 * 0.7 + 0.4 * 0.9 = 0.42 + 0.36 = 0.78
    original_score = 0.7
    similarity_score = 0.9
    expected_combined = (0.6 * original_score) + (0.4 * similarity_score)
    
    result = service._calculate_combined_score(original_score, similarity_score)
    assert result == expected_combined
    assert 0.0 <= result <= 1.0


def test_calculate_combined_score_bounds():
    """
    Test the combined score calculation with boundary values
    """
    service = VectorStoreService()
    
    # Test with minimum values
    result_min = service._calculate_combined_score(0.0, 0.0)
    assert result_min == 0.0
    
    # Test with maximum values
    result_max = service._calculate_combined_score(1.0, 1.0)
    assert result_max == 1.0


def test_prioritize_contexts_for_selected_text():
    """
    Test the prioritization of contexts based on selected text
    """
    service = VectorStoreService()
    
    # Create some test contexts
    contexts = [
        RetrievedContext(
            id="ctx1",
            content="Context one content",
            relevance_score=0.6,
            source_location="Chapter 1",
            query_id="test_query"
        ),
        RetrievedContext(
            id="ctx2",
            content="Context two content",
            relevance_score=0.8,
            source_location="Chapter 2",
            query_id="test_query"
        )
    ]
    
    selected_text = "This is the selected text"
    original_query = "What is this?"
    
    with patch('src.services.vector_store_service.embedding_service') as mock_emb:
        # Mock embedding generation
        mock_emb.generate_embedding.return_value = [0.1, 0.2, 0.3, 0.4]
        mock_emb.calculate_similarity.return_value = 0.7
        
        # This should not raise an exception
        result = service._prioritize_contexts_for_selected_text(contexts, selected_text, original_query)
        
        # Check that results are returned and properly sorted by relevance
        assert len(result) == len(contexts)
        # Results should be sorted in descending order by relevance
        for i in range(len(result) - 1):
            assert result[i].relevance_score >= result[i+1].relevance_score