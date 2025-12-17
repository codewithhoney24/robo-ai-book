import pytest
from unittest.mock import Mock, patch
from src.services.rag_service import rag_service
from src.models.user_query import UserQuery


@pytest.fixture
def mock_services():
    with patch('src.services.rag_service.embedding_service') as mock_embedding, \
         patch('src.services.rag_service.vector_store_service') as mock_vector_store, \
         patch('src.services.rag_service.database_service') as mock_database:
        
        # Mock embedding service
        mock_embedding.generate_embedding.return_value = [0.1, 0.2, 0.3, 0.4]
        
        # Mock vector store service
        from src.models.retrieved_context import RetrievedContext
        mock_vector_store.search.return_value = [
            RetrievedContext(
                id="context_1",
                content="Relevant content for the selected text",
                relevance_score=0.85,
                source_location="Chapter 3, Section 2",
                query_id="test_query"
            )
        ]
        
        # Mock database service
        mock_database.update_session_activity.return_value = True
        mock_database.create_user_query.return_value = True
        
        yield {
            'embedding': mock_embedding,
            'vector_store': mock_vector_store,
            'database': mock_database
        }


def test_process_query_with_selected_text(mock_services):
    """
    Unit test for processing a query with selected text
    """
    # Create a user query with selected text
    user_query = UserQuery(
        id="test_query_1",
        content="Explain this concept more deeply",
        user_session_id="test_session_123",
        selected_text="Retrieval-Augmented Generation (RAG) is a technique that combines information retrieval with text generation."
    )
    
    # Process the query
    result = rag_service.process_query_with_rag(user_query)
    
    # Verify the result
    assert result is not None
    assert "Explain this concept more deeply" in result.content
    assert len(result.retrieved_contexts) > 0
    
    # Verify that the vector store was called with the combined query
    args, kwargs = mock_services['vector_store'].search.call_args
    combined_query = args[0]  # This should be the query embedding
    
    # The embedding service should have been called with the combined text
    mock_services['embedding'].generate_embedding.assert_called_once()
    # Verify the call was made with combined query and selected text
    call_args = mock_services['embedding'].generate_embedding.call_args[0][0]
    assert "Explain this concept more deeply" in call_args
    assert "Retrieval-Augmented Generation (RAG) is a technique" in call_args


def test_process_query_without_selected_text(mock_services):
    """
    Unit test for processing a query without selected text
    """
    # Create a user query without selected text
    user_query = UserQuery(
        id="test_query_2",
        content="What is RAG?",
        user_session_id="test_session_123",
        selected_text=None
    )
    
    # Process the query
    result = rag_service.process_query_with_rag(user_query)
    
    # Verify the result
    assert result is not None
    assert len(result.retrieved_contexts) > 0
    
    # Verify that the vector store was called appropriately
    mock_services['vector_store'].search.assert_called_once()


def test_process_query_empty_selected_text(mock_services):
    """
    Unit test for processing a query with empty selected text
    Should be treated as if no selected text was provided
    """
    # Create a user query with empty selected text
    user_query = UserQuery(
        id="test_query_3",
        content="What is RAG?",
        user_session_id="test_session_123",
        selected_text=""
    )
    
    # Process the query
    result = rag_service.process_query_with_rag(user_query)
    
    # Verify the result
    assert result is not None
    assert len(result.retrieved_contexts) > 0


def test_process_query_long_selected_text(mock_services):
    """
    Unit test for processing a query with very long selected text
    Should handle gracefully without performance issues
    """
    # Create a long selected text (simulating a paragraph or more)
    long_selected_text = "Artificial intelligence (AI) is intelligence demonstrated by machines, in contrast to the natural intelligence displayed by humans and animals. Leading AI textbooks define the field as the study of 'intelligent agents': any device that perceives its environment and takes actions that maximize its chance of successfully achieving its goals. Colloquially, the term 'AI' is often used to describe machines that mimic 'cognitive' functions that humans associate with the human mind, such as 'learning' and 'problem solving'. " * 5  # Repeat to make it longer

    user_query = UserQuery(
        id="test_query_4",
        content="Summarize this section",
        user_session_id="test_session_123",
        selected_text=long_selected_text
    )
    
    # Process the query
    result = rag_service.process_query_with_rag(user_query)
    
    # Verify the result
    assert result is not None
    assert len(result.retrieved_contexts) > 0


def test_rag_service_handles_special_characters_in_selected_text(mock_services):
    """
    Unit test to ensure special characters in selected text are handled properly
    """
    special_char_text = "The algorithm uses \"special\" characters & symbols like @#$%^*()_+."
    
    user_query = UserQuery(
        id="test_query_5",
        content="Explain the special characters",
        user_session_id="test_session_123",
        selected_text=special_char_text
    )
    
    # Process the query
    result = rag_service.process_query_with_rag(user_query)
    
    # Verify the result
    assert result is not None
    assert len(result.retrieved_contexts) > 0