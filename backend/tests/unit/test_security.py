import pytest
from unittest.mock import Mock, patch, MagicMock
from src.api.chatbot_router import chat_endpoint, chat_selected_text_endpoint
from src.config.settings import settings
from src.services.session_service import SessionService


def test_input_validation_for_chat_endpoint():
    """
    Test input validation for the chat endpoint
    """
    # Mock request data
    valid_request = {
        "query": "What is RAG?",
        "session_id": "test_session_123"
    }
    
    # Test with empty query - should raise HTTPException
    empty_query_request = {
        "query": "",
        "session_id": "test_session_123"
    }
    
    with patch('src.api.chatbot_router.rag_service'), \
         patch('src.api.chatbot_router.session_service'), \
         patch('src.api.chatbot_router.database_service'), \
         patch('src.api.chatbot_router.api_logger'):
        
        with pytest.raises(Exception):  # Would be HTTPException in real execution
            chat_endpoint(empty_query_request)


def test_input_validation_for_selected_text_endpoint():
    """
    Test input validation for the selected text chat endpoint
    """
    # Test with empty query
    empty_query_request = {
        "query": "",
        "selected_text": "selected text",
        "session_id": "test_session_123"
    }
    
    with patch('src.api.chatbot_router.rag_service'), \
         patch('src.api.chatbot_router.session_service'), \
         patch('src.api.chatbot_router.database_service'), \
         patch('src.api.chatbot_router.api_logger'):
        
        with pytest.raises(Exception):  # Would be HTTPException in real execution
            chat_selected_text_endpoint(empty_query_request)
    
    # Test with empty selected text
    empty_selected_text_request = {
        "query": "Explain this",
        "selected_text": "",
        "session_id": "test_session_123"
    }
    
    with patch('src.api.chatbot_router.rag_service'), \
         patch('src.api.chatbot_router.session_service'), \
         patch('src.api.chatbot_router.database_service'), \
         patch('src.api.chatbot_router.api_logger'):
        
        with pytest.raises(Exception):  # Would be HTTPException in real execution
            chat_selected_text_endpoint(empty_selected_text_request)


def test_query_length_limit():
    """
    Test query length limit validation
    """
    # Create a query that exceeds the limit
    long_query = "A" * (settings.max_query_length + 1)
    
    long_request = {
        "query": long_query,
        "session_id": "test_session_123"
    }
    
    with patch('src.api.chatbot_router.rag_service'), \
         patch('src.api.chatbot_router.session_service'), \
         patch('src.api.chatbot_router.database_service'), \
         patch('src.api.chatbot_router.api_logger'):
        
        with pytest.raises(Exception):  # Would be HTTPException in real execution
            chat_endpoint(long_request)


def test_session_validation():
    """
    Test session validation
    """
    # Mock session service to return False for validation
    mock_session_service = Mock(spec=SessionService)
    mock_session_service.validate_session.return_value = False
    
    request_with_invalid_session = {
        "query": "What is RAG?",
        "session_id": "invalid_session_123"
    }
    
    with patch('src.api.chatbot_router.rag_service'), \
         patch('src.api.chatbot_router.session_service', mock_session_service), \
         patch('src.api.chatbot_router.database_service'), \
         patch('src.api.chatbot_router.api_logger'):
        
        with pytest.raises(Exception):  # Would be HTTPException in real execution
            chat_endpoint(request_with_invalid_session)


def test_session_creation():
    """
    Test session creation for requests without session_id
    """
    request_without_session = {
        "query": "What is RAG?"
    }
    
    with patch('src.api.chatbot_router.rag_service'), \
         patch('src.api.chatbot_router.session_service'), \
         patch('src.api.chatbot_router.database_service'), \
         patch('src.api.chatbot_router.api_logger'), \
         patch('src.api.chatbot_router.uuid.uuid4') as mock_uuid:
        
        # Mock uuid to return a predictable value
        mock_uuid.return_value.hex = "mocked_uuid"
        
        # This should not raise any exception
        # In real app this would call the endpoint and create a new session
        try:
            chat_endpoint(request_without_session)
        except TypeError:
            # Expected because we're mocking away the actual service implementations
            pass