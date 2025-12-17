import pytest
from unittest.mock import Mock, patch
from src.services.session_service import SessionService
from src.models.user_session import UserSession


@pytest.fixture
def session_service():
    with patch('src.services.session_service.database_service') as mock_db_service:
        service = SessionService()
        service.db_service = mock_db_service
        return service


def test_create_session(session_service):
    """
    Unit test for creating a new user session
    """
    session_id = "test_session_123"
    user_agent = "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36"
    
    # Mock the database service to return True for session creation
    session_service.db_service.create_session.return_value = True
    
    result = session_service.create_session(session_id, user_agent)
    
    assert result is True
    session_service.db_service.create_session.assert_called_once()


def test_get_session(session_service):
    """
    Unit test for retrieving a user session
    """
    session_id = "test_session_123"
    
    # Mock a session response
    mock_session = UserSession(
        session_id=session_id,
        created_at="2023-01-01T00:00:00",
        last_activity="2023-01-01T00:00:00",
        user_agent="Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36"
    )
    
    session_service.db_service.get_session.return_value = mock_session
    
    result = session_service.get_session(session_id)
    
    assert result is not None
    assert result.session_id == session_id
    session_service.db_service.get_session.assert_called_once_with(session_id)


def test_update_session_activity(session_service):
    """
    Unit test for updating session activity
    """
    session_id = "test_session_123"
    
    # Mock the database service to return True for update
    session_service.db_service.update_session_activity.return_value = True
    
    result = session_service.update_session_activity(session_id)
    
    assert result is True
    session_service.db_service.update_session_activity.assert_called_once_with(session_id)


def test_session_expiry_logic(session_service):
    """
    Unit test for session expiry logic
    """
    # Test with a session that's not expired
    active_session = UserSession(
        session_id="active_session",
        created_at="2023-01-01T00:00:00",
        last_activity="2023-01-01T12:00:00",  # Recent activity
        user_agent="Mozilla/5.0"
    )
    
    # Session should not be expired if it has recent activity
    is_expired = session_service.is_session_expired(active_session, hours=24)
    assert is_expired is False
    
    # Test with a session that has expired
    expired_session = UserSession(
        session_id="expired_session",
        created_at="2023-01-01T00:00:00",
        last_activity="2023-01-01T00:00:00",  # Long ago
        user_agent="Mozilla/5.0"
    )
    
    # Session should be expired if the last activity was more than 24 hours ago
    is_expired = session_service.is_session_expired(expired_session, hours=24)
    assert is_expired is True