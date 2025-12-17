import pytest
from unittest.mock import Mock, patch
from src.services.session_service import SessionService, UserSession
from datetime import datetime, timedelta


class TestSessionService:
    """
    Comprehensive tests for SessionService to improve test coverage
    """
    
    @pytest.fixture
    def session_service(self):
        with patch('src.services.session_service.database_service') as mock_db_service:
            service = SessionService()
            service.db_service = mock_db_service
            return service
    
    def test_create_session_success(self, session_service):
        """
        Test successful session creation
        """
        session_id = "test_session_123"
        user_agent = "Mozilla/5.0 Test Browser"
        
        # Mock successful database operation
        session_service.db_service.create_session.return_value = True
        
        result = session_service.create_session(session_id, user_agent)
        
        assert result is True
        session_service.db_service.create_session.assert_called_once()
    
    def test_create_session_failure(self, session_service):
        """
        Test session creation failure
        """
        session_id = "test_session_456"
        user_agent = "Mozilla/5.0 Test Browser"
        
        # Mock failed database operation
        session_service.db_service.create_session.return_value = False
        
        result = session_service.create_session(session_id, user_agent)
        
        assert result is False
    
    def test_get_session_success(self, session_service):
        """
        Test successful session retrieval
        """
        session_id = "test_session_789"
        expected_session = UserSession(
            session_id=session_id,
            created_at=datetime.utcnow(),
            last_activity=datetime.utcnow(),
            user_agent="Mozilla/5.0"
        )
        
        # Mock successful database retrieval
        session_service.db_service.get_session.return_value = expected_session
        
        result = session_service.get_session(session_id)
        
        assert result is expected_session
        assert result.session_id == session_id
    
    def test_get_session_none(self, session_service):
        """
        Test session retrieval when session doesn't exist
        """
        session_id = "nonexistent_session"
        
        # Mock None return from database
        session_service.db_service.get_session.return_value = None
        
        result = session_service.get_session(session_id)
        
        assert result is None
    
    def test_update_session_activity_success(self, session_service):
        """
        Test successful session activity update
        """
        session_id = "test_session_activity"
        
        # Mock successful database update
        session_service.db_service.update_session_activity.return_value = True
        
        result = session_service.update_session_activity(session_id)
        
        assert result is True
        session_service.db_service.update_session_activity.assert_called_once_with(session_id)
    
    def test_update_session_activity_failure(self, session_service):
        """
        Test session activity update failure
        """
        session_id = "test_session_activity_fail"
        
        # Mock failed database update
        session_service.db_service.update_session_activity.return_value = False
        
        result = session_service.update_session_activity(session_id)
        
        assert result is False
    
    def test_is_session_expired_true(self, session_service):
        """
        Test session expiry when last activity was long ago
        """
        old_time = datetime.utcnow() - timedelta(hours=25)  # More than 24 hours ago
        session = UserSession(
            session_id="old_session",
            created_at=old_time,
            last_activity=old_time,
            user_agent="Mozilla/5.0"
        )
        
        result = session_service.is_session_expired(session, hours=24)
        
        assert result is True
    
    def test_is_session_expired_false(self, session_service):
        """
        Test session not expired when last activity was recent
        """
        recent_time = datetime.utcnow() - timedelta(hours=1)  # 1 hour ago
        session = UserSession(
            session_id="recent_session",
            created_at=recent_time,
            last_activity=recent_time,
            user_agent="Mozilla/5.0"
        )
        
        result = session_service.is_session_expired(session, hours=24)
        
        assert result is False
    
    def test_is_session_expired_with_string_datetime(self, session_service):
        """
        Test session expiry check with string datetime
        """
        old_time_str = (datetime.utcnow() - timedelta(hours=25)).isoformat()
        session = UserSession(
            session_id="old_session",
            created_at=datetime.utcnow(),
            last_activity=old_time_str,
            user_agent="Mozilla/5.0"
        )
        
        result = session_service.is_session_expired(session, hours=24)
        
        assert result is True
    
    def test_validate_session_success(self, session_service):
        """
        Test valid session validation
        """
        session_id = "valid_session"
        session = UserSession(
            session_id=session_id,
            created_at=datetime.utcnow(),
            last_activity=datetime.utcnow(),
            user_agent="Mozilla/5.0"
        )
        
        # Mock session exists and is not expired
        session_service.db_service.get_session.return_value = session
        session_service.is_session_expired.return_value = False
        
        result = session_service.validate_session(session_id)
        
        assert result is True
    
    def test_validate_session_not_found(self, session_service):
        """
        Test session validation when session doesn't exist
        """
        session_id = "missing_session"
        
        # Mock session not found
        session_service.db_service.get_session.return_value = None
        
        result = session_service.validate_session(session_id)
        
        assert result is False
    
    def test_validate_session_expired(self, session_service):
        """
        Test session validation when session is expired
        """
        session_id = "expired_session"
        session = UserSession(
            session_id=session_id,
            created_at=datetime.utcnow(),
            last_activity=datetime.utcnow(),
            user_agent="Mozilla/5.0"
        )
        
        # Mock session exists but is expired
        session_service.db_service.get_session.return_value = session
        session_service.is_session_expired.return_value = True
        
        result = session_service.validate_session(session_id)
        
        assert result is False