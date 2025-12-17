import pytest
from unittest.mock import Mock, patch, MagicMock
from src.services.database_service import DatabaseService
from src.models.user_query import UserQuery
from src.models.user_session import UserSession
from datetime import datetime


class TestDatabaseService:
    """
    Comprehensive tests for DatabaseService to improve test coverage
    """
    
    @pytest.fixture
    def db_service(self):
        with patch('src.services.database_service.db_config') as mock_db_config:
            mock_cursor = Mock()
            mock_conn = Mock()
            mock_conn.cursor.return_value.__enter__.return_value = mock_cursor
            
            mock_db_config.get_postgres_connection.return_value = mock_conn
            
            service = DatabaseService()
            return service, mock_cursor
    
    def test_create_session_success(self, db_service):
        """
        Test successful session creation
        """
        service, mock_cursor = db_service
        session_data = UserSession(
            session_id="test_session_123",
            created_at=datetime.utcnow(),
            last_activity=datetime.utcnow(),
            user_agent="Mozilla/5.0"
        )
        
        mock_cursor.rowcount = 1
        result = service.create_session(session_data)
        
        assert result is True
        mock_cursor.execute.assert_called_once()
    
    def test_update_session_activity_success(self, db_service):
        """
        Test successful session activity update
        """
        service, mock_cursor = db_service
        session_id = "test_session_123"
        
        mock_cursor.rowcount = 1
        result = service.update_session_activity(session_id)
        
        assert result is True
        mock_cursor.execute.assert_called_once()
    
    def test_update_session_activity_no_rows(self, db_service):
        """
        Test session activity update when no rows are affected
        """
        service, mock_cursor = db_service
        session_id = "nonexistent_session"
        
        mock_cursor.rowcount = 0
        result = service.update_session_activity(session_id)
        
        assert result is False
    
    def test_get_session_found(self, db_service):
        """
        Test successful session retrieval
        """
        service, mock_cursor = db_service
        session_id = "test_session_123"
        
        # Mock a database row
        mock_cursor.fetchone.return_value = [
            "test_session_123",
            datetime.utcnow(),
            datetime.utcnow(),
            "Mozilla/5.0"
        ]
        
        result = service.get_session(session_id)
        
        assert result is not None
        assert result.session_id == session_id
        mock_cursor.execute.assert_called_once()
    
    def test_get_session_not_found(self, db_service):
        """
        Test session retrieval when session doesn't exist
        """
        service, mock_cursor = db_service
        session_id = "nonexistent_session"
        
        # Mock no results from database
        mock_cursor.fetchone.return_value = None
        
        result = service.get_session(session_id)
        
        assert result is None
    
    def test_create_user_query_success(self, db_service):
        """
        Test successful user query creation
        """
        service, mock_cursor = db_service
        query_data = UserQuery(
            id="test_query_123",
            content="Test query content",
            user_session_id="test_session_123",
            timestamp=datetime.utcnow()
        )
        
        result = service.create_user_query(query_data)
        
        assert result is True
        mock_cursor.execute.assert_called_once()
    
    def test_get_queries_by_session_success(self, db_service):
        """
        Test successful retrieval of queries by session
        """
        service, mock_cursor = db_service
        session_id = "test_session_123"
        
        # Mock database rows
        mock_cursor.fetchall.return_value = [
            [
                "query_1",
                "First query content",
                datetime.utcnow(),
                session_id,
                None
            ],
            [
                "query_2", 
                "Second query content",
                datetime.utcnow(),
                session_id,
                "Selected text"
            ]
        ]
        
        result = service.get_queries_by_session(session_id)
        
        assert len(result) == 2
        assert result[0].id == "query_1"
        assert result[1].id == "query_2"
        mock_cursor.execute.assert_called_once()
    
    def test_get_queries_by_session_empty(self, db_service):
        """
        Test retrieval of queries when none exist for session
        """
        service, mock_cursor = db_service
        session_id = "empty_session"
        
        # Mock no results from database
        mock_cursor.fetchall.return_value = []
        
        result = service.get_queries_by_session(session_id)
        
        assert result == []
    
    def test_database_connection_exception_handling(self, db_service):
        """
        Test database service handling of connection exceptions
        """
        service, mock_cursor = db_service
        session_id = "test_session_123"
        
        # Mock an exception during database operation
        mock_cursor.execute.side_effect = Exception("Database connection failed")
        
        with pytest.raises(Exception) as exc_info:
            service.get_session(session_id)
        
        assert "Database connection failed" in str(exc_info.value)