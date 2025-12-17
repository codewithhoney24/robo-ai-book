from datetime import datetime, timedelta
from typing import Optional
from ..models.user_session import UserSession
from ..services.database_service import database_service
from ..config.logging_config import rag_logger


class SessionService:
    """
    Service for managing user sessions in web interface
    """
    
    def __init__(self):
        self.db_service = database_service
    
    def create_session(self, session_id: str, user_agent: Optional[str] = None) -> bool:
        """
        Create a new user session
        """
        try:
            session = UserSession(
                session_id=session_id,
                created_at=datetime.utcnow(),
                last_activity=datetime.utcnow(),
                user_agent=user_agent
            )
            
            result = self.db_service.create_session(session)
            
            if result:
                rag_logger.info(f"Created new session: {session_id}")
            
            return result
        except Exception as e:
            rag_logger.error(f"Error creating session {session_id}: {str(e)}")
            return False
    
    def get_session(self, session_id: str) -> Optional[UserSession]:
        """
        Get a user session by ID
        """
        try:
            session = self.db_service.get_session(session_id)
            
            if session:
                rag_logger.debug(f"Retrieved session: {session_id}")
            else:
                rag_logger.warning(f"Session not found: {session_id}")
            
            return session
        except Exception as e:
            rag_logger.error(f"Error retrieving session {session_id}: {str(e)}")
            return None
    
    def update_session_activity(self, session_id: str) -> bool:
        """
        Update the last activity timestamp for a session
        """
        try:
            result = self.db_service.update_session_activity(session_id)
            
            if result:
                rag_logger.debug(f"Updated activity for session: {session_id}")
            else:
                rag_logger.warning(f"Failed to update activity for session: {session_id}")
            
            return result
        except Exception as e:
            rag_logger.error(f"Error updating session activity {session_id}: {str(e)}")
            return False
    
    def is_session_expired(self, session: UserSession, hours: int = 24) -> bool:
        """
        Check if a session has expired based on last activity
        Default expiration is 24 hours
        """
        try:
            # Calculate the time threshold
            threshold = datetime.utcnow() - timedelta(hours=hours)
            
            # Compare last activity with threshold
            if isinstance(session.last_activity, str):
                # Parse string datetime if needed
                last_activity = datetime.fromisoformat(session.last_activity.replace('Z', '+00:00'))
            else:
                last_activity = session.last_activity
            
            return last_activity < threshold
        except Exception as e:
            rag_logger.error(f"Error checking session expiry for {session.session_id}: {str(e)}")
            # If there's an error, assume the session is not expired to be safe
            return False
    
    def validate_session(self, session_id: str) -> bool:
        """
        Validate if a session exists and is not expired
        """
        session = self.get_session(session_id)
        
        if not session:
            return False
        
        if self.is_session_expired(session):
            rag_logger.info(f"Session {session_id} has expired")
            return False
        
        return True


# Global instance
session_service = SessionService()