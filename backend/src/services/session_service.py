from typing import Optional
from datetime import datetime, timedelta
from ..models.user_session import UserSession
from ..services.database_service import database_service
from ..config.logging_config import api_logger as service_logger


class SessionService:
    """
    Service for managing user sessions
    """

    def __init__(self):
        self.session_timeout = timedelta(hours=24)  # Sessions expire after 24 hours of inactivity

    def create_session(self, session_id: str, user_agent: Optional[str] = None) -> bool:
        """
        Create a new user session
        """
        try:
            # Create a UserSession object with the provided data
            session_data = UserSession(
                session_id=session_id,
                user_agent=user_agent
            )

            # Use the database service to create the session
            result = database_service.create_session(session_data)

            if result:
                service_logger.info(f"Created new session: {session_id}")
            else:
                service_logger.warning(f"Failed to create session: {session_id}")

            return result
        except Exception as e:
            service_logger.error(f"Error creating session {session_id}: {str(e)}")
            return False

    def validate_session(self, session_id: str) -> bool:
        """
        Validate if a session exists and is still active
        """
        try:
            session = database_service.get_session(session_id)

            if not session:
                service_logger.info(f"Session not found: {session_id}")
                return False

            # Check if session has expired based on last activity
            if datetime.utcnow() - session.last_activity > self.session_timeout:
                service_logger.info(f"Session expired: {session_id}")
                return False

            service_logger.info(f"Session validated: {session_id}")
            return True
        except Exception as e:
            service_logger.error(f"Error validating session {session_id}: {str(e)}")
            return False

    def update_session_activity(self, session_id: str) -> bool:
        """
        Update the last activity timestamp for a session
        """
        try:
            result = database_service.update_session_activity(session_id)

            if result:
                service_logger.info(f"Updated session activity: {session_id}")
            else:
                service_logger.warning(f"Failed to update session activity: {session_id}")

            return result
        except Exception as e:
            service_logger.error(f"Error updating session activity {session_id}: {str(e)}")
            return False


# Global instance
session_service = SessionService()