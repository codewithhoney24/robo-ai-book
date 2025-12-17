import time
import logging
from typing import List, Optional, Dict, Any
from contextlib import contextmanager
from functools import wraps
from ..config.database_config import db_config
from ..config.logging_config import db_logger
from ..models.user_query import UserQuery
from ..models.user_session import UserSession
from ..exceptions import DatabaseConnectionException
from ..config.settings import settings


def retry_on_connection_failure(max_retries: int = 3, delay: float = 0.5):
    """
    Decorator to retry database operations on connection failure
    """
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    error_msg = str(e).lower()
                    is_connection_error = any(
                        keyword in error_msg for keyword in
                        ["connection", "ssl syscall", "server closed", "reset by peer", "timeout"]
                    )

                    if is_connection_error and attempt < max_retries:
                        db_logger.warning(f"Database operation failed (attempt {attempt + 1}/{max_retries + 1}): {str(e)}")
                        time.sleep(delay * (2 ** attempt))  # Exponential backoff
                        continue
                    else:
                        last_exception = e
                        break

            # If we've exhausted all retries, raise the last exception
            raise last_exception
        return wrapper
    return decorator


class DatabaseService:
    """
    Service for interacting with Neon Postgres database
    """

    def __init__(self):
        # Using the db_config to access the Postgres connection
        self.max_retries = settings.retry_attempts
        self.connection_timeout = settings.connection_timeout

    @contextmanager
    def get_cursor(self):
        """
        Context manager to get a database cursor
        """
        try:
            with db_config.get_postgres_cursor() as cursor:
                yield cursor
        except Exception as e:
            db_logger.error(f"Error in cursor context: {str(e)}")
            raise

    @retry_on_connection_failure(max_retries=3, delay=0.5)
    def create_session(self, session_data: UserSession) -> bool:
        """
        Create a new user session in the database
        """
        try:
            with self.get_cursor() as cursor:
                cursor.execute("""
                    INSERT INTO user_sessions (session_id, created_at, last_activity, user_agent)
                    VALUES (%s, %s, %s, %s)
                """, (
                    session_data.session_id,
                    session_data.created_at,
                    session_data.last_activity,
                    session_data.user_agent
                ))
                return True
        except Exception as e:
            db_logger.error(f"Error creating session: {str(e)}")
            raise DatabaseConnectionException(f"Failed to create session: {str(e)}")

    @retry_on_connection_failure(max_retries=3, delay=0.5)
    def update_session_activity(self, session_id: str) -> bool:
        """
        Update the last activity timestamp for a session
        """
        try:
            with self.get_cursor() as cursor:
                cursor.execute("""
                    UPDATE user_sessions
                    SET last_activity = NOW()
                    WHERE session_id = %s
                """, (session_id,))
                return cursor.rowcount > 0
        except Exception as e:
            db_logger.error(f"Error updating session activity: {str(e)}")
            raise DatabaseConnectionException(f"Failed to update session: {str(e)}")

    @retry_on_connection_failure(max_retries=3, delay=0.5)
    def get_session(self, session_id: str) -> Optional[UserSession]:
        """
        Get a user session by ID
        """
        try:
            with self.get_cursor() as cursor:
                cursor.execute("""
                    SELECT session_id, created_at, last_activity, user_agent
                    FROM user_sessions
                    WHERE session_id = %s
                """, (session_id,))
                row = cursor.fetchone()

                if row:
                    return UserSession(
                        session_id=row[0],
                        created_at=row[1],
                        last_activity=row[2],
                        user_agent=row[3]
                    )
                return None
        except Exception as e:
            db_logger.error(f"Error getting session: {str(e)}")
            raise DatabaseConnectionException(f"Failed to get session: {str(e)}")

    @retry_on_connection_failure(max_retries=3, delay=0.5)
    def create_user_query(self, query_data: UserQuery) -> bool:
        """
        Create a new user query in the database
        """
        try:
            with self.get_cursor() as cursor:
                cursor.execute("""
                    INSERT INTO user_queries
                    (query_id, content, timestamp, user_session_id, selected_text)
                    VALUES (%s, %s, %s, %s, %s)
                """, (
                    query_data.id,
                    query_data.content,
                    query_data.timestamp,
                    query_data.user_session_id,
                    query_data.selected_text
                ))
                return True
        except Exception as e:
            db_logger.error(f"Error creating user query: {str(e)}")
            raise DatabaseConnectionException(f"Failed to create user query: {str(e)}")

    @retry_on_connection_failure(max_retries=3, delay=0.5)
    def get_queries_by_session(self, session_id: str, limit: int = 10) -> List[UserQuery]:
        """
        Get user queries by session ID
        """
        try:
            with self.get_cursor() as cursor:
                cursor.execute("""
                    SELECT query_id, content, timestamp, user_session_id, selected_text
                    FROM user_queries
                    WHERE user_session_id = %s
                    ORDER BY timestamp DESC
                    LIMIT %s
                """, (session_id, limit))
                rows = cursor.fetchall()

                queries = []
                for row in rows:
                    query = UserQuery(
                        id=row[0],
                        content=row[1],
                        timestamp=row[2],
                        user_session_id=row[3],
                        selected_text=row[4]
                    )
                    queries.append(query)

                return queries
        except Exception as e:
            db_logger.error(f"Error getting queries by session: {str(e)}")
            raise DatabaseConnectionException(f"Failed to get queries by session: {str(e)}")


# Global instance
database_service = DatabaseService()