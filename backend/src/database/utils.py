import logging
import sys
from typing import Optional
from datetime import datetime
from enum import Enum


class LogCategory(Enum):
    """Categories for different types of logs"""
    CONNECTION = "connection"
    PERFORMANCE = "performance"
    ERROR = "error"
    WARNING = "warning"
    DEBUG = "debug"


class DatabaseLogger:
    """Custom logger for database operations with performance tracking"""

    def __init__(self, name: str = "db_logger"):
        self.logger = logging.getLogger(name)

        # Prevent adding multiple handlers if logger already exists
        if not self.logger.handlers:
            self.logger.setLevel(logging.INFO)

            # Create console handler
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(logging.INFO)

            # Create file handler
            file_handler = logging.FileHandler("database_operations.log")
            file_handler.setLevel(logging.INFO)

            # Create formatter
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            console_handler.setFormatter(formatter)
            file_handler.setFormatter(formatter)

            # Add handlers to logger
            self.logger.addHandler(console_handler)
            self.logger.addHandler(file_handler)

    def log_connection_event(self, event_type: str, details: Optional[dict] = None,
                           connection_id: Optional[str] = None):
        """Log a database connection event"""
        message = f"Connection {event_type}"
        if details:
            message += f" - Details: {details}"
        if connection_id:
            message += f" - Connection ID: {connection_id}"

        self.logger.info(f"[{LogCategory.CONNECTION.value}] {message}")

    def log_performance_metric(self, operation: str, duration_ms: float,
                              threshold_ms: float = 200.0):
        """Log a performance metric"""
        status = "OK" if duration_ms <= threshold_ms else "SLOW"
        message = f"Operation: {operation}, Duration: {duration_ms}ms, Threshold: {threshold_ms}ms, Status: {status}"

        log_level = logging.WARNING if duration_ms > threshold_ms else logging.INFO
        self.logger.log(log_level, f"[{LogCategory.PERFORMANCE.value}] {message}")

    def log_error(self, error: Exception, context: Optional[str] = None):
        """Log an error with context"""
        message = f"Error: {str(error)}"
        if context:
            message += f", Context: {context}"

        self.logger.error(f"[{LogCategory.ERROR.value}] {message}")

    def log_warning(self, message: str, context: Optional[str] = None):
        """Log a warning message"""
        if context:
            message = f"{message}, Context: {context}"

        self.logger.warning(f"[{LogCategory.WARNING.value}] {message}")

    def log_debug(self, message: str, context: Optional[str] = None):
        """Log a debug message"""
        if context:
            message = f"{message}, Context: {context}"

        self.logger.debug(f"[{LogCategory.DEBUG.value}] {message}")


# Global logger instance
db_logger = DatabaseLogger()


def validate_connection_pool_size(pool_size: int, neon_limit: int = 100) -> bool:
    """
    Validate connection pool size based on Neon connection limits

    Args:
        pool_size: Size of the connection pool to validate
        neon_limit: Maximum connections allowed by Neon tier (default 100 for paid, 20 for free)

    Returns:
        bool: True if valid, False otherwise
    """
    if pool_size <= 0:
        db_logger.log_error(ValueError("pool_size must be greater than 0"))
        return False

    if pool_size > neon_limit:
        db_logger.log_warning(
            f"Pool size ({pool_size}) exceeds Neon limit ({neon_limit})",
            context="Connection Pool Validation"
        )
        return False

    return True


def validate_timeout_values(connection_timeout: int, command_timeout: int) -> bool:
    """
    Validate timeout values are within acceptable ranges

    Args:
        connection_timeout: Connection timeout in seconds
        command_timeout: Command timeout in seconds

    Returns:
        bool: True if valid, False otherwise
    """
    connection_valid = 1 <= connection_timeout <= 60
    command_valid = 1 <= command_timeout <= 300

    if not connection_valid:
        db_logger.log_error(
            ValueError(f"connection_timeout must be between 1-60 seconds, got {connection_timeout}"),
            context="Timeout Validation"
        )

    if not command_valid:
        db_logger.log_error(
            ValueError(f"command_timeout must be between 1-300 seconds, got {command_timeout}"),
            context="Timeout Validation"
        )

    return connection_valid and command_valid


def validate_performance_threshold(percentile_95: float, threshold: float = 200.0) -> bool:
    """
    Validate that performance meets the required threshold

    Args:
        percentile_95: 95th percentile response time in milliseconds
        threshold: Maximum allowed response time in milliseconds (default 200ms)

    Returns:
        bool: True if within threshold, False otherwise
    """
    if percentile_95 > threshold:
        db_logger.log_warning(
            f"95th percentile response time ({percentile_95}ms) exceeds threshold ({threshold}ms)",
            context="Performance Validation"
        )
        return False

    return True


def validate_configuration_values(
    database_url: str,
    retry_attempts: int,
    window_size: int = 100
) -> bool:
    """
    Validate configuration values

    Args:
        database_url: Database connection string
        retry_attempts: Number of retry attempts
        window_size: Window size for performance measurement

    Returns:
        bool: True if valid, False otherwise
    """
    # Validate database URL - allow both PostgreSQL and SQLite URLs
    if not (database_url.startswith("postgresql://") or
            database_url.startswith("postgres://") or
            database_url.startswith("sqlite://")):
        db_logger.log_error(
            ValueError(f"Invalid database URL format: {database_url}"),
            context="Configuration Validation"
        )
        return False

    # Validate retry attempts
    if not (1 <= retry_attempts <= 10):
        db_logger.log_error(
            ValueError(f"retry_attempts must be between 1-10, got {retry_attempts}"),
            context="Configuration Validation"
        )
        return False

    # Validate window size for meaningful measurement
    if window_size < 10:
        db_logger.log_warning(
            f"Window size ({window_size}) too small for meaningful performance measurement",
            context="Configuration Validation"
        )
        return False

    return True