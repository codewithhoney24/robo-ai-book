import asyncio
import time
import logging
from contextlib import asynccontextmanager
from typing import Optional, Dict, Any
from sqlalchemy.ext.asyncio import AsyncSession
from src.database.connection import engine, AsyncSessionLocal, pool_status
from src.database.models.connection_pool import ConnectionPoolModel
from src.config.settings import settings
from src.core.exceptions import (
    DatabaseConnectionError,
    ConnectionPoolExhaustedError,
    DatabaseTimeoutError
)
from src.database.utils import (
    db_logger,
    validate_connection_pool_size,
    validate_timeout_values
)
from src.services.error_handler import CircuitBreaker


logger = logging.getLogger(__name__)


class ConnectionPoolService:
    """
    Service to manage database connections using pooling.
    Implements the connection pool model from the data model.
    """

    def __init__(self):
        self.pool_size = settings.max_connections
        self.min_pool_size = settings.min_connections
        self.connection_timeout = settings.connection_timeout
        self.command_timeout = settings.command_timeout
        self.status = "active"  # Default status
        self.active_connections = 0
        self.queued_requests = 0

        # Initialize circuit breaker for this service
        self.circuit_breaker = CircuitBreaker(
            failure_threshold=5,
            recovery_timeout=30  # 30 seconds
        )

        # Validate configuration
        if not validate_connection_pool_size(self.pool_size):
            raise ValueError(f"Invalid pool size: {self.pool_size}")

        if not validate_timeout_values(self.connection_timeout, self.command_timeout):
            raise ValueError(
                f"Invalid timeout values - connection: {self.connection_timeout}, "
                f"command: {self.command_timeout}"
            )

    @asynccontextmanager
    async def get_connection(self):
        """
        Context manager to get a database connection from the pool.
        Handles connection lifecycle and error management.
        """
        if self.circuit_breaker.is_open():
            raise ConnectionPoolExhaustedError(
                message="Circuit breaker is open, database temporarily unavailable",
                connection_count=self.active_connections
            )

        start_time = time.time()
        connection_acquired = False

        try:
            # Increment active connections counter
            self.active_connections += 1

            # Get connection from the pool
            async with AsyncSessionLocal() as session:
                connection_acquired = True
                yield session

        except Exception as e:
            logger.error(f"Database connection error: {e}")
            db_logger.log_error(e, context="Connection Pool Service")

            if isinstance(e, asyncio.TimeoutError):
                raise DatabaseTimeoutError(
                    timeout_duration=self.connection_timeout
                ) from e
            else:
                # Mark as failure in circuit breaker
                self.circuit_breaker.record_failure()
                raise DatabaseConnectionError(str(e)) from e
        finally:
            # Decrement active connections counter
            if connection_acquired:
                self.active_connections -= 1

            # Log connection operation performance
            duration = (time.time() - start_time) * 1000  # Convert to milliseconds
            db_logger.log_performance_metric(
                operation="get_connection",
                duration_ms=duration,
                threshold_ms=self.command_timeout * 1000  # Convert to ms for comparison
            )

    async def get_pool_status(self) -> Dict[str, Any]:
        """
        Get the current status of the connection pool.
        """
        # Update the status regularly to reflect current state
        await pool_status.update_status()

        return {
            "status": self.status,
            "pool_size": self.pool_size,
            "active_connections": self.active_connections,
            "queued_requests": self.queued_requests,
            "min_pool_size": self.min_pool_size,
            "connection_timeout": self.connection_timeout,
            "command_timeout": self.command_timeout,
            "circuit_breaker_state": self.circuit_breaker.state.value if hasattr(self.circuit_breaker.state, 'value') else str(self.circuit_breaker.state),
            "circuit_breaker_failures": self.circuit_breaker.failure_count
        }

    async def validate_connection(self, session: AsyncSession) -> bool:
        """
        Validate that a database connection is still active.
        """
        try:
            # Execute a simple query to test connection
            await session.execute("SELECT 1")
            return True
        except Exception as e:
            logger.error(f"Connection validation failed: {e}")
            return False

    async def execute_with_retry(self, operation_func, max_retries: Optional[int] = None):
        """
        Execute a database operation with retry logic.
        Implements the retry policies from research.md.
        """
        if max_retries is None:
            max_retries = settings.retry_attempts

        last_exception = None

        for attempt in range(max_retries):
            try:
                if self.circuit_breaker.is_open():
                    raise ConnectionPoolExhaustedError(
                        message="Circuit breaker is open, database temporarily unavailable"
                    )

                # Execute the operation
                result = await operation_func()

                # If successful, reset circuit breaker
                self.circuit_breaker.record_success()

                # Log successful operation
                db_logger.log_connection_event(
                    "operation_success",
                    {"attempt": attempt + 1, "operation": operation_func.__name__}
                )

                return result

            except Exception as e:
                last_exception = e

                # Log the failure
                db_logger.log_error(
                    e,
                    context=f"Operation failed on attempt {attempt + 1}: {operation_func.__name__}"
                )

                # Record failure in circuit breaker
                self.circuit_breaker.record_failure()

                # If this was the last attempt, don't sleep before raising
                if attempt == max_retries - 1:
                    break

                # Sleep with exponential backoff and jitter
                # Formula: base_delay * (multiplier ^ attempt) + random_jitter
                base_delay = 0.1  # 100ms base delay
                delay = base_delay * (2 ** attempt)  # Exponential backoff
                jitter = asyncio.random.uniform(0, 0.1)  # 0-100ms jitter
                total_delay = delay + jitter

                logger.info(f"Retrying operation in {total_delay:.2f} seconds...")
                await asyncio.sleep(total_delay)

        # If all retries failed, raise the last exception
        raise last_exception

    def update_status(self, new_status: str):
        """
        Update the status of the connection pool.
        """
        valid_statuses = ['active', 'warming_up', 'degraded', 'unavailable']
        if new_status not in valid_statuses:
            raise ValueError(f"Invalid status: {new_status}. Must be one of {valid_statuses}")

        self.status = new_status
        db_logger.log_connection_event(f"status_change_to_{new_status}")

    def get_pool_utilization_percentage(self) -> float:
        """
        Get the percentage of pool utilization.

        Returns:
            float: Percentage of pool utilization (0.0 to 100.0)
        """
        if self.pool_size <= 0:
            return 0.0
        return (self.active_connections / self.pool_size) * 100.0

    def is_pool_healthy(self) -> bool:
        """
        Check if the connection pool is in a healthy state.

        Returns:
            bool: True if healthy, False if degraded or unavailable
        """
        return self.status in ['active', 'warming_up']

    def is_degraded(self) -> bool:
        """
        Check if the connection pool is in a degraded state.

        Returns:
            bool: True if degraded, False otherwise
        """
        return self.status == 'degraded'

    def is_unavailable(self) -> bool:
        """
        Check if the connection pool is unavailable.

        Returns:
            bool: True if unavailable, False otherwise
        """
        return self.status == 'unavailable'

    async def handle_connection_limit_reached(self, operation_func, *args, **kwargs):
        """
        Handle the situation when connection limits are reached.
        Implements graceful degradation as per research.md.

        Args:
            operation_func: The operation to attempt
            *args, **kwargs: Arguments to pass to the operation

        Returns:
            Result of the operation or a fallback response
        """
        # Check if we're already in a degraded state
        if self.status == 'degraded':
            db_logger.log_warning(
                "Connection pool already in degraded state, queuing request",
                context="Connection Limit Handler"
            )
            # In degraded state, we may want to queue or limit requests
            self.queued_requests += 1
            try:
                # Try to execute the operation with reduced timeout
                result = await operation_func(*args, **kwargs)
                return result
            finally:
                self.queued_requests -= 1
        elif self.status == 'unavailable':
            db_logger.log_error(
                DatabaseConnectionError("Database is unavailable"),
                context="Connection Limit Handler"
            )
            # Return a fallback response when unavailable
            return {
                "error": "Database temporarily unavailable",
                "fallback_response": "We're experiencing technical difficulties. Please try again later."
            }
        else:
            # Check if we're approaching the connection limit
            utilization = self.get_pool_utilization_percentage()
            if utilization >= 80:  # Consider degraded at 80% utilization
                db_logger.log_warning(
                    f"Connection pool utilization at {utilization:.2f}%, approaching limit",
                    context="Connection Limit Handler"
                )

                # Switch to degraded state
                self.update_status('degraded')

                # Try to execute the operation
                try:
                    result = await operation_func(*args, **kwargs)
                    # If successful, check if we can return to active state
                    if self.get_pool_utilization_percentage() < 70:
                        self.update_status('active')
                    return result
                except Exception as e:
                    # If operation fails in degraded state, handle appropriately
                    db_logger.log_error(
                        e,
                        context="Operation failed in degraded state"
                    )
                    return {
                        "error": str(e),
                        "fallback_response": "Service is currently under heavy load. Please try again shortly."
                    }

            # If not approaching the limit, execute normally
            return await operation_func(*args, **kwargs)


# Global instance
connection_pool_service = ConnectionPoolService()