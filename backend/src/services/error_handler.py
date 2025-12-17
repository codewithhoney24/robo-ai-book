import asyncio
import time
import logging
from enum import Enum
from typing import Optional, Dict, Any, Callable, Awaitable
from src.core.exceptions import CircuitBreakerOpenError, DatabaseUnavailableError
from src.database.utils import db_logger


logger = logging.getLogger(__name__)


class CircuitBreakerState(Enum):
    CLOSED = "closed"      # Normal operation
    OPEN = "open"          # Tripped, blocking requests
    HALF_OPEN = "half_open" # Testing if failure condition is resolved


class CircuitBreaker:
    """
    Circuit breaker implementation to prevent cascading failures.
    Implements the circuit breaker pattern from research.md.
    """
    
    def __init__(self, failure_threshold: int = 5, recovery_timeout: int = 60):
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout  # in seconds
        
        self.failure_count = 0
        self.last_failure_time = None
        self.state = CircuitBreakerState.CLOSED
        
    def record_failure(self):
        """Record a failure and update the circuit breaker state."""
        self.failure_count += 1
        self.last_failure_time = time.time()
        
        if self.failure_count >= self.failure_threshold:
            self.state = CircuitBreakerState.OPEN
            db_logger.log_warning(
                f"Circuit breaker opened after {self.failure_count} failures",
                "CircuitBreaker"
            )
    
    def record_success(self):
        """Record a success and reset the failure counter."""
        self.failure_count = 0
        self.state = CircuitBreakerState.CLOSED
    
    def is_open(self) -> bool:
        """Check if the circuit breaker is open."""
        if self.state == CircuitBreakerState.OPEN:
            # Check if it's time to try again (half-open state)
            if time.time() - self.last_failure_time >= self.recovery_timeout:
                self.state = CircuitBreakerState.HALF_OPEN
                db_logger.log_debug("Circuit breaker entering half-open state", "CircuitBreaker")
            else:
                return True  # Still in open state
        
        return self.state == CircuitBreakerState.OPEN
    
    def allow_request(self) -> bool:
        """Check if a request should be allowed."""
        if self.state == CircuitBreakerState.HALF_OPEN:
            # In half-open state, allow one request to test recovery
            return True
        elif self.state == CircuitBreakerState.OPEN:
            # In open state, reject requests
            return False
        else:
            # In closed state, allow requests
            return True


class ErrorHandler:
    """
    Error handling service with fallbacks and circuit breaker pattern.
    Implements the error handling approach from research.md.
    """
    
    def __init__(self):
        self.circuit_breaker = CircuitBreaker()
        self.error_handlers = {}
        
        # Default fallback responses
        self.fallback_responses = {
            "service_unavailable": {
                "error": "Service temporarily unavailable",
                "fallback_response": "We're experiencing technical difficulties. Please try again in a moment."
            },
            "connection_limit": {
                "error": "Connection limit reached",
                "fallback_response": "High demand detected. Please try again in a moment."
            },
            "timeout": {
                "error": "Request timed out",
                "fallback_response": "Request took too long. Please try again."
            }
        }
    
    async def handle_database_error(
        self, 
        error: Exception, 
        fallback_type: str = "service_unavailable",
        context: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Handle database-related errors with appropriate responses.
        
        Args:
            error: The exception that occurred
            fallback_type: Type of fallback to use
            context: Additional context about where the error occurred
            
        Returns:
            Dict with error information and fallback response
        """
        # Record failure in circuit breaker
        self.circuit_breaker.record_failure()
        
        # Log the error
        db_logger.log_error(error, context=f"Database error - {context}" if context else "Database error")
        
        # Determine error type and appropriate response
        error_type = self._categorize_error(error)
        
        # Prepare error response
        error_response = {
            "error_type": error_type,
            "message": str(error),
            "fallback_response": self.fallback_responses.get(fallback_type, {}).get("fallback_response", "An error occurred"),
            "timestamp": time.time()
        }
        
        return error_response
    
    def _categorize_error(self, error: Exception) -> str:
        """Categorize an error based on its type."""
        error_str = str(error).lower()
        
        if "connection" in error_str and ("refused" in error_str or "timeout" in error_str):
            return "connection_error"
        elif "pool" in error_str and ("exhausted" in error_str or "limit" in error_str):
            return "connection_limit"
        elif "timeout" in error_str:
            return "timeout"
        elif "unavailable" in error_str:
            return "unavailable"
        else:
            return "general_error"
    
    async def execute_with_fallback(
        self, 
        primary_func: Callable[[], Awaitable],
        fallback_func: Optional[Callable[[], Awaitable]] = None,
        fallback_type: str = "service_unavailable",
        max_retries: int = 3
    ) -> Any:
        """
        Execute a function with primary implementation, falling back to alternative
        if the primary fails.
        
        Args:
            primary_func: Primary function to execute
            fallback_func: Fallback function to execute if primary fails
            fallback_type: Type of fallback to use if fallback_func is None
            max_retries: Maximum number of retries before falling back
            
        Returns:
            Result of either primary or fallback function
        """
        # Check if circuit breaker allows the request
        if self.circuit_breaker.is_open():
            raise CircuitBreakerOpenError(
                reset_timeout=self.circuit_breaker.recovery_timeout
            )
        
        last_exception = None
        
        # Try primary function with retries
        for attempt in range(max_retries):
            try:
                if not self.circuit_breaker.allow_request():
                    # Circuit breaker may have opened during retries
                    raise CircuitBreakerOpenError()
                
                result = await primary_func()
                self.circuit_breaker.record_success()  # Record success
                return result
            except Exception as e:
                last_exception = e
                db_logger.log_error(
                    e, 
                    context=f"Primary function failed on attempt {attempt + 1}"
                )
                
                # Add a small delay between retries
                await asyncio.sleep(0.1 * (2 ** attempt))  # Exponential backoff: 0.1s, 0.2s, 0.4s
        
        # If all retries failed and we have a fallback, try it
        if fallback_func:
            try:
                result = await fallback_func()
                db_logger.log_debug("Fallback function executed successfully", "ErrorHandler")
                return result
            except Exception as e:
                db_logger.log_error(
                    e, 
                    context="Fallback function also failed"
                )
        
        # If no fallback or fallback also failed, return fallback response
        if last_exception:
            return await self.handle_database_error(last_exception, fallback_type)
        
        # This should not happen but added for completeness
        return self.fallback_responses.get(fallback_type, {})
    
    def get_error_summary(self) -> Dict[str, Any]:
        """
        Get a summary of the current error handling state.
        """
        return {
            "circuit_breaker_state": self.circuit_breaker.state.value,
            "failure_count": self.circuit_breaker.failure_count,
            "failure_threshold": self.circuit_breaker.failure_threshold,
            "recovery_timeout": self.circuit_breaker.recovery_timeout,
            "last_failure_time": self.circuit_breaker.last_failure_time
        }
    
    def reset_circuit_breaker(self):
        """Manually reset the circuit breaker."""
        self.circuit_breaker.failure_count = 0
        self.circuit_breaker.state = CircuitBreakerState.CLOSED
        self.circuit_breaker.last_failure_time = None
        db_logger.log_debug("Circuit breaker manually reset", "ErrorHandler")


# Global instance
error_handler_service = ErrorHandler()