"""
Custom exceptions for the Neon Database Connection & Performance feature.
"""


class DatabaseConnectionError(Exception):
    """Raised when there's an issue establishing a database connection"""
    def __init__(self, message="Database connection failed", error_code=None):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)


class ConnectionPoolExhaustedError(Exception):
    """Raised when the connection pool reaches its limits"""
    def __init__(self, message="Database connection pool exhausted", connection_count=None):
        self.message = message
        self.connection_count = connection_count
        super().__init__(self.message)


class DatabaseTimeoutError(Exception):
    """Raised when database operations timeout"""
    def __init__(self, message="Database operation timed out", timeout_duration=None):
        self.message = message
        self.timeout_duration = timeout_duration
        super().__init__(self.message)


class PerformanceThresholdExceededError(Exception):
    """Raised when performance metrics exceed defined thresholds"""
    def __init__(self, message="Performance threshold exceeded", threshold=None, actual_value=None):
        self.message = message
        self.threshold = threshold
        self.actual_value = actual_value
        super().__init__(self.message)


class CircuitBreakerOpenError(Exception):
    """Raised when the circuit breaker is open due to repeated failures"""
    def __init__(self, message="Circuit breaker is open due to repeated failures", reset_timeout=None):
        self.message = message
        self.reset_timeout = reset_timeout
        super().__init__(self.message)


class DatabaseUnavailableError(Exception):
    """Raised when the database is unavailable"""
    def __init__(self, message="Database is temporarily unavailable"):
        self.message = message
        super().__init__(self.message)


class ValidationError(Exception):
    """Raised when validation fails for configuration or input"""
    def __init__(self, message="Validation failed", field=None, value=None):
        self.message = message
        self.field = field
        self.value = value
        super().__init__(self.message)