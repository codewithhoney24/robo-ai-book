import os
import asyncio
import asyncpg
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.pool import AsyncAdaptedQueuePool
from contextlib import asynccontextmanager
from typing import AsyncGenerator
import logging

# Import settings to mask the database URL properly
from src.config.settings import settings
from src.database.utils import db_logger
from src.utils.db_url_parser import clean_database_url_for_asyncpg

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load configuration from environment or defaults
DATABASE_URL = settings.neon_database_url
MAX_CONNECTIONS = settings.max_connections
MIN_CONNECTIONS = settings.min_connections
CONNECTION_TIMEOUT = settings.connection_timeout
COMMAND_TIMEOUT = settings.command_timeout
MAX_LIFETIME = int(os.getenv("MAX_LIFETIME", "3600"))  # 1 hour
POOL_RECYCLE = int(os.getenv("POOL_RECYCLE", "3600"))  # 1 hour

# Ensure we have a valid database URL
if not DATABASE_URL or DATABASE_URL.strip() == "":
    # Provide a default or placeholder URL if the environment variable is not set
    logger.warning("NEON_DATABASE_URL not found in environment. Using default connection settings.")
    DATABASE_URL = "sqlite+aiosqlite:///./default.db"

# Clean the database URL for asyncpg first (only for PostgreSQL URLs)
if DATABASE_URL.startswith("postgresql://") or DATABASE_URL.startswith("postgres://"):
    clean_database_url = clean_database_url_for_asyncpg(DATABASE_URL)
else:
    # For SQLite, don't process the URL with PostgreSQL-specific cleaning
    clean_database_url = DATABASE_URL

# Ensure async driver in DATABASE_URL (use asyncpg for PostgreSQL, aiosqlite for SQLite)
if clean_database_url.startswith("postgresql://") and "+" not in clean_database_url.split("://",1)[1]:
    async_db_url = clean_database_url.replace("postgresql://", "postgresql+asyncpg://", 1)
elif clean_database_url.startswith("postgres://") and "+" not in clean_database_url.split("://",1)[1]:
    async_db_url = clean_database_url.replace("postgres://", "postgresql+asyncpg://", 1)
elif clean_database_url.startswith("sqlite://") and "+" not in clean_database_url.split("://",1)[1]:
    # Handle both sqlite:// and sqlite:/// formats
    async_db_url = clean_database_url.replace("sqlite://", "sqlite+aiosqlite://", 1)
else:
    async_db_url = clean_database_url

# Create async engine with connection pooling
engine = create_async_engine(
    async_db_url,
    poolclass=AsyncAdaptedQueuePool,
    pool_size=MAX_CONNECTIONS,
    pool_timeout=CONNECTION_TIMEOUT,
    pool_recycle=POOL_RECYCLE,
    pool_pre_ping=True,  # Verify connections are alive before use
    echo=False  # Set to True for SQL debugging
)

# Create async session maker
AsyncSessionLocal = async_sessionmaker(engine, expire_on_commit=False)

async def get_db_session() -> AsyncGenerator[AsyncSession, None]:
    """
    Dependency function that provides database sessions with timeout handling.
    """
    import async_timeout

    try:
        async with async_timeout.timeout(COMMAND_TIMEOUT):
            async with AsyncSessionLocal() as session:
                try:
                    yield session
                except Exception as e:
                    await session.rollback()
                    logger.error(f"Database session error: {e}")
                    raise
                finally:
                    await session.close()
    except asyncio.TimeoutError:
        logger.error(f"Database command timed out after {COMMAND_TIMEOUT} seconds")
        raise
    except Exception as e:
        logger.error(f"Database session setup error: {e}")
        raise


async def validate_db_connection() -> bool:
    """
    Validate that the database connection is working properly.

    Returns:
        bool: True if connection is valid, False otherwise
    """
    from sqlalchemy import text
    try:
        async with async_timeout.timeout(CONNECTION_TIMEOUT):
            async with engine.connect() as conn:
                # Execute a simple query to test the connection
                result = await conn.execute(text("SELECT 1"))
                _ = result.fetchone()
        return True
    except Exception as e:
        logger.error(f"Database connection validation failed: {e}")
        return False


def get_connection_metrics() -> dict:
    """
    Get metrics about the current connection state.

    Returns:
        dict: Connection metrics
    """
    # Get the raw pool object to access detailed metrics
    pool = engine.pool

    return {
        "pool_size": getattr(engine.pool, 'size', 'unknown') if hasattr(engine.pool, 'size') else 'unknown',
        "pool_timeout": CONNECTION_TIMEOUT,
        "command_timeout": COMMAND_TIMEOUT,
        "max_overflow": getattr(engine.pool, 'max_overflow', 'unknown') if hasattr(engine.pool, 'max_overflow') else 'unknown',
        "recycle": POOL_RECYCLE,
        "database_url": "***@***"  # Mask the database URL for security
    }

async def init_db() -> None:
    """
    Initialize the database engine and test connection.
    """
    try:
        # Test the connection
        async with engine.connect() as conn:
            # This will raise an exception if unable to connect
            await conn.execute("SELECT 1")
        logger.info("Database connection established successfully")
    except Exception as e:
        logger.error(f"Failed to connect to database: {e}")
        raise

async def close_db() -> None:
    """
    Close the database engine.
    """
    await engine.dispose()
    logger.info("Database engine disposed")

# Connection pool status tracking
class ConnectionPoolStatus:
    def __init__(self):
        self.status = "active"
        self.pool_size = MAX_CONNECTIONS
        self.active_connections = 0
        self.queued_requests = 0

    async def update_status(self):
        """Update connection pool status"""
        # This is a simplified status update
        # In a real implementation, you would query the actual pool status
        pool = engine.pool
        self.pool_size = MAX_CONNECTIONS
        # For SQLAlchemy 2.0, getting active connections is more complex
        # This is a simplified version
        self.status = "active"  # Simplified status

# Global status instance
pool_status = ConnectionPoolStatus()


class ConnectionPoolOptimizer:
    """
    Service to dynamically adjust connection pool settings based on load.
    """

    def __init__(self):
        self.base_max_connections = MAX_CONNECTIONS
        self.base_min_connections = MIN_CONNECTIONS
        self.current_max_connections = MAX_CONNECTIONS
        self.current_min_connections = MIN_CONNECTIONS
        self.optimization_enabled = True

        # Thresholds for optimization
        self.utilization_high_threshold = 75  # Percentage
        self.utilization_low_threshold = 25   # Percentage
        self.response_time_threshold = 150    # ms

        # Adjustment factors
        self.scale_up_factor = 1.5  # Increase by 50% when needed
        self.scale_down_factor = 0.75  # Decrease to 75% when over-provisioned

    def adjust_for_peak_load(self, is_peak: bool, current_utilization: float, avg_response_time: float) -> bool:
        """
        Adjust connection pool settings based on current load conditions.

        Args:
            is_peak: Whether the system is currently under peak load
            current_utilization: Current pool utilization percentage
            avg_response_time: Average response time in ms

        Returns:
            bool: True if adjustments were made, False otherwise
        """
        if not self.optimization_enabled:
            return False

        # Get the current pool metrics to evaluate adjustment needs
        from src.services.connection_pool import connection_pool_service

        # If we're in peak load or utilization is high, consider scaling up
        needs_scale_up = (
            is_peak or
            current_utilization > self.utilization_high_threshold or
            avg_response_time > self.response_time_threshold
        )

        # If utilization is low and we're not in peak, consider scaling down
        needs_scale_down = (
            not is_peak and
            current_utilization < self.utilization_low_threshold and
            avg_response_time < (self.response_time_threshold * 0.5)  # Lower threshold for scale down
        )

        adjustments_made = False

        if needs_scale_up and self.current_max_connections < self.base_max_connections * 2:
            # Scale up the pool size
            new_max = min(
                int(self.current_max_connections * self.scale_up_factor),
                self.base_max_connections * 2  # Don't exceed double the base
            )

            # Only adjust if there's a significant change
            if new_max > self.current_max_connections * 1.1:  # At least 10% increase
                self.current_max_connections = new_max
                self.current_min_connections = max(
                    self.base_min_connections,
                    int(self.current_min_connections * self.scale_up_factor)
                )

                logger.info(f"Scaled up connection pool: max={self.current_max_connections}, min={self.current_min_connections}")
                db_logger.log_debug(
                    f"Connection pool scaled up: max={self.current_max_connections}",
                    "ConnectionOptimizer"
                )
                adjustments_made = True

        elif needs_scale_down:
            # Scale down the pool size
            new_max = max(
                self.base_max_connections,  # Don't go below the configured base
                int(self.current_max_connections * self.scale_down_factor)
            )

            # Only adjust if there's a significant change
            if new_max < self.current_max_connections * 0.9:  # At least 10% decrease
                self.current_max_connections = new_max
                self.current_min_connections = max(
                    self.base_min_connections,
                    int(self.current_min_connections * self.scale_down_factor)
                )

                logger.info(f"Scaled down connection pool: max={self.current_max_connections}, min={self.current_min_connections}")
                db_logger.log_debug(
                    f"Connection pool scaled down: max={self.current_max_connections}",
                    "ConnectionOptimizer"
                )
                adjustments_made = True

        # In a real system, we would need to recreate the engine with new settings
        # For this implementation, we'll just log the intended changes
        if adjustments_made:
            logger.info(f"Pool optimization: max={self.current_max_connections}, min={self.current_min_connections}")

        return adjustments_made

    def get_optimization_status(self) -> dict:
        """
        Get the current status of connection pool optimization.

        Returns:
            dict: Optimization status information
        """
        return {
            "optimization_enabled": self.optimization_enabled,
            "current_max_connections": self.current_max_connections,
            "current_min_connections": self.current_min_connections,
            "base_max_connections": self.base_max_connections,
            "base_min_connections": self.base_min_connections,
            "high_utilization_threshold": self.utilization_high_threshold,
            "low_utilization_threshold": self.utilization_low_threshold,
            "response_time_threshold": self.response_time_threshold
        }


# Global optimizer instance
connection_pool_optimizer = ConnectionPoolOptimizer()


class DatabaseAvailabilityChecker:
    """
    Utility to check database availability and detect unavailability.
    """

    @staticmethod
    async def is_database_available() -> bool:
        """
        Check if the database is currently available by attempting a simple query.

        Returns:
            bool: True if database is available, False otherwise
        """
        try:
            # Try to establish a connection and execute a simple query
            async with engine.connect() as conn:
                await conn.execute("SELECT 1")
            return True
        except Exception as e:
            logger.error(f"Database availability check failed: {e}")
            return False

    @staticmethod
    async def get_connection_pool_metrics() -> dict:
        """
        Get metrics about the connection pool status.

        Returns:
            dict: Metrics about the connection pool
        """
        try:
            # Get the current pool status
            pool = engine.pool
            return {
                "pool_size": MAX_CONNECTIONS,
                "max_overflow": getattr(pool, 'max_overflow', 0),
                "checked_out": getattr(pool, '_checkedout', 0),  # This attribute might vary by SQLAlchemy version
                "database_available": await DatabaseAvailabilityChecker.is_database_available()
            }
        except Exception as e:
            logger.error(f"Error getting connection pool metrics: {e}")
            return {
                "error": str(e),
                "database_available": False
            }


# Create a global instance of the checker
db_availability_checker = DatabaseAvailabilityChecker()