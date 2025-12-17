from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.pool import NullPool
from typing import AsyncGenerator
import logging
from contextlib import contextmanager
import psycopg2
from qdrant_client import QdrantClient
from .settings import settings

logger = logging.getLogger(__name__)

# ✅ CRITICAL FIX: Check if database URL exists
if not settings.neon_database_url or settings.neon_database_url.strip() == "":
    raise ValueError(
        "❌ NEON_DATABASE_URL is missing or empty in .env file!\n"
        "Please add: NEON_DATABASE_URL=postgresql://your_connection_string"
    )

# Convert database URLs to work with async drivers
DATABASE_URL = settings.neon_database_url
if DATABASE_URL.startswith("postgresql://"):
    DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://", 1)
elif DATABASE_URL.startswith("postgres://"):
    DATABASE_URL = DATABASE_URL.replace("postgres://", "postgresql+asyncpg://", 1)
elif DATABASE_URL.startswith("sqlite://"):
    # For SQLite, use aiosqlite for async operations
    DATABASE_URL = DATABASE_URL.replace("sqlite://", "sqlite+aiosqlite://", 1)

# Create async engine
engine = create_async_engine(
    DATABASE_URL,
    echo=settings.debug,
    pool_pre_ping=True,
    poolclass=NullPool if settings.debug else None,
    connect_args={
        "server_settings": {
            "application_name": settings.app_name,
            "jit": "off"
        },
        "command_timeout": settings.command_timeout,
        "timeout": settings.connection_timeout,
    }
)

# Session maker
async_session_maker = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False,
    autocommit=False,
    autoflush=False
)

# Dependency for FastAPI
async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """Database session dependency"""
    async with async_session_maker() as session:
        try:
            yield session
            await session.commit()
        except Exception:
            await session.rollback()
            raise
        finally:
            await session.close()

# Database availability checker (simplified)
class DatabaseAvailabilityChecker:
    """Simple database health checker"""

    @staticmethod
    async def check() -> bool:
        """Check if database is available"""
        try:
            async with engine.connect() as conn:
                await conn.execute("SELECT 1")
            return True
        except Exception as e:
            logger.error(f"Database check failed: {e}")
            return False

# Connection metrics (simplified)
def get_connection_metrics() -> dict:
    """Get basic connection info"""
    return {
        "pool_size": settings.max_connections,
        "min_connections": settings.min_connections,
        "status": "configured"
    }

class DatabaseConfig:
    """Configuration class that provides database connection methods"""

    def __init__(self):
        # Initialize Qdrant client
        if settings.qdrant_api_key:
            self.qdrant_client = QdrantClient(
                url=settings.qdrant_endpoint,
                api_key=settings.qdrant_api_key,
                timeout=10
            )
        else:
            # For local Qdrant instance
            self.qdrant_client = QdrantClient(
                url=settings.qdrant_client_url,
                timeout=10
            )

    def get_qdrant_client(self):
        """Get Qdrant client instance"""
        return self.qdrant_client

    def get_postgres_connection(self):
        """Get Postgres database connection"""
        return psycopg2.connect(
            dsn=settings.neon_database_url,
            connect_timeout=settings.connection_timeout
        )

    @contextmanager
    def get_postgres_cursor(self):
        """Context manager for Postgres database cursor"""
        conn = self.get_postgres_connection()
        try:
            with conn.cursor() as cursor:
                yield cursor
            conn.commit()
        except Exception:
            conn.rollback()
            raise
        finally:
            conn.close()

# Create global instance
db_config = DatabaseConfig()