import asyncio
import sys
import os
from sqlalchemy.ext.asyncio import create_async_engine
from urllib.parse import urlparse

# Add the project root directory to Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from src.database.models.user import Base
from src.config.settings import settings


def make_async_compatible_url(url):
    """
    Convert a standard PostgreSQL URL to asyncpg compatible format
    Extract query parameters separately to avoid conflicts
    """
    # Parse the URL
    parsed = urlparse(url)

    # Rebuild the URL without query parameters
    password_part = f":{parsed.password}" if parsed.password else ""
    port_part = f":{parsed.port}" if parsed.port else ""

    async_url = f"postgresql+asyncpg://{parsed.username}{password_part}@{parsed.hostname}{port_part}{parsed.path}"

    return async_url


# Make the database URL compatible with asyncpg
async_db_url = make_async_compatible_url(settings.neon_database_url)


async def init_db_tables():
    """Initialize database tables"""
    engine = create_async_engine(async_db_url)

    async with engine.begin() as conn:
        # Create all tables
        await conn.run_sync(Base.metadata.create_all)

    await engine.dispose()
    print("Database tables created successfully!")


if __name__ == "__main__":
    asyncio.run(init_db_tables())