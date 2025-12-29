import asyncio
import sys
import os

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from sqlalchemy.ext.asyncio import create_async_engine
from src.models.user import Base
from src.config.settings import settings
from src.utils.db_url_parser import clean_database_url_for_asyncpg

async def create_tables():
    # Use the database URL from settings
    database_url = settings.neon_database_url

    # Clean the database URL for asyncpg
    clean_database_url = clean_database_url_for_asyncpg(database_url)

    # Ensure async driver in DATABASE_URL (use asyncpg for PostgreSQL)
    if clean_database_url.startswith("postgresql://") and "+" not in clean_database_url.split("://", 1)[1]:
        async_db_url = clean_database_url.replace("postgresql://", "postgresql+asyncpg://", 1)
    else:
        async_db_url = clean_database_url

    print(f"Using cleaned database URL: {async_db_url.replace('@', '[@]').replace('://', '[://]').split('@')[-1] if '@' in async_db_url else async_db_url}")

    # Create async engine
    engine = create_async_engine(async_db_url)

    try:
        # Create all tables
        async with engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)
        print("Tables created successfully!")
    except Exception as e:
        print(f"Error creating tables: {e}")
    finally:
        await engine.dispose()

if __name__ == "__main__":
    asyncio.run(create_tables())