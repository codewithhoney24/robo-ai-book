import asyncio
import sys
import os
from urllib.parse import urlparse, parse_qs, urlencode, urlunparse

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from sqlalchemy.ext.asyncio import create_async_engine
from src.database.models.user import Base
from src.config.settings import settings

async def create_tables():
    # Use the database URL from settings
    database_url = settings.neon_database_url

    # Parse the database URL to handle special parameters
    parsed = urlparse(database_url)

    # Extract query parameters
    query_params = parse_qs(parsed.query)

    # Remove channel_binding and sslmode if present since asyncpg doesn't support them directly in the URL
    if 'channel_binding' in query_params:
        del query_params['channel_binding']
    if 'sslmode' in query_params:
        del query_params['sslmode']

    # Rebuild query string without problematic parameters
    new_query = urlencode(query_params, doseq=True)
    new_parsed = parsed._replace(query=new_query)
    clean_database_url = urlunparse(new_parsed)

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