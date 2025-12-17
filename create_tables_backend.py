import asyncio
import sys
import os

# Change to backend directory to ensure proper imports
backend_dir = os.path.join(os.path.dirname(__file__), 'backend')
os.chdir(backend_dir)
sys.path.insert(0, os.path.join(backend_dir, 'src'))

# Change to backend directory to load the .env file
os.chdir(backend_dir)

from sqlalchemy.ext.asyncio import create_async_engine
from src.database.models.user import Base
from src.config.settings import settings

async def create_tables():
    # Use the database URL from settings
    database_url = settings.neon_database_url
    
    # Ensure async driver in DATABASE_URL (use asyncpg for PostgreSQL)
    if database_url.startswith("postgresql://") and "+" not in database_url.split("://", 1)[1]:
        async_db_url = database_url.replace("postgresql://", "postgresql+asyncpg://", 1)
    else:
        async_db_url = database_url

    print(f"Using database URL: {async_db_url.replace('@', '[@]').replace('://', '[://]').split('@')[-1] if '@' in async_db_url else async_db_url}")
    
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