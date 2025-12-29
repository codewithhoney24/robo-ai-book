#!/usr/bin/env python3
"""
Script to check the columns in the user table
"""
import asyncio
import sys
import os
from sqlalchemy import inspect
from sqlalchemy.ext.asyncio import create_async_engine

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from src.config.settings import settings
from src.utils.db_url_parser import clean_database_url_for_asyncpg


async def check_columns():
    """Check the columns in the user table"""
    # Use the database URL from settings
    database_url = settings.neon_database_url

    # Clean the database URL for asyncpg
    clean_database_url = clean_database_url_for_asyncpg(database_url)

    # Ensure async driver in DATABASE_URL (use asyncpg for PostgreSQL)
    if clean_database_url.startswith("postgresql://") and "+" not in clean_database_url.split("://", 1)[1]:
        async_db_url = clean_database_url.replace("postgresql://", "postgresql+asyncpg://", 1)
    else:
        async_db_url = clean_database_url

    # Create async engine
    engine = create_async_engine(async_db_url)

    try:
        async with engine.begin() as conn:
            from sqlalchemy import text

            # Query the information schema directly to get column info
            result = await conn.execute(text("""
                SELECT column_name, data_type
                FROM information_schema.columns
                WHERE table_name = 'user'
                ORDER BY ordinal_position;
            """))

            print('User table columns:')
            for row in result:
                print(f'  - {row[0]} ({row[1]})')
    finally:
        await engine.dispose()


if __name__ == "__main__":
    asyncio.run(check_columns())