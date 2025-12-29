import asyncio
import sys
import os

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy import text
from src.config.settings import settings
from src.utils.db_url_parser import clean_database_url_for_asyncpg

async def check_table():
    # Use the database URL from settings
    database_url = settings.neon_database_url

    # Clean the database URL for asyncpg
    clean_database_url = clean_database_url_for_asyncpg(database_url)

    if clean_database_url.startswith('postgresql://') and '+' not in clean_database_url.split('://', 1)[1]:
        async_db_url = clean_database_url.replace('postgresql://', 'postgresql+asyncpg://', 1)
    else:
        async_db_url = clean_database_url

    print(f"Using database URL: {async_db_url}")

    engine = create_async_engine(async_db_url)

    try:
        # Query to check if the 'user' table exists and its structure
        async with engine.begin() as conn:
            # Check table existence
            result = await conn.execute(text("""
                SELECT table_name
                FROM information_schema.tables
                WHERE table_schema = 'public'
                AND table_name = 'user'
            """))
            table_exists = result.fetchone()

            if table_exists:
                print('Table "user" exists!')

                # Get column information
                result = await conn.execute(text("""
                    SELECT column_name, data_type, is_nullable, column_default
                    FROM information_schema.columns
                    WHERE table_name = 'user'
                    ORDER BY ordinal_position
                """))

                columns = result.fetchall()
                print('\nTable columns:')
                for col in columns:
                    print(f'  {col[0]}: {col[1]}, nullable: {col[2]}, default: {col[3]}')
            else:
                print('Table "user" does not exist!')

    except Exception as e:
        print(f'Error checking table: {e}')
    finally:
        await engine.dispose()

if __name__ == '__main__':
    asyncio.run(check_table())