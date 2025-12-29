#!/usr/bin/env python3
"""
Script to add user profile columns (software_background, hardware_background, name)
to the database if they don't exist, directly using SQLAlchemy.
This script is an alternative to running database migrations when they run out of memory.
"""
import asyncio
import sys
import os
from sqlalchemy import text
from sqlalchemy.ext.asyncio import create_async_engine

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Add backend directory to path as well
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.config.settings import settings
from src.utils.db_url_parser import clean_database_url_for_asyncpg


async def add_user_columns():
    """Add user profile columns if they don't exist"""

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
        async with engine.begin() as conn:
            # Add software_background column if it doesn't exist
            await conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1
                        FROM information_schema.columns
                        WHERE table_name = 'user'
                        AND column_name = 'software_background'
                    ) THEN
                        ALTER TABLE "user" ADD COLUMN "software_background" TEXT DEFAULT 'beginner';
                        RAISE NOTICE 'Added software_background column to user table';
                    ELSE
                        RAISE NOTICE 'software_background column already exists';
                    END IF;
                END
                $$;
            """))

            # Add hardware_background column if it doesn't exist
            await conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1
                        FROM information_schema.columns
                        WHERE table_name = 'user'
                        AND column_name = 'hardware_background'
                    ) THEN
                        ALTER TABLE "user" ADD COLUMN "hardware_background" TEXT DEFAULT 'no_gpu';
                        RAISE NOTICE 'Added hardware_background column to user table';
                    ELSE
                        RAISE NOTICE 'hardware_background column already exists';
                    END IF;
                END
                $$;
            """))

            # Add name column if it doesn't exist
            await conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1
                        FROM information_schema.columns
                        WHERE table_name = 'user'
                        AND column_name = 'name'
                    ) THEN
                        ALTER TABLE "user" ADD COLUMN "name" TEXT DEFAULT '';
                        RAISE NOTICE 'Added name column to user table';
                    ELSE
                        RAISE NOTICE 'name column already exists';
                    END IF;
                END
                $$;
            """))

            # Create indexes for better query performance
            await conn.execute(text("""
                CREATE INDEX IF NOT EXISTS "user_email_idx" ON "user"("email");
            """))

            await conn.execute(text("""
                CREATE INDEX IF NOT EXISTS "user_software_background_idx" ON "user"("software_background");
            """))

            await conn.execute(text("""
                CREATE INDEX IF NOT EXISTS "user_hardware_background_idx" ON "user"("hardware_background");
            """))

            # Update any existing users with missing profile data
            await conn.execute(text("""
                UPDATE "user"
                SET "software_background" = 'beginner'
                WHERE "software_background" IS NULL OR "software_background" = '';
            """))

            await conn.execute(text("""
                UPDATE "user"
                SET "hardware_background" = 'no_gpu'
                WHERE "hardware_background" IS NULL OR "hardware_background" = '';
            """))

            await conn.execute(text("""
                UPDATE "user"
                SET "name" = email
                WHERE "name" IS NULL OR "name" = '';
            """))

            await conn.commit()
            print("User profile columns added successfully!")

    except Exception as e:
        print(f"Error adding user columns: {e}")
        await conn.rollback()
    finally:
        await engine.dispose()


if __name__ == "__main__":
    asyncio.run(add_user_columns())