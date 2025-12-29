#!/usr/bin/env python3
"""
Test script to verify that the signup functionality works correctly after fixing the sslmode issue.
"""
import asyncio
import sys
import os
from typing import Optional

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy import select
from src.config.settings import settings
from src.models.user import User
from src.api.v1.auth import validate_background_fields, get_password_hash
from src.utils.db_url_parser import clean_database_url_for_asyncpg


async def test_database_connection():
    """Test that the database connection works without sslmode errors"""
    print("Testing database connection...")

    # Clean the database URL before creating the engine
    database_url = settings.neon_database_url
    clean_database_url = clean_database_url_for_asyncpg(database_url)

    # Ensure async driver in DATABASE_URL (use asyncpg for PostgreSQL)
    if clean_database_url.startswith("postgresql://") and "+" not in clean_database_url.split("://", 1)[1]:
        async_db_url = clean_database_url.replace("postgresql://", "postgresql+asyncpg://", 1)
    else:
        async_db_url = clean_database_url

    # Create a new engine with the cleaned URL
    engine = create_async_engine(async_db_url)

    try:
        async with engine.connect() as conn:
            result = await conn.execute(select(1))
            print("[OK] Database connection successful")
            await engine.dispose()  # Clean up the engine
            return True
    except Exception as e:
        print(f"[ERROR] Database connection failed: {e}")
        await engine.dispose()  # Clean up the engine
        return False


async def test_user_creation():
    """Test creating a user in the database"""
    print("\nTesting user creation...")

    # Clean the database URL before creating the engine
    database_url = settings.neon_database_url
    clean_database_url = clean_database_url_for_asyncpg(database_url)

    # Ensure async driver in DATABASE_URL (use asyncpg for PostgreSQL)
    if clean_database_url.startswith("postgresql://") and "+" not in clean_database_url.split("://", 1)[1]:
        async_db_url = clean_database_url.replace("postgresql://", "postgresql+asyncpg://", 1)
    else:
        async_db_url = clean_database_url

    # Create a new engine with the cleaned URL
    engine = create_async_engine(async_db_url)

    # Create a test user
    test_email = "test_signup_fix@example.com"
    test_password = "testpassword123"
    test_name = "Test User"
    test_software_bg = "beginner"
    test_hardware_bg = "no_gpu"

    # Validate background fields
    if not validate_background_fields(test_software_bg, test_hardware_bg):
        print("[ERROR] Background field validation failed")
        await engine.dispose()  # Clean up the engine
        return False

    # Hash the password
    hashed_password = get_password_hash(test_password)

    # Create user in the database
    async with AsyncSession(engine) as session:
        try:
            # Check if user already exists
            existing_user = await session.execute(
                select(User).filter(User.email == test_email)
            )
            existing_user_result = existing_user.scalar_one_or_none()

            if existing_user_result:
                print(f"User {test_email} already exists, skipping creation")
                await engine.dispose()  # Clean up the engine
                return True

            # Create new user
            db_user = User(
                email=test_email,
                password=hashed_password,  # Use the correct field name
                name=test_name,
                softwareBackground=test_software_bg,  # Use the correct field name
                hardware_background=test_hardware_bg  # Use the correct field name
            )

            session.add(db_user)
            await session.commit()
            await session.refresh(db_user)

            print(f"[SUCCESS] User {test_email} created successfully")

            # Verify the user was created
            retrieved_user = await session.execute(
                select(User).filter(User.email == test_email)
            )
            user = retrieved_user.scalar_one_or_none()

            if user and user.email == test_email:
                print(f"[SUCCESS] User verification successful: {user.name}")
                await engine.dispose()  # Clean up the engine
                return True
            else:
                print("[ERROR] User verification failed")
                await engine.dispose()  # Clean up the engine
                return False

        except Exception as e:
            print(f"[ERROR] User creation failed: {e}")
            await session.rollback()
            await engine.dispose()  # Clean up the engine
            return False


async def test_signup_flow():
    """Test the complete signup flow"""
    print("\nTesting signup flow...")

    # Test database connection
    db_ok = await test_database_connection()
    if not db_ok:
        return False

    # Test user creation
    user_ok = await test_user_creation()
    if not user_ok:
        return False

    print("\n[SUCCESS] All signup functionality tests passed!")
    return True


async def cleanup_test_user():
    """Remove the test user from the database"""
    print("\nCleaning up test user...")

    # Clean the database URL before creating the engine
    database_url = settings.neon_database_url
    clean_database_url = clean_database_url_for_asyncpg(database_url)

    # Ensure async driver in DATABASE_URL (use asyncpg for PostgreSQL)
    if clean_database_url.startswith("postgresql://") and "+" not in clean_database_url.split("://", 1)[1]:
        async_db_url = clean_database_url.replace("postgresql://", "postgresql+asyncpg://", 1)
    else:
        async_db_url = clean_database_url

    # Create a new engine with the cleaned URL
    engine = create_async_engine(async_db_url)

    async with AsyncSession(engine) as session:
        try:
            # Find and delete the test user
            result = await session.execute(
                select(User).filter(User.email == "test_signup_fix@example.com")
            )
            user = result.scalar_one_or_none()

            if user:
                await session.delete(user)
                await session.commit()
                print("✓ Test user cleaned up successfully")
            else:
                print("No test user found to clean up")

        except Exception as e:
            # If the table doesn't exist, that's fine - just report it
            if "does not exist" in str(e):
                print("[INFO] Users table does not exist, skipping cleanup")
            else:
                print(f"[ERROR] Cleanup failed: {e}")
            await session.rollback()
        finally:
            await engine.dispose()  # Clean up the engine


async def main():
    """Main test function"""
    print("Starting signup functionality test...")

    success = await test_signup_flow()

    # Always try to clean up
    await cleanup_test_user()

    if success:
        print("\n[SUCCESS] All tests passed! The signup sslmode issue has been fixed.")
        return True
    else:
        print("\n[FAILED] Some tests failed.")
        return False


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)