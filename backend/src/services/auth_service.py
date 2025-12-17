from typing import Optional, Union, Any
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from fastapi_users.db import SQLAlchemyUserDatabase
from fastapi_users import FastAPIUsers
from sqlalchemy.ext.asyncio import AsyncSession
from pydantic import BaseModel
from datetime import datetime, timedelta
from typing import Dict, Optional

from src.database import get_async_session
from src.database.models.user import User
from src.models.auth import UserCreate, UserUpdate, UserLogin, UserResponse
from src.config.settings import settings
from src.services.user_db_service import get_user_db
from src.services.user_manager import get_user_manager

# Authentication backend
from fastapi_users.authentication import AuthenticationBackend, BearerTransport, JWTStrategy


def get_jwt_strategy() -> JWTStrategy:
    return JWTStrategy(
        secret=settings.secret_key,
        lifetime_seconds=settings.access_token_expire_minutes * 60
    )


bearer_transport = BearerTransport(tokenUrl="auth/login")

auth_backend = AuthenticationBackend(
    name="jwt",
    transport=bearer_transport,
    get_strategy=get_jwt_strategy,
)

# Create the FastAPIUsers instance with the custom user manager
fastapi_users = FastAPIUsers[User, str](
    get_user_manager,
    [auth_backend],
)

# Create the FastAPIUsers instance with authentication
current_active_user = fastapi_users.current_user(active=True)