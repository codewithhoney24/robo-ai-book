from typing import Optional
from fastapi import Depends, Request
from fastapi_users import BaseUserManager, UUIDIDMixin
from fastapi_users.db import SQLAlchemyUserDatabase
from fastapi_users.exceptions import UserAlreadyExists
from sqlalchemy.ext.asyncio import AsyncSession

from src.database.models.user import User
from src.database import get_async_session
from src.services.user_db_service import get_user_db
from src.config.settings import settings


class UserManager(UUIDIDMixin, BaseUserManager[User, str]):
    def __init__(self, user_db):
        super().__init__(user_db)
        self.reset_password_token_secret = settings.secret_key
        self.verification_token_secret = settings.secret_key

    async def on_after_register(self, user: User, request: Optional[Request] = None):
        print(f"User {user.id} has registered.")

    async def create(self, create_dict: dict):
        """
        Create a user with custom fields
        """
        create_dict.setdefault('name', create_dict.get('email', '').split('@')[0])
        create_dict.setdefault('software_background', 'beginner')
        create_dict.setdefault('hardware_background', 'no_gpu')
        return await super().create(create_dict)


async def get_user_manager(user_db: SQLAlchemyUserDatabase = Depends(get_user_db)):
    yield UserManager(user_db)