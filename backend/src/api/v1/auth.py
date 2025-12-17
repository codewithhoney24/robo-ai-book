from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from src.database import get_async_session
from src.database.models.user import User
from src.services.auth_service import auth_backend, fastapi_users
from src.models.auth import UserCreate, UserUpdate, UserResponse
from src.services.user_manager import get_user_manager
from fastapi_users.exceptions import UserAlreadyExists

router = APIRouter(prefix="/auth", tags=["auth"])

# Register the authentication routes with FastAPI Users (includes login and logout)
router.include_router(fastapi_users.get_auth_router(auth_backend))

# Register the standard register route
router.include_router(
    fastapi_users.get_register_router(UserResponse, UserCreate),
    prefix="/register",
    tags=["auth"]
)

# Register the reset password routes
router.include_router(fastapi_users.get_reset_password_router())

# Register the verify email routes
router.include_router(fastapi_users.get_verify_router(UserResponse))

# User management router
user_router = APIRouter(tags=["users"])

@user_router.get("/users/me", response_model=UserResponse)
async def get_user_me(user: User = Depends(fastapi_users.current_user())):
    return UserResponse(
        id=str(user.id),
        name=getattr(user, 'name', ''),  # Custom field
        email=user.email,
        software_background=getattr(user, 'software_background', ''),  # Custom field
        hardware_background=getattr(user, 'hardware_background', ''),  # Custom field
        created_at=user.created_at
    )

@user_router.patch("/users/me", response_model=UserResponse)
async def update_user_me(
    user_update: UserUpdate,
    user: User = Depends(fastapi_users.current_user()),
    session: AsyncSession = Depends(get_async_session)
):
    update_data = user_update.dict(exclude_unset=True)

    # Update custom fields
    for field, value in update_data.items():
        if value is not None:
            setattr(user, field, value)

    session.add(user)
    await session.commit()
    await session.refresh(user)

    return UserResponse(
        id=str(user.id),
        name=user.name,
        email=user.email,
        software_background=user.software_background,
        hardware_background=user.hardware_background,
        created_at=user.created_at
    )

router.include_router(user_router)