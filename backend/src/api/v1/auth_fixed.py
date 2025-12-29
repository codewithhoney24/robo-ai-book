from fastapi import APIRouter, HTTPException, Depends, Request
from pydantic import BaseModel, EmailStr
from typing import Optional
from datetime import datetime, timedelta
import bcrypt
import jwt
import os
from sqlalchemy import select
from src.config.settings import settings
from src.database.connection import get_db_session
from src.models.user import User
from sqlalchemy.ext.asyncio import AsyncSession
import re

router = APIRouter(prefix="/auth", tags=["auth"])

# Pydantic models
class UserSignupRequest(BaseModel):
    email: EmailStr
    password: str
    name: str
    softwareBackground: str
    hardwareBackground: str

class UserSigninRequest(BaseModel):
    email: EmailStr
    password: str

class UserUpdateRequest(BaseModel):
    name: Optional[str] = None
    softwareBackground: Optional[str] = None
    hardwareBackground: Optional[str] = None

class UserResponse(BaseModel):
    id: str
    email: str
    name: str
    softwareBackground: str
    hardwareBackground: str
    emailVerified: bool  # Add email verification status
    created_at: datetime

    class Config:
        from_attributes = True

# JWT token creation
def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=30)
    to_encode.update({"exp": expire})

    # Check if secret key is set
    if not settings.secret_key:
        raise HTTPException(status_code=500, detail="JWT secret key not configured")

    encoded_jwt = jwt.encode(to_encode, settings.secret_key, algorithm=settings.algorithm)
    return encoded_jwt

# Password hashing
def verify_password(plain_password: str, hashed_password: str) -> bool:
    return bcrypt.checkpw(plain_password.encode('utf-8'), hashed_password.encode('utf-8'))

def get_password_hash(password: str) -> str:
    return bcrypt.hashpw(password.encode('utf-8'), bcrypt.gensalt()).decode('utf-8')

# Validate background fields
def validate_background_fields(software_bg: str, hardware_bg: str) -> bool:
    valid_software = ["beginner", "intermediate", "advanced", "python_intermediate", "ros2_developer", "ai_robotics_expert"]
    valid_hardware = ["none", "beginner", "intermediate", "advanced", "no_gpu", "rtx_laptop", "rtx_workstation", "jetson_kit", "cloud"]

    return software_bg in valid_software and hardware_bg in valid_hardware

@router.post("/signup", response_model=UserResponse)
async def signup(user_data: UserSignupRequest, db: AsyncSession = Depends(get_db_session)):
    # Validate background fields
    if not validate_background_fields(user_data.softwareBackground, user_data.hardwareBackground):
        raise HTTPException(status_code=400, detail="Invalid background values provided")

    # Check if user already exists
    existing_user = await db.execute(select(User).filter(User.email == user_data.email))
    existing_user_result = existing_user.scalar_one_or_none()
    if existing_user_result:
        raise HTTPException(status_code=409, detail="User with this email already exists")

    # Validate password strength
    if len(user_data.password) < 8:
        raise HTTPException(status_code=400, detail="Password must be at least 8 characters long")

    # Hash the password
    hashed_password = get_password_hash(user_data.password)

    # Create new user with email verification set to True by default
    db_user = User(
        email=user_data.email,
        password=hashed_password,
        name=user_data.name,
        emailVerified=True,  # Set to True by default (no email verification required)
        softwareBackground=user_data.softwareBackground,  # Use the correct field name
        hardware_background=user_data.hardwareBackground  # Use the correct field name
    )

    db.add(db_user)
    await db.commit()
    await db.refresh(db_user)

    return db_user

@router.post("/signin")
async def signin(user_data: UserSigninRequest, db: AsyncSession = Depends(get_db_session)):
    # Find user by email
    result = await db.execute(select(User).filter(User.email == user_data.email))
    user = result.scalar_one_or_none()

    if not user or not verify_password(user_data.password, user.password):
        raise HTTPException(status_code=401, detail="Incorrect email or password")

    # Check if secret key is set
    if not settings.secret_key:
        raise HTTPException(status_code=500, detail="JWT secret key not configured")

    # Create access token
    access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
    access_token = create_access_token(
        data={"sub": user.email, "user_id": user.id}, expires_delta=access_token_expires
    )

    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": UserResponse(
            id=str(user.id),
            email=user.email,
            name=user.name,
            softwareBackground=user.softwareBackground,  # Use the correct field name
            hardwareBackground=user.hardware_background,  # Use the correct field name
            emailVerified=user.emailVerified,
            created_at=user.created_at
        )
    }

@router.get("/session", response_model=UserResponse)
async def get_session(request: Request, db: AsyncSession = Depends(get_db_session)):
    # Extract token from request
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Not authenticated")

    token = auth_header.split(" ")[1]

    try:
        # Check if secret key is set
        if not settings.secret_key:
            raise HTTPException(status_code=500, detail="JWT secret key not configured")

        # Decode the token
        payload = jwt.decode(token, settings.secret_key, algorithms=[settings.algorithm])
        user_id = payload.get("user_id")

        if user_id is None:
            raise HTTPException(status_code=401, detail="Invalid token")

        # Get user from database
        result = await db.execute(select(User).filter(User.id == user_id))
        user = result.scalar_one_or_none()
        if not user:
            raise HTTPException(status_code=401, detail="User not found")

        return UserResponse(
            id=str(user.id),
            email=user.email,
            name=user.name,
            softwareBackground=user.softwareBackground,  # Use the correct field name
            hardwareBackground=user.hardware_background,  # Use the correct field name
            emailVerified=user.emailVerified,
            created_at=user.created_at
        )
    except jwt.JWTError:
        raise HTTPException(status_code=401, detail="Invalid token")

@router.post("/signout")
async def signout():
    # In a real implementation, you might want to add the token to a blacklist
    return {"success": True}

@router.patch("/user", response_model=UserResponse)
async def update_user(
    user_update: UserUpdateRequest,
    request: Request,
    db: AsyncSession = Depends(get_db_session)
):
    # Extract token from request
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Not authenticated")

    token = auth_header.split(" ")[1]

    try:
        # Check if secret key is set
        if not settings.secret_key:
            raise HTTPException(status_code=500, detail="JWT secret key not configured")

        # Decode the token
        payload = jwt.decode(token, settings.secret_key, algorithms=[settings.algorithm])
        user_id = payload.get("user_id")

        if user_id is None:
            raise HTTPException(status_code=401, detail="Invalid token")

        # Get user from database
        result = await db.execute(select(User).filter(User.id == user_id))
        user = result.scalar_one_or_none()
        if not user:
            raise HTTPException(status_code=401, detail="User not found")

        # Update user fields if provided
        if user_update.name is not None:
            user.name = user_update.name
        if user_update.softwareBackground is not None:
            if user_update.softwareBackground not in ["beginner", "intermediate", "advanced", "python_intermediate", "ros2_developer", "ai_robotics_expert"]:
                raise HTTPException(status_code=400, detail="Invalid software background value")
            user.softwareBackground = user_update.softwareBackground  # Use the correct field name
        if user_update.hardwareBackground is not None:
            if user_update.hardwareBackground not in ["none", "beginner", "intermediate", "advanced", "no_gpu", "rtx_laptop", "rtx_workstation", "jetson_kit", "cloud"]:
                raise HTTPException(status_code=400, detail="Invalid hardware background value")
            user.hardware_background = user_update.hardwareBackground  # Use the correct field name

        await db.commit()
        await db.refresh(user)

        return UserResponse(
            id=str(user.id),
            email=user.email,
            name=user.name,
            softwareBackground=user.softwareBackground,  # Use the correct field name
            hardwareBackground=user.hardware_background,  # Use the correct field name
            emailVerified=user.emailVerified,
            created_at=user.created_at
        )
    except jwt.JWTError:
        raise HTTPException(status_code=401, detail="Invalid token")