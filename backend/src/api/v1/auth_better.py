
from fastapi import APIRouter, HTTPException, Depends, Request
from pydantic import BaseModel
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
class UserBackground(BaseModel):
    softwareBackground: str
    hardwareBackground: str

class UserSignupRequest(BaseModel):
    email: str
    password: str
    name: str
    softwareBackground: str
    hardwareBackground: str

class UserSigninRequest(BaseModel):
    email: str
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
    emailVerified: bool
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

@router.post("/signup-with-background")
async def signup_with_background(user_data: UserSignupRequest, db: AsyncSession = Depends(get_db_session)):
    try:
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

        # Create access token
        access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
        access_token = create_access_token(
            data={"sub": db_user.email, "user_id": str(db_user.id)}, expires_delta=access_token_expires
        )

        return {
            "user": UserResponse(
                id=str(db_user.id),
                email=db_user.email,
                name=db_user.name,
                softwareBackground=db_user.softwareBackground or db_user.software_background,  # Use the correct field name
                hardwareBackground=db_user.hardware_background or db_user.hardwareBackground,  # Use the correct field name
                emailVerified=db_user.emailVerified,
                created_at=db_user.created_at
            ),
            "access_token": access_token,
            "token_type": "bearer"
        }
    except Exception as e:
        await db.rollback()
        raise HTTPException(status_code=500, detail=f"Signup failed: {str(e)}")

@router.post("/session")
async def signin(user_data: UserSigninRequest, db: AsyncSession = Depends(get_db_session)):
    try:
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
            data={"sub": user.email, "user_id": str(user.id)}, expires_delta=access_token_expires
        )

        return {
            "access_token": access_token,
            "token_type": "bearer",
            "user": UserResponse(
                id=str(user.id),
                email=user.email,
                name=user.name,
                softwareBackground=user.softwareBackground or user.software_background,  # Use the correct field name
                hardwareBackground=user.hardware_background or user.hardwareBackground,  # Use the correct field name
                emailVerified=user.emailVerified,
                created_at=user.created_at
            )
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Signin failed: {str(e)}")

@router.get("/session")
async def get_session(request: Request, db: AsyncSession = Depends(get_db_session)):
    try:
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
                softwareBackground=user.softwareBackground or user.software_background,  # Use the correct field name
                hardwareBackground=user.hardware_background or user.hardwareBackground,  # Use the correct field name
                emailVerified=user.emailVerified,
                created_at=user.created_at
            )
        except jwt.JWTError:
            raise HTTPException(status_code=401, detail="Invalid token")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Session retrieval failed: {str(e)}")

@router.post("/update-background")
async def update_user_background(background: UserBackground, request: Request, db: AsyncSession = Depends(get_db_session)):
    try:
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

            # Update user background fields
            user.softwareBackground = background.softwareBackground  # Use the correct field name
            user.hardware_background = background.hardwareBackground  # Use the correct field name

            await db.commit()
            await db.refresh(user)

            return UserResponse(
                id=str(user.id),
                email=user.email,
                name=user.name,
                softwareBackground=user.softwareBackground or user.software_background,  # Use the correct field name
                hardwareBackground=user.hardware_background or user.hardwareBackground,  # Use the correct field name
                emailVerified=user.emailVerified,
                created_at=user.created_at
            )
        except jwt.JWTError:
            raise HTTPException(status_code=401, detail="Invalid token")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Background update failed: {str(e)}")

@router.delete("/session")
async def signout():
    # In a real implementation, you might want to add the token to a blacklist
    return {"success": True}