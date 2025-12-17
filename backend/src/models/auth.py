from typing import Optional
from pydantic import BaseModel, EmailStr, Field
from sqlalchemy import Column, String, Integer, DateTime
from sqlalchemy.sql import func
from datetime import datetime

# Pydantic models for request/response validation
class UserCreate(BaseModel):
    name: str = Field(
        ...,
        min_length=1,
        max_length=100,
        description="User's full name",
        example="John Doe"
    )
    email: EmailStr = Field(
        ...,
        description="User's email address for authentication",
        example="user@example.com"
    )
    password: str = Field(
        ...,
        min_length=8,
        max_length=128,
        description="User's password (minimum 8 characters)",
        example="SecurePassword123!"
    )
    software_background: str = Field(
        ...,
        description="User's software background level",
        pattern="^(beginner|python_intermediate|ros2_developer|ai_robotics_expert)$",
        example="beginner"
    )
    hardware_background: str = Field(
        ...,
        description="User's hardware background/available resources",
        pattern="^(no_gpu|rtx_laptop|rtx_workstation|jetson_kit|cloud)$",
        example="no_gpu"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "name": "John Doe",
                "email": "user@example.com",
                "password": "SecurePassword123!",
                "software_background": "beginner",
                "hardware_background": "no_gpu"
            }
        }


class UserUpdate(BaseModel):
    name: Optional[str] = Field(
        None,
        min_length=1,
        max_length=100,
        description="User's full name",
        example="John Doe"
    )
    software_background: Optional[str] = Field(
        None,
        description="User's software background",
        pattern="^(beginner|python_intermediate|ros2_developer|ai_robotics_expert)$",
        example="python_intermediate"
    )
    hardware_background: Optional[str] = Field(
        None,
        description="User's hardware background",
        pattern="^(no_gpu|rtx_laptop|rtx_workstation|jetson_kit|cloud)$",
        example="rtx_laptop"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "name": "John Doe",
                "software_background": "python_intermediate",
                "hardware_background": "rtx_laptop"
            }
        }


class UserLogin(BaseModel):
    email: EmailStr = Field(
        ...,
        description="User's email address for login",
        example="user@example.com"
    )
    password: str = Field(
        ...,
        description="User's password for login",
        example="SecurePassword123!"
    )


class UserResponse(BaseModel):
    id: str = Field(
        ...,
        description="Unique identifier for the user",
        example="123e4567-e89b-12d3-a456-426614174000"
    )
    name: str = Field(
        ...,
        description="User's full name",
        example="John Doe"
    )
    email: EmailStr = Field(
        ...,
        description="User's email address",
        example="user@example.com"
    )
    software_background: str = Field(
        ...,
        description="User's software background level",
        example="beginner"
    )
    hardware_background: str = Field(
        ...,
        description="User's hardware background/available resources",
        example="no_gpu"
    )
    created_at: datetime = Field(
        ...,
        description="Timestamp when the user account was created",
        example="2023-01-01T00:00:00Z"
    )

    class Config:
        from_attributes = True
        json_schema_extra = {
            "example": {
                "id": "123e4567-e89b-12d3-a456-426614174000",
                "name": "John Doe",
                "email": "user@example.com",
                "software_background": "beginner",
                "hardware_background": "no_gpu",
                "created_at": "2023-01-01T00:00:00Z"
            }
        }


class Token(BaseModel):
    access_token: str = Field(
        ...,
        description="JWT access token for authentication",
        example="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
    )
    token_type: str = Field(
        ...,
        description="Type of token (typically 'bearer')",
        example="bearer"
    )


class TokenData(BaseModel):
    email: Optional[str] = Field(
        None,
        description="Email associated with the token",
        example="user@example.com"
    )