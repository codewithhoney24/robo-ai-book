from sqlalchemy import Column, Integer, String, DateTime, Float
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
from typing import Optional
from pydantic import BaseModel, validator
from datetime import datetime


class ConnectionPoolModel(BaseModel):
    """
    Pydantic model for Database Connection Pool entity.
    This represents the configuration and state of the connection pool.
    """
    pool_size: int
    min_pool_size: int
    connection_timeout: int  # in seconds
    command_timeout: int     # in seconds
    max_lifetime: int        # in seconds
    pool_recycle: int        # in seconds
    status: str  # active, warming_up, degraded, unavailable

    @validator('pool_size')
    def validate_pool_size(cls, v):
        if v <= 0:
            raise ValueError('pool_size must be greater than 0')
        # Assuming Neon free tier limit, adjust as needed
        if v > 100:
            raise ValueError('pool_size exceeds reasonable limits')
        return v

    @validator('min_pool_size')
    def validate_min_pool_size(cls, v, values):
        pool_size = values.get('pool_size', 20)
        if v < 0:
            raise ValueError('min_pool_size must be non-negative')
        if v > pool_size:
            raise ValueError('min_pool_size cannot exceed pool_size')
        return v

    @validator('connection_timeout')
    def validate_connection_timeout(cls, v):
        if v < 1 or v > 60:
            raise ValueError('connection_timeout must be between 1-60 seconds')
        return v

    @validator('command_timeout')
    def validate_command_timeout(cls, v):
        if v < 1 or v > 300:
            raise ValueError('command_timeout must be between 1-300 seconds')
        return v

    @validator('status')
    def validate_status(cls, v):
        allowed_statuses = ['active', 'warming_up', 'degraded', 'unavailable']
        if v not in allowed_statuses:
            raise ValueError(f'status must be one of {allowed_statuses}')
        return v


# SQLAlchemy model if needed for ORM operations
Base = declarative_base()


class SQLAlchemyConnectionPool(Base):
    """
    SQLAlchemy model for connection pool if persistence is needed.
    Note: This is typically not persisted for connection pool data,
    but provided as an example if needed.
    """
    __tablename__ = 'connection_pools'

    id = Column(Integer, primary_key=True, index=True)
    pool_size = Column(Integer, nullable=False)
    min_pool_size = Column(Integer, nullable=False)
    connection_timeout = Column(Integer, nullable=False)  # in seconds
    command_timeout = Column(Integer, nullable=False)     # in seconds
    max_lifetime = Column(Integer, nullable=False)        # in seconds
    pool_recycle = Column(Integer, nullable=False)        # in seconds
    status = Column(String(20), nullable=False)           # active, warming_up, degraded, unavailable
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())