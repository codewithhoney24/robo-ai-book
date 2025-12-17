from sqlalchemy import Column, String, Integer, DateTime, Boolean
from sqlalchemy.sql import func
from sqlalchemy.ext.declarative import declarative_base
from fastapi_users.db import SQLAlchemyBaseUserTableUUID
from typing import Optional
from pydantic import BaseModel

Base = declarative_base()


class User(SQLAlchemyBaseUserTableUUID, Base):
    """
    User model with additional fields for software and hardware background
    """
    name = Column(String, nullable=False)
    software_background = Column(String, nullable=False)
    hardware_background = Column(String, nullable=False)