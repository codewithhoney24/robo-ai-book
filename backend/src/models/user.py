from sqlalchemy import Column, String, DateTime, Boolean
from sqlalchemy.sql import func
import uuid
from sqlalchemy.ext.declarative import declarative_base

# Create the Base for this model
Base = declarative_base()

class User(Base):
    __tablename__ = "user"  # Note: table name is "user", not "users"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))  # Using text type as in the DB schema
    email = Column(String, unique=True, index=True, nullable=False)
    emailVerified = Column("emailVerified", Boolean, default=False, nullable=False)  # Added email verification field
    name = Column(String, nullable=False)
    password = Column(String)  # Adding this column for authentication purposes
    image = Column(String)  # From actual DB schema
    created_at = Column("createdAt", DateTime, server_default=func.now(), nullable=False)  # Match actual DB column name
    updated_at = Column("updatedAt", DateTime, server_default=func.now(), onupdate=func.now(), nullable=False)  # Match actual DB column name
    softwareBackground = Column(String)  # From actual DB schema
    programmingLevel = Column(String)  # From actual DB schema
    knownLanguages = Column(String)  # From actual DB schema
    aiMlExperience = Column(Boolean)  # From actual DB schema
    rosRoboticsExperience = Column(Boolean)  # From actual DB schema
    hasJetson = Column(Boolean)  # From actual DB schema
    hasRTXGPU = Column(Boolean)  # From actual DB schema
    hasRobotHardware = Column(Boolean)  # From actual DB schema
    simulationOnly = Column(Boolean)  # From actual DB schema
    software_background = Column("software_background", String)  # From actual DB schema
    hardware_background = Column("hardware_background", String)  # From actual DB schema