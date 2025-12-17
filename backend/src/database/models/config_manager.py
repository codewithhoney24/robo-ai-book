from pydantic import BaseModel, validator
from typing import Optional


class ConfigurationManagerModel(BaseModel):
    """
    Pydantic model for Configuration Manager entity.
    This represents the configuration settings loaded from environment variables.
    """
    database_url: str
    max_connections: int
    min_connections: int
    connection_timeout: int  # in seconds
    command_timeout: int     # in seconds
    retry_attempts: int
    warmup_interval: int     # in seconds

    @validator('database_url')
    def validate_database_url(cls, v):
        if not v.startswith("postgresql://"):
            raise ValueError('Database URL must use PostgreSQL protocol')
        return v

    @validator('max_connections')
    def validate_max_connections(cls, v):
        if v <= 0:
            raise ValueError('max_connections must be greater than 0')
        if v > 100:  # Assuming a reasonable upper limit
            raise ValueError('max_connections seems unreasonably high')
        return v

    @validator('min_connections')
    def validate_min_connections(cls, v, values):
        max_conn = values.get('max_connections', 20)
        if v < 0:
            raise ValueError('min_connections must be non-negative')
        if v > max_conn:
            raise ValueError('min_connections cannot exceed max_connections')
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

    @validator('retry_attempts')
    def validate_retry_attempts(cls, v):
        if v < 1 or v > 10:
            raise ValueError('retry_attempts must be between 1-10')
        return v

    @validator('warmup_interval')
    def validate_warmup_interval(cls, v):
        if v < 10 or v > 300:
            raise ValueError('warmup_interval should be between 10-300 seconds')
        return v