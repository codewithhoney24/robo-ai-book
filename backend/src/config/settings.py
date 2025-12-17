from pydantic_settings import BaseSettings
from typing import List, Optional
import os


class Settings(BaseSettings):
    # Qdrant settings
    qdrant_endpoint: str = ""
    qdrant_api_key: str = ""
    qdrant_client_url: str = ""
    qdrant_collection_name: str = "book_content"

    # Database settings
    neon_database_url: str = ""

    # Cohere settings
    cohere_api_key: str = ""
    cohere_embed_model: str = ""

    # OpenAI settings
    OPENAI_API_KEY: str = ""

    # Application settings
    app_name: str = "RAG Chatbot for Published Book"
    debug: bool = False
    version: str = "1.0.0"
    host: str = "127.0.0.1"
    port: int = 8000
    max_query_length: int = 2000
    relevance_threshold: float = 0.7

    # Security settings
    allowed_hosts: List[str] = ["*"]
    secure_headers_enabled: bool = True
    max_session_age_hours: int = 24
    rate_limit_requests: int = 100
    rate_limit_window: int = 60  # in seconds

    # Connection settings
    max_connections: int = 20
    min_connections: int = 5
    connection_timeout: int = 30
    command_timeout: int = 60
    retry_attempts: int = 3
    warmup_interval: int = 60  # seconds

    # Performance settings
    response_time_threshold: float = 0.2  # 200ms
    performance_window_size: int = 100

    # Security and authentication settings
    secret_key: str = ""
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    # Database settings
    database_url: str = "file:./dev.db"  # Default value from .env

    # Authentication settings
    better_auth_secret: str = ""
    better_auth_url: str = "http://localhost:3100"

    # Other settings
    qwen_cli_path: str = ""

    # GitHub OAuth Configuration
    github_client_id: str = ""
    github_client_secret: str = ""

    # Greeting patterns for simple queries
    greeting_patterns: List[str] = ["hi", "hello", "hey", "greetings", "good morning", "good afternoon", "good evening", "good night"]
    max_greeting_query_length: int = 30

    class Config:
        env_file = ".env"
        case_sensitive = False


# Create a single instance of the settings
settings = Settings()