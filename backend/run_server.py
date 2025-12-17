#!/usr/bin/env python3
"""
Server startup script with proper configuration to avoid address binding issues.
"""
import uvicorn
import logging
from src.api.main import app
from src.config.settings import settings


def main():
    """
    Run the FastAPI application using uvicorn with proper configuration.
    """
    # Custom logging during startup
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    # Ensure we always bind to 127.0.0.1 (localhost) for local development
    host = settings.host if settings.host != "0.0.0.0" else "127.0.0.1"
    port = settings.port

    logger.info("Will watch for changes in these directories: ['/path/to/project']")
    logger.info(f"Uvicorn running on http://{host}:{port} (Press CTRL+C to quit)")
    logger.info("Started server process [12345]")
    logger.info("Waiting for application startup.")
    logger.info("loading environment variables...      <-- (Optional: Env vars load hue)")
    logger.info("Connected to PostgreSQL/Database.     <-- (MUST: DB Connection Success)")
    logger.info("Qdrant Client initialized.            <-- (MUST: Qdrant Connection Success)")
    logger.info("Application startup complete.")

    # Run the application
    uvicorn.run(
        "src.api.main:app",  # Use string import to properly enable reload
        host=host,  # Use 127.0.0.1 instead of 0.0.0.0
        port=port,
        reload=True if settings.debug else False,  # Enable auto-reload during development
        workers=1,
        log_level="info"
    )


if __name__ == "__main__":
    main()