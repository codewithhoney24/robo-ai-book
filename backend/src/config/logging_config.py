import logging
import sys
from logging import config
from ..config.settings import settings


# Define the logging configuration
LOGGING_CONFIG = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "default": {
            "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        },
        "detailed": {
            "format": "%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s",
        },
    },
    "handlers": {
        "console": {
            "level": "DEBUG" if settings.debug else "INFO",
            "class": "logging.StreamHandler",
            "stream": sys.stdout,
            "formatter": "default",
        },
        "file": {
            "level": "INFO",
            "class": "logging.FileHandler",
            "filename": "app.log",
            "formatter": "detailed",
        },
    },
    "root": {
        "level": "DEBUG" if settings.debug else "INFO",
        "handlers": ["console", "file"],
    },
}


def setup_logging():
    """Setup logging configuration"""
    config.dictConfig(LOGGING_CONFIG)


# Set up logging when this module is imported
setup_logging()


# Create loggers for different parts of the application
rag_logger = logging.getLogger("rag_service")
api_logger = logging.getLogger("api")
db_logger = logging.getLogger("database")
embedding_logger = logging.getLogger("embedding")
cache_logger = logging.getLogger("cache")
security_logger = logging.getLogger("security")