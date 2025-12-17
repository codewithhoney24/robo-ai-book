import os
import logging
from typing import Dict, Any, Optional
from src.config.settings import settings
from src.database.models.config_manager import ConfigurationManagerModel
from src.database.utils import db_logger, validate_configuration_values
from src.core.exceptions import ValidationError


logger = logging.getLogger(__name__)


class ConfigurationManagerService:
    """
    Service to manage configuration settings and validate them.
    Implements the Configuration Manager model from the data model.
    """

    def __init__(self):
        self.settings = settings  # Using the global settings instance

        # Validate configuration values on initialization
        is_valid = validate_configuration_values(
            self.settings.neon_database_url,
            self.settings.retry_attempts
        )

        if not is_valid:
            raise ValidationError(
                message="Configuration validation failed during initialization",
                field="config_validation",
                value="initialization"
            )

    def get_config(self) -> ConfigurationManagerModel:
        """
        Get the current configuration as a Configuration Manager model.
        """
        return ConfigurationManagerModel(
            database_url=self.settings.neon_database_url,
            max_connections=self.settings.max_connections,
            min_connections=self.settings.min_connections,
            connection_timeout=self.settings.connection_timeout,
            command_timeout=self.settings.command_timeout,
            retry_attempts=self.settings.retry_attempts,
            warmup_interval=self.settings.warmup_interval
        )

    def get_database_url(self) -> str:
        """
        Get the database URL from configuration.
        """
        return self.settings.neon_database_url

    def get_connection_settings(self) -> Dict[str, Any]:
        """
        Get connection-related settings.
        """
        return {
            "max_connections": self.settings.max_connections,
            "min_connections": self.settings.min_connections,
            "connection_timeout": self.settings.connection_timeout,
            "command_timeout": self.settings.command_timeout,
        }

    def get_performance_settings(self) -> Dict[str, Any]:
        """
        Get performance-related settings.
        """
        return {
            "response_time_threshold": self.settings.response_time_threshold,
            "performance_window_size": self.settings.performance_window_size,
        }

    def get_application_settings(self) -> Dict[str, Any]:
        """
        Get general application settings.
        """
        return {
            "app_name": self.settings.app_name,
            "debug": self.settings.debug,
            "version": self.settings.version,
            "max_query_length": self.settings.max_query_length,
            "relevance_threshold": self.settings.relevance_threshold,
        }

    def update_setting(self, key: str, value: Any) -> bool:
        """
        Update a specific setting. Note: This only updates the in-memory value,
        not the environment variable. For persistent changes, update the .env file.

        Args:
            key: The setting key to update
            value: The new value for the setting

        Returns:
            bool: True if update was successful, False otherwise
        """
        try:
            # Check if the key exists in settings
            if hasattr(self.settings, key):
                # Validate the new value
                if key == "neon_database_url":
                    if not value.startswith("postgresql://"):
                        raise ValidationError(
                            message=f"Invalid database URL format: {value}",
                            field=key,
                            value=value
                        )
                elif key == "max_connections":
                    if value <= 0 or value > 100:
                        raise ValidationError(
                            message=f"Invalid max_connections value: {value}",
                            field=key,
                            value=value
                        )
                elif key in ["min_connections", "connection_timeout", "command_timeout", "retry_attempts", "warmup_interval"]:
                    # Add additional validation for these fields as needed
                    pass

                # Update the value
                setattr(self.settings, key, value)

                # Log the configuration change
                db_logger.log_debug(f"Configuration updated: {key} = {value}", "ConfigManager")

                return True
            else:
                logger.warning(f"Attempted to update non-existent setting: {key}")
                return False

        except ValidationError as e:
            logger.error(f"Configuration update failed validation: {e}")
            db_logger.log_error(e, context="ConfigManager Update")
            return False
        except Exception as e:
            logger.error(f"Configuration update failed: {e}")
            db_logger.log_error(e, context="ConfigManager Update")
            return False

    def validate_config(self) -> Dict[str, Any]:
        """
        Validate the current configuration and return validation results.
        """
        results = {
            "database_url_valid": self.settings.neon_database_url.startswith("postgresql://"),
            "max_connections_valid": 0 < self.settings.max_connections <= 100,
            "min_connections_valid": 0 <= self.settings.min_connections <= self.settings.max_connections,
            "connection_timeout_valid": 1 <= self.settings.connection_timeout <= 60,
            "command_timeout_valid": 1 <= self.settings.command_timeout <= 300,
            "retry_attempts_valid": 1 <= self.settings.retry_attempts <= 10,
            "warmup_interval_valid": 10 <= self.settings.warmup_interval <= 300,
        }

        # Overall validity
        results["overall_valid"] = all(results.values())

        return results

    def get_env_var(self, key: str, default: Optional[str] = None) -> Optional[str]:
        """
        Get an environment variable value.
        """
        return os.getenv(key, default)

    def set_env_var(self, key: str, value: str) -> bool:
        """
        Set an environment variable (only affects current process).
        """
        try:
            os.environ[key] = value
            db_logger.log_debug(f"Environment variable set: {key}", "ConfigManager")
            return True
        except Exception as e:
            logger.error(f"Failed to set environment variable {key}: {e}")
            db_logger.log_error(e, context="ConfigManager SetEnvVar")
            return False


# Global instance
config_manager_service = ConfigurationManagerService()