import asyncio
import logging
import time
from typing import Optional
from contextlib import asynccontextmanager

from sqlalchemy import text
from sqlalchemy.ext.asyncio import AsyncSession

from src.config.settings import settings
from src.database.connection import engine
from src.database.utils import db_logger
from src.core.exceptions import DatabaseConnectionError


logger = logging.getLogger(__name__)


class WarmupService:
    """
    Service to periodically connect to Neon to prevent hibernation.
    Implements the warm-up strategy from research.md.
    """
    
    def __init__(self, interval: Optional[int] = None):
        self.interval = interval or settings.warmup_interval  # Default to setting
        self.is_running = False
        self.last_warmup_time = None
        self.warmup_task: Optional[asyncio.Task] = None
        
        # Validate interval is reasonable
        if self.interval < 10 or self.interval > 300:
            raise ValueError(f"Warmup interval ({self.interval}s) should be between 10-300 seconds")
    
    async def warmup_connection(self) -> bool:
        """
        Execute a lightweight query to keep the connection alive and prevent Neon hibernation.
        
        Returns:
            True if warmup was successful, False otherwise
        """
        start_time = time.time()
        
        try:
            # Create a direct connection to the database using the engine
            async with engine.connect() as conn:
                # Execute a simple query to wake up the database
                result = await conn.execute(text("SELECT 1"))
                
                # Fetch the result to ensure the query completed
                _ = result.fetchone()
                
                # Record successful warmup
                duration = (time.time() - start_time) * 1000  # Convert to milliseconds
                self.last_warmup_time = time.time()
                
                db_logger.log_connection_event(
                    "warmup_success",
                    {"duration_ms": duration, "interval": self.interval}
                )
                
                logger.info(f"Connection warmup completed successfully in {duration:.2f}ms")
                return True
        
        except Exception as e:
            duration = (time.time() - start_time) * 1000  # Convert to milliseconds
            db_logger.log_error(
                e,
                context=f"Connection warmup failed after {duration:.2f}ms"
            )
            
            logger.error(f"Connection warmup failed: {e}")
            return False
    
    async def start(self):
        """
        Start the background warmup task.
        """
        if self.is_running:
            logger.warning("Warmup service is already running")
            return
        
        self.is_running = True
        logger.info(f"Starting connection warmup service with {self.interval}s interval")
        
        async def warmup_loop():
            while self.is_running:
                success = await self.warmup_connection()
                
                if not success:
                    # On failure, try again sooner than the normal interval
                    # This helps recover faster from connection issues
                    failure_retry_delay = min(10, self.interval // 3)  # Max 10s, min of interval/3
                    logger.info(f"Warmup failed, retrying in {failure_retry_delay}s")
                    await asyncio.sleep(failure_retry_delay)
                else:
                    # On success, wait for the full interval
                    await asyncio.sleep(self.interval)
        
        # Create the background task
        self.warmup_task = asyncio.create_task(warmup_loop())
        
        # Log the service start
        db_logger.log_debug(
            f"Warmup service started with {self.interval}s interval",
            "WarmupService"
        )
    
    async def stop(self):
        """
        Stop the background warmup task.
        """
        if not self.is_running:
            logger.info("Warmup service is not running")
            return
        
        self.is_running = False
        
        if self.warmup_task:
            # Cancel the warmup task
            self.warmup_task.cancel()
            
            try:
                # Wait for the task to finish (with timeout)
                await asyncio.wait_for(self.warmup_task, timeout=2.0)
            except asyncio.TimeoutError:
                logger.warning("Warmup task did not finish in time")
            except asyncio.CancelledError:
                # Expected when cancelling the task
                pass
        
        logger.info("Connection warmup service stopped")
        
        # Log the service stop
        db_logger.log_debug("Warmup service stopped", "WarmupService")
    
    def get_status(self) -> dict:
        """
        Get the current status of the warmup service.
        
        Returns:
            Dictionary with status information
        """
        return {
            "is_running": self.is_running,
            "interval": self.interval,
            "last_warmup_time": self.last_warmup_time,
            "last_warmup_ago": time.time() - self.last_warmup_time if self.last_warmup_time else None
        }
    
    async def manual_warmup(self) -> bool:
        """
        Perform a manual warmup operation outside the scheduled interval.
        
        Returns:
            True if warmup was successful, False otherwise
        """
        if not self.is_running:
            logger.info("Warmup service not running, performing one-time warmup")
            return await self.warmup_connection()
        
        # If service is running, just perform a warmup
        return await self.warmup_connection()


# Global instance
warmup_service = WarmupService()