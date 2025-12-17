# Quickstart: Neon Database Connection & Performance Implementation

## Overview
This guide provides a quick setup for implementing the Neon database connection management in the RAG chatbot system.

## Prerequisites
- Python 3.11+
- FastAPI
- asyncpg
- SQLAlchemy
- Neon Serverless Postgres account

## Setup Instructions

### 1. Install Dependencies
```bash
pip install fastapi asyncpg sqlalchemy[asyncio] python-dotenv pydantic
```

### 2. Environment Configuration
Create a `.env` file with the following variables:
```env
NEON_DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require
MAX_CONNECTIONS=20
MIN_CONNECTIONS=5
CONNECTION_TIMEOUT=10
COMMAND_TIMEOUT=30
RETRY_ATTEMPTS=3
WARMUP_INTERVAL=55
```

### 3. Database Connection Pool Setup
Create `src/database/connection.py`:
```python
import os
import asyncio
from sqlalchemy.ext.asyncio import create_async_engine, async_sessionmaker
from contextlib import asynccontextmanager
from fastapi import FastAPI

# Load configuration from environment
DATABASE_URL = os.getenv("NEON_DATABASE_URL")
MAX_CONNECTIONS = int(os.getenv("MAX_CONNECTIONS", 20))
MIN_CONNECTIONS = int(os.getenv("MIN_CONNECTIONS", 5))
CONNECTION_TIMEOUT = int(os.getenv("CONNECTION_TIMEOUT", 10))
COMMAND_TIMEOUT = int(os.getenv("COMMAND_TIMEOUT", 30))

# Create async engine with connection pooling
engine = create_async_engine(
    DATABASE_URL,
    pool_size=MAX_CONNECTIONS,
    min_size=MIN_CONNECTIONS,
    pool_timeout=CONNECTION_TIMEOUT,
    command_timeout=COMMAND_TIMEOUT,
    pool_recycle=3600,  # Recycle connections every hour
)

# Create async session maker
AsyncSessionLocal = async_sessionmaker(engine, expire_on_commit=False)

@asynccontextmanager
async def get_db_session():
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()
```

### 4. Connection Warmup Service
Create `src/services/warmup_service.py`:
```python
import asyncio
import logging
from sqlalchemy import text
from src.database.connection import engine

logger = logging.getLogger(__name__)

class WarmupService:
    def __init__(self, interval: int = 55):
        self.interval = interval
        self.is_running = False
    
    async def warmup_connection(self):
        """Execute a lightweight query to keep the connection alive"""
        try:
            async with engine.connect() as conn:
                await conn.execute(text("SELECT 1"))
                logger.info("Connection warmup completed successfully")
                return True
        except Exception as e:
            logger.error(f"Connection warmup failed: {e}")
            return False
    
    async def start(self):
        """Start the background warmup task"""
        self.is_running = True
        logger.info(f"Starting connection warmup service with {self.interval}s interval")
        
        while self.is_running:
            await self.warmup_connection()
            await asyncio.sleep(self.interval)
    
    async def stop(self):
        """Stop the background warmup task"""
        self.is_running = False
        logger.info("Connection warmup service stopped")
```

### 5. Integrate with FastAPI App
Update your `main.py`:
```python
from contextlib import asynccontextmanager
from fastapi import FastAPI, Depends
from src.database.connection import get_db_session
from src.services.warmup_service import WarmupService
import asyncio

warmup_service = WarmupService()

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    warmup_task = asyncio.create_task(warmup_service.start())
    yield
    # Shutdown
    await warmup_service.stop()
    warmup_task.cancel()

app = FastAPI(lifespan=lifespan)

@app.get("/health/database")
async def health_check():
    # Implementation here
    pass
```

### 6. Performance Monitoring
Add performance monitoring middleware:
```python
import time
from fastapi import Request
from collections import deque
import statistics

class PerformanceMonitor:
    def __init__(self, window_size: int = 1000):
        self.response_times = deque(maxlen=window_size)
        self.window_size = window_size

    def add_response_time(self, response_time: float):
        self.response_times.append(response_time)

    def get_percentile_95(self) -> float:
        if len(self.response_times) < 10:  # Need at least 10 samples
            return 0.0
        return statistics.quantiles(self.response_times, n=20)[-1]  # 95th percentile

    def is_within_threshold(self, threshold: float = 200.0) -> bool:
        p95 = self.get_percentile_95()
        return p95 <= threshold

performance_monitor = PerformanceMonitor()

@app.middleware("http")
async def performance_monitoring(request: Request, call_next):
    start_time = time.time()
    response = await call_next(request)
    end_time = time.time()
    
    response_time_ms = (end_time - start_time) * 1000
    performance_monitor.add_response_time(response_time_ms)
    
    return response
```

## Key Configuration Notes
- Adjust `MAX_CONNECTIONS` based on your Neon tier limits
- Set `WARMUP_INTERVAL` slightly below Neon's hibernation time (typically 60 seconds)
- The performance monitor tracks 95th percentile response times
- Connection timeouts help prevent resource blocking during outages