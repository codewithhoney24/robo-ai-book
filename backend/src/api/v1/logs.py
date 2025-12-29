from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import JSONResponse
from typing import List, Optional
from datetime import datetime, timedelta
import os
import re
from pydantic import BaseModel

from src.config.logging_config import api_logger, rag_logger, db_logger, embedding_logger, cache_logger, security_logger

router = APIRouter(prefix="/logs", tags=["logs"])

# Pydantic models for logs
class LogEntry(BaseModel):
    timestamp: str
    level: str
    message: str
    logger: str

class LogResponse(BaseModel):
    logs: List[LogEntry]
    total: int
    page: int
    limit: int

# Function to read log files
def read_log_file(
    log_file_path: str,
    limit: int = 100,
    level_filter: Optional[str] = None,
    search_term: Optional[str] = None,
    start_date: Optional[datetime] = None,
    end_date: Optional[datetime] = None
) -> List[LogEntry]:
    """
    Reads log file and returns entries based on filters
    """
    if not os.path.exists(log_file_path):
        return []

    log_entries = []

    try:
        with open(log_file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        # Process lines in reverse to get most recent first
        for line in reversed(lines):
            # Parse log entry - expecting format like: "YYYY-MM-DD HH:MM:SS,mmm - LEVEL - Message"
            match = re.match(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}) - (\w+) - (.+)', line.strip())
            if match:
                timestamp_str, level, message = match.groups()

                # Apply filters
                if level_filter and level != level_filter:
                    continue

                if search_term and search_term.lower() not in message.lower():
                    continue

                # Parse timestamp for date filtering
                timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S,%f')
                if start_date and timestamp < start_date:
                    continue
                if end_date and timestamp > end_date:
                    continue

                log_entries.append(LogEntry(
                    timestamp=timestamp_str,
                    level=level,
                    message=message,
                    logger=os.path.basename(log_file_path)
                ))

                # Limit results
                if len(log_entries) >= limit:
                    break

        # Since we read in reverse to get most recent first, we don't need to reverse again
        return log_entries

    except Exception as e:
        api_logger.error(f"Error reading log file {log_file_path}: {str(e)}")
        return []

@router.get("/", response_model=LogResponse)
async def get_logs(
    limit: int = Query(50, ge=1, le=1000, description="Number of logs to return"),
    offset: int = Query(0, ge=0, description="Offset for pagination"),
    level: Optional[str] = Query(None, description="Filter by log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)"),
    logger_name: Optional[str] = Query(None, description="Filter by logger name (api, rag, db, embedding, cache, security)"),
    search: Optional[str] = Query(None, description="Search term to filter logs"),
    start_date: Optional[datetime] = Query(None, description="Start date for filtering logs"),
    end_date: Optional[datetime] = Query(None, description="End date for filtering logs")
):
    """
    Get application logs with filtering and pagination
    """
    # Determine which log files to read based on logger_name filter
    log_files = []

    if logger_name:
        if logger_name == "api":
            log_files = ["api.log"]
        elif logger_name == "rag":
            log_files = ["rag.log"]
        elif logger_name == "db":
            log_files = ["database_operations.log"]
        elif logger_name == "embedding":
            log_files = ["embedding.log"]
        elif logger_name == "cache":
            log_files = ["cache.log"]
        elif logger_name == "security":
            log_files = ["security.log"]
        else:
            raise HTTPException(status_code=400, detail=f"Unknown logger: {logger_name}")
    else:
        # Read from all log files
        log_files = ["api.log", "rag.log", "database_operations.log", "embedding.log", "cache.log", "security.log"]

    all_logs = []

    # Get logs from each file
    for log_file in log_files:
        log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "logs")
        log_file_path = os.path.join(log_dir, log_file)

        file_logs = read_log_file(
            log_file_path,
            limit=limit,
            level_filter=level,
            search_term=search,
            start_date=start_date,
            end_date=end_date
        )

        all_logs.extend(file_logs)

    # Sort logs by timestamp (most recent first)
    all_logs.sort(key=lambda x: x.timestamp, reverse=True)

    # Apply offset and limit for pagination
    start_idx = offset
    end_idx = min(offset + limit, len(all_logs))
    paginated_logs = all_logs[start_idx:end_idx]

    return LogResponse(
        logs=paginated_logs,
        total=len(all_logs),
        page=offset // limit + 1,
        limit=limit
    )

@router.get("/types")
async def get_log_types():
    """
    Get available log types/systems
    """
    return {
        "log_types": [
            {"name": "api", "description": "API request/response logs"},
            {"name": "rag", "description": "RAG (Retrieval Augmented Generation) logs"},
            {"name": "db", "description": "Database operation logs"},
            {"name": "embedding", "description": "Embedding operation logs"},
            {"name": "cache", "description": "Cache operation logs"},
            {"name": "security", "description": "Security-related logs"}
        ]
    }

@router.get("/health")
async def logs_health():
    """
    Health check for the logs system
    """
    return {
        "status": "healthy",
        "message": "Logs endpoint is operational",
        "timestamp": datetime.utcnow().isoformat()
    }