from fastapi import APIRouter
from typing import Dict, Any
import time

from src.database.connection import DatabaseAvailabilityChecker, get_connection_metrics
from src.services.connection_pool import connection_pool_service
from src.services.connection_monitor import performance_monitor, recovery_monitor
from src.services.warmup_service import warmup_service
from src.config.settings import settings


router = APIRouter(prefix="/health", tags=["health"])


@router.get("/database")
async def database_health_check() -> Dict[str, Any]:
    """
    Check database connection health.
    
    Returns the health status of the database connection pool as specified in the API contracts.
    """
    # Check if database is available
    is_available = await DatabaseAvailabilityChecker.is_database_available()
    
    # Get connection pool status
    pool_status = await connection_pool_service.get_pool_status()
    
    # Get performance metrics
    perf_stats = performance_monitor.get_current_stats()
    
    # Determine overall status
    status = "healthy"
    if not is_available:
        status = "unavailable"
    elif connection_pool_service.is_degraded():
        status = "degraded"
    
    response = {
        "status": status,
        "pool_size": pool_status["pool_size"],
        "active_connections": pool_status["active_connections"],
        "queued_requests": pool_status["queued_requests"],
        "response_time": perf_stats["avg_response_time"],
        "timestamp": time.time()
    }
    
    # If unavailable, include error information
    if status == "unavailable":
        response["error"] = "Database is currently unavailable"
    
    return response


@router.get("/system")
async def system_health_check() -> Dict[str, Any]:
    """
    Check overall system health including database, performance, and other services.
    """
    # Check all components
    db_available = await DatabaseAvailabilityChecker.is_database_available()
    pool_status = await connection_pool_service.get_pool_status()
    perf_stats = performance_monitor.get_current_stats()
    warmup_status = warmup_service.get_status()
    recovery_status = recovery_monitor.get_recovery_status()
    peak_load_metrics = performance_monitor.get_peak_load_metrics()
    
    # Determine system status
    status = "healthy"
    if not db_available:
        status = "unavailable"
    elif not connection_pool_service.is_pool_healthy() or perf_stats["threshold_exceeded_percentage"] > 10:
        status = "degraded"
    
    return {
        "status": status,
        "database": {
            "available": db_available,
            "connection_pool": pool_status
        },
        "performance": {
            "avg_response_time": perf_stats["avg_response_time"],
            "percentile_95": perf_stats["percentile_95"],
            "within_threshold": perf_stats["within_threshold"],
            "exceeding_threshold": perf_stats["exceeding_threshold"],
            "threshold_exceeded_percentage": perf_stats["threshold_exceeded_percentage"],
            "is_within_threshold": perf_stats["avg_response_time"] <= settings.response_time_threshold
        },
        "services": {
            "warmup_service": warmup_status,
            "recovery_monitor": recovery_status
        },
        "peak_load": {
            "is_peak": peak_load_metrics["is_peak_load"],
            "request_rate_last_min": peak_load_metrics["request_rate_last_min"],
            "request_rate_last_5min": peak_load_metrics["request_rate_last_5min"]
        },
        "connection_metrics": get_connection_metrics(),
        "timestamp": time.time()
    }


@router.get("/detailed")
async def detailed_health_check() -> Dict[str, Any]:
    """
    Get detailed health information about all system components.
    """
    # Gather detailed information about all components
    db_metrics = await DatabaseAvailabilityChecker.get_connection_pool_metrics()
    pool_status = await connection_pool_service.get_pool_status()
    perf_summary = performance_monitor.get_summary()
    warmup_status = warmup_service.get_status()
    recovery_status = recovery_monitor.get_recovery_status()
    peak_metrics = performance_monitor.get_peak_load_metrics()
    
    return {
        "database": db_metrics,
        "connection_pool": pool_status,
        "performance": perf_summary,
        "services": {
            "warmup": warmup_status,
            "recovery": recovery_status
        },
        "peak_load": peak_metrics,
        "application": {
            "app_name": settings.app_name,
            "version": settings.version,
            "debug": settings.debug
        },
        "timestamp": time.time()
    }