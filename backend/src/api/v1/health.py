from fastapi import APIRouter
from typing import Dict, Any
import time
import logging

from src.database.connection import DatabaseAvailabilityChecker, get_connection_metrics
from src.services.connection_pool import connection_pool_service
from src.services.connection_monitor import performance_monitor, recovery_monitor
from src.services.warmup_service import warmup_service
from src.config.settings import settings

logger = logging.getLogger(__name__)


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
    try:
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
                "avg_response_time": float(perf_stats["avg_response_time"]) if perf_stats["avg_response_time"] is not None else 0.0,
                "percentile_95": float(perf_stats["percentile_95"]) if perf_stats["percentile_95"] is not None else 0.0,
                "within_threshold": int(perf_stats["within_threshold"]) if perf_stats["within_threshold"] is not None else 0,
                "exceeding_threshold": int(perf_stats["exceeding_threshold"]) if perf_stats["exceeding_threshold"] is not None else 0,
                "threshold_exceeded_percentage": float(perf_stats["threshold_exceeded_percentage"]) if perf_stats["threshold_exceeded_percentage"] is not None else 0.0,
                "is_within_threshold": bool(perf_stats["avg_response_time"] <= settings.response_time_threshold) if perf_stats["avg_response_time"] is not None else False
            },
            "services": {
                "warmup_service": warmup_status,
                "recovery_monitor": recovery_status
            },
            "peak_load": {
                "is_peak": bool(peak_load_metrics["is_peak_load"]) if peak_load_metrics["is_peak_load"] is not None else False,
                "request_rate_last_min": float(peak_load_metrics["request_rate_last_min"]) if peak_load_metrics["request_rate_last_min"] is not None else 0.0,
                "request_rate_last_5min": float(peak_load_metrics["request_rate_last_5min"]) if peak_load_metrics["request_rate_last_5min"] is not None else 0.0
            },
            "connection_metrics": get_connection_metrics(),
            "timestamp": time.time()
        }
    except Exception as e:
        logger.error(f"System health check failed: {str(e)}")
        return {
            "status": "error",
            "error": str(e),
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