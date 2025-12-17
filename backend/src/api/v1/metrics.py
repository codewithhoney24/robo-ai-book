from fastapi import APIRouter
from typing import Dict, Any

from src.services.connection_monitor import performance_monitor
from src.config.settings import settings


router = APIRouter(prefix="/metrics", tags=["metrics"])


@router.get("/")
async def get_performance_metrics() -> Dict[str, Any]:
    """
    Get performance metrics for database operations.
    
    Returns current performance metrics as specified in the API contracts.
    """
    current_stats = performance_monitor.get_current_stats()
    
    return {
        "percentile_95": current_stats["percentile_95"],
        "threshold": settings.response_time_threshold,
        "is_within_threshold": performance_monitor.is_within_threshold(),
        "total_requests": current_stats["total_requests"],
        "success_rate": (
            (current_stats["within_threshold"] / current_stats["total_requests"]) * 100
            if current_stats["total_requests"] > 0 else 100
        ),
        "avg_response_time": current_stats["avg_response_time"],
        "min_response_time": current_stats["min_response_time"],
        "max_response_time": current_stats["max_response_time"],
        "percentile_99": current_stats["percentile_99"],
        "window_size": settings.performance_window_size,
        "timestamp": __import__('time').time()
    }


@router.get("/alerts")
async def get_performance_alerts() -> Dict[str, Any]:
    """
    Get any performance alerts based on current metrics.
    """
    alerts = performance_monitor.get_performance_alerts()
    peak_load_alerts = performance_monitor.get_peak_load_alerts()
    
    return {
        "alerts": alerts,
        "peak_load_alerts": peak_load_alerts,
        "total_alerts": len(alerts) + len(peak_load_alerts),
        "timestamp": __import__('time').time()
    }


@router.get("/peak-load")
async def get_peak_load_metrics() -> Dict[str, Any]:
    """
    Get metrics specifically related to peak load conditions.
    """
    peak_metrics = performance_monitor.get_peak_load_metrics()
    
    return {
        **peak_metrics,
        "timestamp": __import__('time').time()
    }


@router.get("/endpoints")
async def get_endpoint_metrics() -> Dict[str, Any]:
    """
    Get performance metrics broken down by API endpoint.
    """
    # Get stats for all monitored endpoints
    endpoint_stats = {}
    for endpoint in performance_monitor.endpoint_response_times.keys():
        endpoint_stats[endpoint] = performance_monitor.get_current_stats(endpoint=endpoint)
    
    return {
        "endpoint_metrics": endpoint_stats,
        "monitored_endpoints": list(performance_monitor.endpoint_response_times.keys()),
        "timestamp": __import__('time').time()
    }