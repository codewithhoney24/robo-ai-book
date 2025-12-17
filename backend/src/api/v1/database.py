from fastapi import APIRouter, HTTPException, status
from typing import Dict, Any

from src.services.warmup_service import warmup_service
from src.database.connection import DatabaseAvailabilityChecker


router = APIRouter(prefix="/database", tags=["database"])


@router.post("/warmup")
async def trigger_warmup() -> Dict[str, Any]:
    """
    Trigger connection warmup to prevent Neon hibernation.
    
    Forces a connection to Neon to prevent hibernation as specified in the API contracts.
    """
    success = await warmup_service.manual_warmup()
    
    if success:
        return {
            "message": "Connection warmup completed successfully",
            "timestamp": __import__('time').time()
        }
    else:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Connection warmup failed"
        )


@router.get("/status")
async def get_database_status() -> Dict[str, Any]:
    """
    Get detailed database connection status.
    """
    # Check database availability
    is_available = await DatabaseAvailabilityChecker.is_database_available()
    
    # Get warmup service status
    warmup_status = warmup_service.get_status()
    
    # Get connection pool status
    from src.services.connection_pool import connection_pool_service
    pool_status = await connection_pool_service.get_pool_status()
    
    return {
        "database_available": is_available,
        "warmup_service": warmup_status,
        "connection_pool": pool_status,
        "timestamp": __import__('time').time()
    }


@router.get("/availability")
async def check_database_availability() -> Dict[str, Any]:
    """
    Check if the database is available.
    """
    is_available = await DatabaseAvailabilityChecker.is_database_available()
    
    return {
        "available": is_available,
        "timestamp": __import__('time').time()
    }