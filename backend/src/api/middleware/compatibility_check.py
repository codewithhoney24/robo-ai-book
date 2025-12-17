from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from ...config.logging_config import api_logger
import re


async def compatibility_check_middleware(request: Request, call_next):
    """
    Middleware to check compatibility across different platforms and browsers
    """
    # Get user agent from request
    user_agent = request.headers.get("user-agent", "")

    # Log the user agent for analytics
    api_logger.debug(f"Request from user agent: {user_agent}")

    # Check if user agent is from a known problematic browser or platform
    # This is a simple example - in a real application, you might want more sophisticated checks
    if _is_potentially_incompatible(user_agent):
        api_logger.warning(f"Potentially incompatible user agent: {user_agent}")
        # For now, we'll just log this - in a real app you might want to respond differently

    # Continue with the request
    response = await call_next(request)

    # Add headers to improve cross-platform compatibility
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["X-XSS-Protection"] = "1; mode=block"

    return response


def _is_potentially_incompatible(user_agent: str) -> bool:
    """
    Check if the user agent is potentially incompatible
    This is a simplified check - in practice, you'd want a more comprehensive list
    """
    # Convert to lowercase for case-insensitive matching
    ua_lower = user_agent.lower()

    # Check for very old browser versions that might not support required features
    # Example: Internet Explorer versions
    if "msie" in ua_lower or "trident" in ua_lower:
        # Check if it's an old version (IE 10 or below)
        match = re.search(r"(msie|trident).*?(\d+)", ua_lower)
        if match and int(match.group(2)) <= 10:
            return True

    # Add other compatibility checks as needed

    return False