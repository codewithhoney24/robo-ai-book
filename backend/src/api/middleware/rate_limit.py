"""
Rate limiting middleware to prevent abuse of the API
"""
import time
from collections import defaultdict, deque
from fastapi import Request, HTTPException
from ...config.logging_config import security_logger
from ...config.settings import settings


class RateLimiter:
    def __init__(self):
        # Dictionary to store request times for each IP
        # Using deque to efficiently manage request times
        self.requests = defaultdict(lambda: deque(maxlen=settings.rate_limit_requests))

    def is_allowed(self, ip_address: str) -> bool:
        """
        Check if a request from the given IP is allowed based on rate limits
        """
        current_time = time.time()

        # Remove requests that are outside the time window
        while self.requests[ip_address] and \
              current_time - self.requests[ip_address][0] > settings.rate_limit_window:
            self.requests[ip_address].popleft()

        # Check if the limit has been exceeded
        if len(self.requests[ip_address]) >= settings.rate_limit_requests:
            return False

        # Add the current request time
        self.requests[ip_address].append(current_time)
        return True


# Global rate limiter instance
rate_limiter = RateLimiter()


async def rate_limit_middleware(request: Request, call_next):
    """
    Middleware to implement rate limiting
    """
    # Get the client's IP address
    client_ip = request.client.host

    if not rate_limiter.is_allowed(client_ip):
        security_logger.warning(f"Rate limit exceeded for IP: {client_ip}")
        raise HTTPException(
            status_code=429,
            detail={
                "error": "RATE_LIMIT_EXCEEDED",
                "message": f"Rate limit exceeded. Maximum {settings.rate_limit_requests} requests per {settings.rate_limit_window} seconds."
            }
        )

    response = await call_next(request)
    return response