import asyncio
import time
from typing import List, Any, Callable
from functools import wraps
from ..config.logging_config import api_logger


class OptimizationService:
    """
    Service for performance optimizations in the web interface
    """
    
    def __init__(self):
        self.cache = {}
        self.cache_ttl = {}  # Time-to-live for cache entries
        self.default_cache_ttl = 300  # 5 minutes in seconds
    
    def cache_result(self, ttl: int = None):
        """
        Decorator to cache the result of a function
        """
        def decorator(func: Callable) -> Callable:
            @wraps(func)
            async def async_wrapper(*args, **kwargs):
                # Create a cache key based on function name and arguments
                cache_key = f"{func.__name__}:{str(args)}:{str(kwargs)}"
                
                # Check if result is in cache and not expired
                current_time = time.time()
                if cache_key in self.cache and current_time < self.cache_ttl.get(cache_key, 0):
                    api_logger.debug(f"Cache hit for {cache_key}")
                    return self.cache[cache_key]
                
                # Call the function and cache the result
                result = await func(*args, **kwargs)
                
                # Determine TTL for this cache entry
                actual_ttl = ttl if ttl is not None else self.default_cache_ttl
                self.cache[cache_key] = result
                self.cache_ttl[cache_key] = current_time + actual_ttl
                
                api_logger.debug(f"Cached result for {cache_key}")
                return result
            
            @wraps(func)
            def sync_wrapper(*args, **kwargs):
                # Create a cache key based on function name and arguments
                cache_key = f"{func.__name__}:{str(args)}:{str(kwargs)}"
                
                # Check if result is in cache and not expired
                current_time = time.time()
                if cache_key in self.cache and current_time < self.cache_ttl.get(cache_key, 0):
                    api_logger.debug(f"Cache hit for {cache_key}")
                    return self.cache[cache_key]
                
                # Call the function and cache the result
                result = func(*args, **kwargs)
                
                # Determine TTL for this cache entry
                actual_ttl = ttl if ttl is not None else self.default_cache_ttl
                self.cache[cache_key] = result
                self.cache_ttl[cache_key] = current_time + actual_ttl
                
                api_logger.debug(f"Cached result for {cache_key}")
                return result
            
            # Return the appropriate wrapper based on whether the function is async
            if asyncio.iscoroutinefunction(func):
                return async_wrapper
            else:
                return sync_wrapper
        
        return decorator
    
    def get_from_cache(self, cache_key: str) -> Any:
        """
        Get a value from the cache if it exists and is not expired
        """
        current_time = time.time()
        if cache_key in self.cache and current_time < self.cache_ttl.get(cache_key, 0):
            api_logger.debug(f"Cache hit for {cache_key}")
            return self.cache[cache_key]
        else:
            api_logger.debug(f"Cache miss for {cache_key}")
            return None
    
    def set_in_cache(self, cache_key: str, value: Any, ttl: int = None):
        """
        Set a value in the cache with an optional TTL
        """
        actual_ttl = ttl if ttl is not None else self.default_cache_ttl
        current_time = time.time()
        
        self.cache[cache_key] = value
        self.cache_ttl[cache_key] = current_time + actual_ttl
        
        api_logger.debug(f"Stored in cache: {cache_key}")
    
    def invalidate_cache(self, cache_key: str = None):
        """
        Invalidate cache entries
        If cache_key is None, clear the entire cache
        """
        if cache_key is None:
            self.cache.clear()
            self.cache_ttl.clear()
            api_logger.info("Cleared entire cache")
        else:
            if cache_key in self.cache:
                del self.cache[cache_key]
                if cache_key in self.cache_ttl:
                    del self.cache_ttl[cache_key]
                api_logger.info(f"Cleared cache for {cache_key}")
    
    def batch_process(self, items: List[Any], process_func: Callable, batch_size: int = 10) -> List[Any]:
        """
        Process a list of items in batches to optimize performance
        """
        results = []
        
        for i in range(0, len(items), batch_size):
            batch = items[i:i+batch_size]
            batch_results = [process_func(item) for item in batch]
            results.extend(batch_results)
        
        return results
    
    def measure_execution_time(self, func: Callable) -> Callable:
        """
        Decorator to measure and log execution time of functions
        """
        @wraps(func)
        def wrapper(*args, **kwargs):
            start_time = time.time()
            result = func(*args, **kwargs)
            end_time = time.time()
            
            execution_time = (end_time - start_time) * 1000  # Convert to milliseconds
            api_logger.info(f"{func.__name__} executed in {execution_time:.2f}ms")
            
            return result
        
        return wrapper


# Global instance
optimization_service = OptimizationService()