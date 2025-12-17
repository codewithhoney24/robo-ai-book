"""
Caching service for the RAG chatbot application to improve performance
"""
import hashlib
import json
from datetime import datetime, timedelta
from typing import Any, Optional
from ..config.settings import settings
from ..config.logging_config import cache_logger


class CachingService:
    """
    Service for caching frequently accessed data to improve performance
    """
    
    def __init__(self):
        # In-memory cache for simplicity; in production, consider Redis or Memcached
        self._cache = {}
        self._expirations = {}
    
    def get_cache_key(self, *args, **kwargs) -> str:
        """
        Generate a cache key from arguments
        """
        key_content = json.dumps((args, sorted(kwargs.items())), sort_keys=True)
        return hashlib.md5(key_content.encode()).hexdigest()
    
    def get(self, key: str) -> Optional[Any]:
        """
        Retrieve a value from the cache if it exists and is not expired
        """
        if key in self._expirations and datetime.now() > self._expirations[key]:
            # Remove expired entry
            del self._cache[key]
            del self._expirations[key]
            cache_logger.info(f"Cache entry expired for key: {key}")
            return None
        
        if key in self._cache:
            cache_logger.debug(f"Cache hit for key: {key}")
            return self._cache[key]
        
        cache_logger.debug(f"Cache miss for key: {key}")
        return None
    
    def set(self, key: str, value: Any, ttl_seconds: int = 3600) -> bool:
        """
        Store a value in the cache with a time-to-live
        """
        try:
            self._cache[key] = value
            self._expirations[key] = datetime.now() + timedelta(seconds=ttl_seconds)
            cache_logger.debug(f"Cache set for key: {key} with TTL: {ttl_seconds}s")
            return True
        except Exception as e:
            cache_logger.error(f"Error setting cache for key {key}: {str(e)}")
            return False
    
    def delete(self, key: str) -> bool:
        """
        Remove a value from the cache
        """
        if key in self._cache:
            del self._cache[key]
            if key in self._expirations:
                del self._expirations[key]
            cache_logger.debug(f"Cache deleted for key: {key}")
            return True
        return False
    
    def clear(self) -> bool:
        """
        Clear all entries from the cache
        """
        try:
            self._cache.clear()
            self._expirations.clear()
            cache_logger.info("Cache cleared")
            return True
        except Exception as e:
            cache_logger.error(f"Error clearing cache: {str(e)}")
            return False
    
    def cleanup_expired(self) -> int:
        """
        Remove all expired entries from the cache and return the count of removed entries
        """
        now = datetime.now()
        expired_keys = [
            key for key, expiration in self._expirations.items()
            if now > expiration
        ]
        
        for key in expired_keys:
            del self._cache[key]
            del self._expirations[key]
        
        if expired_keys:
            cache_logger.info(f"Cleaned up {len(expired_keys)} expired cache entries")
        
        return len(expired_keys)


# Global instance
caching_service = CachingService()