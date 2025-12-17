import pytest
from unittest.mock import Mock, patch
from src.services.caching_service import CachingService


def test_cache_get_set():
    """
    Test basic cache set and get functionality
    """
    cache = CachingService()
    
    # Test setting a value
    key = "test_key"
    value = "test_value"
    
    result = cache.set(key, value)
    assert result is True
    
    # Test getting the value
    retrieved = cache.get(key)
    assert retrieved == value


def test_cache_get_nonexistent():
    """
    Test getting a non-existent key
    """
    cache = CachingService()
    
    retrieved = cache.get("nonexistent_key")
    assert retrieved is None


def test_cache_expiry():
    """
    Test cache expiry functionality
    """
    cache = CachingService()
    
    # Set with a TTL of 0.1 seconds
    cache.set("expiring_key", "expiring_value", ttl_seconds=0.1)
    
    # Should be available immediately
    assert cache.get("expiring_key") == "expiring_value"
    
    # Wait for expiry
    import time
    time.sleep(0.2)
    
    # Should be expired and return None
    assert cache.get("expiring_key") is None


def test_cache_delete():
    """
    Test cache deletion functionality
    """
    cache = CachingService()
    
    # Set a value
    cache.set("delete_key", "delete_value")
    assert cache.get("delete_key") == "delete_value"
    
    # Delete the value
    result = cache.delete("delete_key")
    assert result is True
    
    # Should no longer be available
    assert cache.get("delete_key") is None


def test_cache_clear():
    """
    Test cache clearing functionality
    """
    cache = CachingService()
    
    # Set a few values
    cache.set("key1", "value1")
    cache.set("key2", "value2")
    
    # Verify they exist
    assert cache.get("key1") == "value1"
    assert cache.get("key2") == "value2"
    
    # Clear the cache
    result = cache.clear()
    assert result is True
    
    # Should be empty now
    assert cache.get("key1") is None
    assert cache.get("key2") is None


def test_cache_cleanup_expired():
    """
    Test cleanup of expired entries
    """
    cache = CachingService()
    
    # Set an expired value
    cache.set("expired_key", "expired_value", ttl_seconds=0.1)
    import time
    time.sleep(0.2)
    
    # The value should be expired
    assert cache.get("expired_key") is None
    
    # Add a non-expired value
    cache.set("valid_key", "valid_value", ttl_seconds=1)
    
    # Cleanup should remove only the expired one
    removed_count = cache.cleanup_expired()
    assert removed_count == 1
    
    # Valid key should still exist
    assert cache.get("valid_key") == "valid_value"