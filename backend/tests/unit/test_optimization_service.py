import asyncio
import time
import pytest
from unittest.mock import Mock, patch
from src.services.optimization_service import OptimizationService


class TestOptimizationService:
    """
    Comprehensive tests for OptimizationService to improve test coverage
    """
    
    @pytest.fixture
    def optimization_service(self):
        return OptimizationService()
    
    def test_cache_result_decorator(self, optimization_service):
        """
        Test the cache_result decorator functionality
        """
        call_count = 0
        
        @optimization_service.cache_result(ttl=10)
        def test_function(x):
            nonlocal call_count
            call_count += 1
            return x * 2
        
        # First call - should execute the function
        result1 = test_function(5)
        assert result1 == 10
        assert call_count == 1
        
        # Second call with same argument - should return cached result
        result2 = test_function(5)
        assert result2 == 10
        assert call_count == 1  # Count should not increase
        
        # Call with different argument - should execute the function
        result3 = test_function(10)
        assert result3 == 20
        assert call_count == 2
    
    def test_cache_expiration(self, optimization_service):
        """
        Test cache expiration functionality
        """
        call_count = 0
        
        @optimization_service.cache_result(ttl=0.01)  # 0.01 seconds TTL
        def test_function():
            nonlocal call_count
            call_count += 1
            return "result"
        
        # Call function - should execute
        result1 = test_function()
        assert call_count == 1
        
        # Call again immediately - should return cached result
        result2 = test_function()
        assert call_count == 1
        
        # Wait for cache to expire
        time.sleep(0.02)
        
        # Call again - should execute the function again
        result3 = test_function()
        assert call_count == 2
    
    def test_get_from_cache_hit(self, optimization_service):
        """
        Test getting a value from cache when it exists
        """
        cache_key = "test_key"
        test_value = "cached_value"
        
        # Manually set a value in cache
        optimization_service.cache[cache_key] = test_value
        optimization_service.cache_ttl[cache_key] = time.time() + 10  # Expires in 10 seconds
        
        result = optimization_service.get_from_cache(cache_key)
        assert result == test_value
    
    def test_get_from_cache_miss(self, optimization_service):
        """
        Test getting a value from cache when it doesn't exist
        """
        result = optimization_service.get_from_cache("nonexistent_key")
        assert result is None
    
    def test_get_from_cache_expired(self, optimization_service):
        """
        Test getting a value from cache when it has expired
        """
        cache_key = "expired_key"
        
        # Manually set a value in cache with past expiration time
        optimization_service.cache[cache_key] = "expired_value"
        optimization_service.cache_ttl[cache_key] = time.time() - 1  # Expired 1 second ago
        
        result = optimization_service.get_from_cache(cache_key)
        assert result is None
    
    def test_set_in_cache(self, optimization_service):
        """
        Test setting a value in cache
        """
        cache_key = "new_key"
        test_value = "new_value"
        
        optimization_service.set_in_cache(cache_key, test_value, ttl=5)
        
        assert cache_key in optimization_service.cache
        assert optimization_service.cache[cache_key] == test_value
        assert cache_key in optimization_service.cache_ttl
    
    def test_invalidate_cache_specific_key(self, optimization_service):
        """
        Test invalidating a specific cache key
        """
        # Set up some cache entries
        optimization_service.cache["key1"] = "value1"
        optimization_service.cache["key2"] = "value2"
        optimization_service.cache_ttl["key1"] = time.time() + 10
        optimization_service.cache_ttl["key2"] = time.time() + 10
        
        # Invalidate specific key
        optimization_service.invalidate_cache("key1")
        
        assert "key1" not in optimization_service.cache
        assert "key1" not in optimization_service.cache_ttl
        assert "key2" in optimization_service.cache  # Should still be there
    
    def test_invalidate_cache_all(self, optimization_service):
        """
        Test invalidating entire cache
        """
        # Set up some cache entries
        optimization_service.cache["key1"] = "value1"
        optimization_service.cache["key2"] = "value2"
        optimization_service.cache_ttl["key1"] = time.time() + 10
        optimization_service.cache_ttl["key2"] = time.time() + 10
        
        # Clear entire cache
        optimization_service.invalidate_cache()
        
        assert len(optimization_service.cache) == 0
        assert len(optimization_service.cache_ttl) == 0
    
    def test_batch_process(self, optimization_service):
        """
        Test batch processing functionality
        """
        def process_item(item):
            return item * 2
        
        items = [1, 2, 3, 4, 5]
        results = optimization_service.batch_process(items, process_item, batch_size=2)
        
        expected = [2, 4, 6, 8, 10]
        assert results == expected
    
    def test_batch_process_empty_list(self, optimization_service):
        """
        Test batch processing with empty list
        """
        def process_item(item):
            return item * 2
        
        results = optimization_service.batch_process([], process_item)
        assert results == []
    
    def test_measure_execution_time_decorator(self, optimization_service):
        """
        Test execution time measurement decorator
        """
        @optimization_service.measure_execution_time
        def slow_function():
            time.sleep(0.01)  # Sleep for 10ms
            return "done"
        
        result = slow_function()
        assert result == "done"
    
    def test_cache_result_with_async_function(self, optimization_service):
        """
        Test cache_result decorator with async function
        """
        call_count = 0
        
        @optimization_service.cache_result(ttl=10)
        async def async_test_function(x):
            nonlocal call_count
            call_count += 1
            return x * 3
        
        # Run the async function
        async def run_test():
            # First call - should execute the function
            result1 = await async_test_function(3)
            assert result1 == 9
            assert call_count == 1
            
            # Second call with same argument - should return cached result
            result2 = await async_test_function(3)
            assert result2 == 9
            assert call_count == 1
            
            # Call with different argument - should execute the function
            result3 = await async_test_function(4)
            assert result3 == 12
            assert call_count == 2
        
        asyncio.run(run_test())