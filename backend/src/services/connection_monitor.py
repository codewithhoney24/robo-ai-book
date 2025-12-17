import time
import asyncio
import logging
import statistics
from collections import deque, defaultdict
from typing import Dict, List, Optional, Tuple, Deque
from datetime import datetime, timedelta
from src.config.settings import settings
from src.database.utils import db_logger
from src.core.exceptions import PerformanceThresholdExceededError


logger = logging.getLogger(__name__)


class PerformanceMonitor:
    """
    Service to monitor performance metrics, particularly response times.
    Implements the performance monitoring approach from research.md.
    """
    
    def __init__(self):
        self.response_times: Deque[float] = deque(maxlen=settings.performance_window_size)
        self.window_size = settings.performance_window_size
        self.threshold = settings.response_time_threshold  # 200ms default
        self.start_time = time.time()
        
        # Track metrics by endpoint for more granular monitoring
        self.endpoint_response_times: Dict[str, Deque[float]] = defaultdict(
            lambda: deque(maxlen=settings.performance_window_size)
        )
        
        # Performance statistics cache
        self._stats_cache = {}
        self._last_stats_update = 0
        self._cache_duration = 1  # Cache for 1 second
    
    def add_response_time(self, response_time_ms: float, endpoint: str = "general"):
        """
        Add a response time measurement to the monitoring system.
        
        Args:
            response_time_ms: Response time in milliseconds
            endpoint: API endpoint this measurement is for
        """
        # Add to general response times
        self.response_times.append(response_time_ms)
        
        # Add to endpoint-specific response times
        self.endpoint_response_times[endpoint].append(response_time_ms)
        
        # Log performance metric
        db_logger.log_performance_metric(
            operation=f"api_call_{endpoint}", 
            duration_ms=response_time_ms,
            threshold_ms=self.threshold
        )
    
    def get_percentile(self, percentile: float = 95.0, endpoint: str = "general") -> float:
        """
        Calculate the specified percentile of response times.
        
        Args:
            percentile: Percentile to calculate (e.g., 95.0 for 95th percentile)
            endpoint: API endpoint to get percentile for (default "general" for all)
        
        Returns:
            The response time at the specified percentile
        """
        if endpoint == "general":
            times = list(self.response_times)
        else:
            times = list(self.endpoint_response_times[endpoint])
        
        if len(times) < 10:  # Need at least 10 samples for meaningful percentile
            return 0.0
        
        try:
            # Use quantiles method which is more precise
            if len(times) >= 20:  # statistics.quantiles needs at least n+1 data points
                result = statistics.quantiles(times, n=100, method='inclusive')
                # Get the index that corresponds to the requested percentile
                index = int((percentile / 100) * len(result)) - 1
                index = max(0, min(index, len(result) - 1))
                return result[index]
            else:
                # For smaller datasets, use linear interpolation method
                sorted_times = sorted(times)
                index = (percentile / 100) * (len(sorted_times) - 1)
                idx = int(index)
                
                if idx == len(sorted_times) - 1:
                    return sorted_times[idx]
                else:
                    # Linear interpolation between adjacent values
                    fraction = index - idx
                    lower = sorted_times[idx]
                    upper = sorted_times[idx + 1]
                    return lower + fraction * (upper - lower)
        except Exception:
            # If quantiles calculation fails, fall back to simpler method
            sorted_times = sorted(times)
            index = int((percentile / 100) * len(sorted_times))
            index = min(index, len(sorted_times) - 1)
            return sorted_times[index] if sorted_times else 0.0

    def get_percentile_95(self, endpoint: str = "general") -> float:
        """
        Get the 95th percentile of response times.
        
        Args:
            endpoint: API endpoint to get percentile for (default "general" for all)
            
        Returns:
            The 95th percentile response time
        """
        return self.get_percentile(95.0, endpoint)
    
    def get_current_stats(self, endpoint: str = "general") -> Dict[str, float]:
        """
        Get current performance statistics.
        
        Args:
            endpoint: API endpoint to get stats for (default "general" for all)
            
        Returns:
            Dictionary containing various performance statistics
        """
        # Use caching for performance
        cache_key = f"{endpoint}_{int(time.time() // self._cache_duration)}"
        
        if cache_key != self._stats_cache.get('key'):
            if endpoint == "general":
                times = list(self.response_times)
            else:
                times = list(self.endpoint_response_times[endpoint])
            
            if not times:
                stats = {
                    "avg_response_time": 0.0,
                    "min_response_time": 0.0,
                    "max_response_time": 0.0,
                    "percentile_50": 0.0,
                    "percentile_95": 0.0,
                    "percentile_99": 0.0,
                    "total_requests": 0,
                    "within_threshold": 0,
                    "exceeding_threshold": 0,
                    "threshold_exceeded_percentage": 0.0
                }
            else:
                avg_response = sum(times) / len(times)
                stats = {
                    "avg_response_time": avg_response,
                    "min_response_time": min(times),
                    "max_response_time": max(times),
                    "percentile_50": self.get_percentile(50.0, endpoint),
                    "percentile_95": self.get_percentile(95.0, endpoint),
                    "percentile_99": self.get_percentile(99.0, endpoint),
                    "total_requests": len(times),
                    "within_threshold": len([t for t in times if t <= self.threshold]),
                    "exceeding_threshold": len([t for t in times if t > self.threshold]),
                }
                stats["threshold_exceeded_percentage"] = (
                    stats["exceeding_threshold"] / stats["total_requests"] * 100
                )
            
            self._stats_cache = {"key": cache_key, "stats": stats}
        
        return self._stats_cache["stats"]
    
    def is_within_threshold(self, endpoint: str = "general") -> bool:
        """
        Check if the 95th percentile response time is within the threshold.
        
        Args:
            endpoint: API endpoint to check (default "general" for all)
            
        Returns:
            True if within threshold, False otherwise
        """
        percentile_95 = self.get_percentile_95(endpoint)
        return percentile_95 <= self.threshold
    
    def validate_performance(self, endpoint: str = "general") -> bool:
        """
        Validate if performance meets requirements, raising an exception if not.
        
        Args:
            endpoint: API endpoint to validate (default "general" for all)
            
        Returns:
            True if performance is acceptable
            
        Raises:
            PerformanceThresholdExceededError if performance threshold is exceeded
        """
        if not self.is_within_threshold(endpoint):
            percentile_95 = self.get_percentile_95(endpoint)
            raise PerformanceThresholdExceededError(
                message=f"95th percentile response time ({percentile_95}ms) exceeds threshold ({self.threshold}ms)",
                threshold=self.threshold,
                actual_value=percentile_95
            )
        return True
    
    def get_performance_alerts(self) -> List[Dict[str, any]]:
        """
        Get any performance alerts based on thresholds.
        
        Returns:
            List of alert dictionaries
        """
        alerts = []
        stats = self.get_current_stats()
        
        if stats["percentile_95"] > self.threshold:
            alerts.append({
                "type": "PERFORMANCE_THRESHOLD_EXCEEDED",
                "message": f"95th percentile response time ({stats['percentile_95']}ms) exceeds threshold ({self.threshold}ms)",
                "severity": "HIGH",
                "value": stats["percentile_95"],
                "threshold": self.threshold
            })
        
        if stats["threshold_exceeded_percentage"] > 5:  # Alert if >5% of requests exceed threshold
            alerts.append({
                "type": "HIGH_THRESHOLD_EXCEEDANCE",
                "message": f"{stats['threshold_exceeded_percentage']:.2f}% of requests exceed the {self.threshold}ms threshold",
                "severity": "MEDIUM",
                "value": stats["threshold_exceeded_percentage"],
                "threshold": 5.0
            })
        
        return alerts
    
    def get_summary(self) -> Dict[str, any]:
        """
        Get a summary of performance metrics.
        """
        now = time.time()
        stats = self.get_current_stats()

        return {
            "summary": {
                "uptime_seconds": now - self.start_time,
                "total_requests": stats["total_requests"],
                "avg_response_time": stats["avg_response_time"],
                "percentile_95": stats["percentile_95"],
                "percentile_99": stats["percentile_99"],
                "within_threshold": stats["within_threshold"],
                "exceeding_threshold": stats["exceeding_threshold"],
                "threshold_exceeded_percentage": stats["threshold_exceeded_percentage"],
                "threshold": self.threshold,
                "is_within_threshold": self.is_within_threshold(),
                "window_size": self.window_size
            },
            "alerts": self.get_performance_alerts()
        }

    def detect_peak_load(self, threshold_multiplier: float = 3.0) -> bool:
        """
        Detect if the system is under peak load conditions.
        Peak load is defined as request rate significantly higher than normal.

        Args:
            threshold_multiplier: Multiplier for what constitutes peak load (default 3x)

        Returns:
            True if peak load is detected, False otherwise
        """
        if len(self.response_times) < 20:  # Need sufficient data for comparison
            return False

        # Calculate the request rate in the last minute vs historical average
        import datetime
        from collections import Counter

        # For this implementation, we'll check if recent request rate
        # is significantly higher than average
        recent_count = len([t for t in self.response_times if time.time() - t/1000 < 60])  # Last 60 seconds
        avg_rate = len(self.response_times) / ((time.time() - self.start_time) / 60)  # Requests per minute

        if avg_rate == 0:
            return recent_count > 10  # If no historical avg, consider >10 req/min as peak

        return (recent_count / 60) > (avg_rate * threshold_multiplier)

    def get_peak_load_metrics(self) -> Dict[str, any]:
        """
        Get metrics specifically related to peak load conditions.

        Returns:
            Dictionary containing peak load metrics
        """
        # Calculate request rates
        now = time.time()

        # Request rate in last minute
        last_min_requests = len([t for t in self.response_times if now - t/1000 < 60])

        # Request rate in last 5 minutes
        last_5min_requests = len([t for t in self.response_times if now - t/1000 < 300])

        # Historical average rate (requests per minute)
        elapsed_minutes = (now - self.start_time) / 60 if now - self.start_time > 60 else 1
        avg_rate = len(self.response_times) / elapsed_minutes

        # Detect if currently in peak load
        is_peak = self.detect_peak_load()

        # Calculate utilization percentages
        utilization_metrics = {}
        for endpoint, times in self.endpoint_response_times.items():
            if times:
                # Calculate how many requests exceeded threshold in last minute
                recent_times = [t for t in times if now - t/1000 < 60]
                if recent_times:
                    exceeding = len([t for t in recent_times if t > self.threshold])
                    utilization_metrics[endpoint] = {
                        "recent_request_count": len(recent_times),
                        "threshold_exceeding_count": exceeding,
                        "threshold_exceeding_percentage": (exceeding / len(recent_times)) * 100
                    }

        return {
            "is_peak_load": is_peak,
            "request_rate_last_min": last_min_requests,
            "request_rate_last_5min": last_5min_requests / 5,
            "average_rate": avg_rate,
            "peak_multiplier": (last_min_requests / avg_rate) if avg_rate > 0 else 0,
            "endpoint_utilization": utilization_metrics,
            "total_connections_monitored": len(self.response_times)
        }

    def get_peak_load_alerts(self) -> List[Dict[str, any]]:
        """
        Get alerts related to peak load conditions.

        Returns:
            List of peak load related alerts
        """
        alerts = []
        peak_metrics = self.get_peak_load_metrics()

        # Alert if we're in peak load
        if peak_metrics["is_peak_load"]:
            alerts.append({
                "type": "PEAK_LOAD_DETECTED",
                "message": f"Peak load detected: {peak_metrics['peak_multiplier']:.2f}x normal rate",
                "severity": "MEDIUM",
                "value": peak_metrics["peak_multiplier"],
                "threshold": 3.0  # Peak is 3x normal rate
            })

        # Alert if threshold exceeded percentage is too high during peak
        for endpoint, util in peak_metrics["endpoint_utilization"].items():
            if util["threshold_exceeding_percentage"] > 10:  # More than 10% of requests during peak
                alerts.append({
                    "type": "HIGH_THRESHOLD_EXCEEDANCE_DURING_PEAK",
                    "message": f"High threshold exceedance for {endpoint} during peak: {util['threshold_exceeding_percentage']:.2f}%",
                    "severity": "HIGH",
                    "endpoint": endpoint,
                    "value": util["threshold_exceeding_percentage"],
                    "threshold": 10.0
                })

        return alerts


class DatabaseRecoveryMonitor:
    """
    Service to monitor database availability and handle recovery.
    """

    def __init__(self):
        self.last_recovery_attempt = None
        self.recovery_attempts = 0
        self.max_recovery_attempts = 5
        self.recovery_backoff_base = 1.5  # Base for exponential backoff
        self.database_was_down = False

    async def check_and_handle_recovery(self) -> bool:
        """
        Check database availability and handle recovery if it comes back online.

        Returns:
            bool: True if database is available and recovered, False otherwise
        """
        from src.database.connection import DatabaseAvailabilityChecker

        # Check if database is currently available
        is_available = await DatabaseAvailabilityChecker.is_database_available()

        # If database was previously down and is now available, handle recovery
        if self.database_was_down and is_available:
            db_logger.log_connection_event(
                "recovery_success",
                {"recovery_attempts": self.recovery_attempts}
            )

            # Reset recovery tracking
            self.database_was_down = False
            self.last_recovery_attempt = None
            self.recovery_attempts = 0

            db_logger.log_debug("Database recovery successful", "RecoveryMonitor")
            return True
        elif not is_available:
            # Database is still down or went down
            if not self.database_was_down:
                # This is a new outage
                self.database_was_down = True
                db_logger.log_warning("Database outage detected", "RecoveryMonitor")

            # Check if we should attempt recovery
            should_attempt = self._should_attempt_recovery()
            if should_attempt:
                return await self._attempt_recovery()

        return is_available

    def _should_attempt_recovery(self) -> bool:
        """
        Determine if a recovery attempt should be made based on timing and attempt count.

        Returns:
            bool: True if a recovery attempt should be made
        """
        if self.recovery_attempts >= self.max_recovery_attempts:
            return False

        if self.last_recovery_attempt is None:
            return True  # First attempt

        # Exponential backoff: wait longer between each attempt
        # Formula: base * (base ^ attempt_number)
        backoff_time = self.recovery_backoff_base * (self.recovery_backoff_base ** self.recovery_attempts)

        time_since_last_attempt = time.time() - self.last_recovery_attempt
        return time_since_last_attempt >= backoff_time

    async def _attempt_recovery(self) -> bool:
        """
        Attempt to recover from database unavailability.

        Returns:
            bool: True if recovery was successful, False otherwise
        """
        from src.database.connection import DatabaseAvailabilityChecker

        self.recovery_attempts += 1
        self.last_recovery_attempt = time.time()

        db_logger.log_debug(
            f"Recovery attempt {self.recovery_attempts}/{self.max_recovery_attempts}",
            "RecoveryMonitor"
        )

        # Check database availability
        is_available = await DatabaseAvailabilityChecker.is_database_available()

        if is_available:
            # Recovery successful
            db_logger.log_connection_event(
                "recovery_success",
                {"attempt_number": self.recovery_attempts}
            )

            # Reset recovery tracking
            self.database_was_down = False
            self.last_recovery_attempt = None
            self.recovery_attempts = 0

            return True
        else:
            # Recovery failed, log the failure
            db_logger.log_error(
                Exception(f"Recovery attempt {self.recovery_attempts} failed"),
                context="RecoveryMonitor"
            )

            return False

    def get_recovery_status(self) -> dict:
        """
        Get the current status of the recovery mechanism.

        Returns:
            dict: Recovery status information
        """
        return {
            "database_was_down": self.database_was_down,
            "recovery_attempts": self.recovery_attempts,
            "max_recovery_attempts": self.max_recovery_attempts,
            "last_recovery_attempt": self.last_recovery_attempt,
            "next_recovery_in": self._get_next_recovery_time()
        }

    def _get_next_recovery_time(self) -> Optional[float]:
        """
        Get the time until the next recovery attempt.

        Returns:
            float: Seconds until next attempt, or None if no attempt scheduled
        """
        if not self._should_attempt_recovery() or self.last_recovery_attempt is None:
            return None

        backoff_time = self.recovery_backoff_base * (self.recovery_backoff_base ** self.recovery_attempts)
        time_since_last_attempt = time.time() - self.last_recovery_attempt
        return max(0, backoff_time - time_since_last_attempt)


# Global instances
performance_monitor = PerformanceMonitor()
recovery_monitor = DatabaseRecoveryMonitor()