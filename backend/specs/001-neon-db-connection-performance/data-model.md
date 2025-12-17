# Data Model: Neon Database Connection & Performance

## Overview
This document describes the data structures and models needed to implement Neon database connection management for the RAG chatbot system.

## Key Entities

### 1. Database Connection Pool
- **Purpose**: Manages connections to Neon Serverless Postgres, prevents connection exhaustion, implements timeout and retry logic
- **Attributes**:
  - pool_size (integer): Maximum number of connections in the pool
  - min_pool_size (integer): Minimum number of connections to maintain
  - connection_timeout (integer): Time in seconds before giving up on new connection
  - command_timeout (integer): Time in seconds before canceling command
  - max_lifetime (integer): Maximum lifetime of a connection before recycling
  - pool_recycle (integer): Interval to recycle connections (in seconds)
  - status (string): Current state (active, warming_up, degraded, unavailable)

### 2. Performance Monitor
- **Purpose**: Tracks response times and ensures 95% of non-LLM requests complete within 200ms threshold
- **Attributes**:
  - response_times (list of floats): Recent response times in milliseconds
  - percentile_95 (float): 95th percentile of response times
  - window_size (integer): Number of requests in current measurement window
  - threshold (float): Maximum allowed response time (200ms)
  - is_within_threshold (boolean): Whether performance meets requirements
  - last_check (datetime): Time of last performance check

### 3. Error Handler
- **Purpose**: Manages database unavailability scenarios and connection limit issues, provides graceful degradation
- **Attributes**:
  - error_type (string): Type of error (connection_limit, unavailable, timeout)
  - retry_count (integer): Number of times operation has been retried
  - circuit_open (boolean): Whether circuit breaker is open
  - fallback_response (string): Response to use during errors
  - last_error_time (datetime): Time of last error
  - recovery_strategy (string): Strategy to use for recovery

### 4. Configuration Manager
- **Purpose**: Loads and manages database connection settings from environment variables
- **Attributes**:
  - database_url (string): Connection string for Neon database
  - max_connections (integer): Maximum allowed connections for the app
  - min_connections (integer): Minimum connections to maintain
  - connection_timeout (integer): Timeout for establishing new connections
  - command_timeout (integer): Timeout for database commands
  - retry_attempts (integer): Number of retry attempts for failed operations
  - warmup_interval (integer): Interval for connection warm-up in seconds

## Relationships

```
Configuration Manager --(configures)--> Database Connection Pool
Performance Monitor --(monitors)--> Database Connection Pool
Error Handler --(handles errors from)--> Database Connection Pool
```

## States and Transitions

### Database Connection Pool States:
- **active**: Normal operation, connections available
- **warming_up**: Pool is establishing initial connections
- **degraded**: Connection limits reached, limited service
- **unavailable**: No connections available, service unavailable

Transition rules:
- `warming_up` → `active`: When initial connections established
- `active` → `degraded`: When connection limit reached
- `degraded` → `active`: When connections become available
- `active` → `unavailable`: When no connections available
- `unavailable` → `active`: When connections become available
- `degraded` → `unavailable`: When no connections available

### Error Handler States:
- **normal**: No errors detected
- **retrying**: Operation is being retried
- **circuit_open**: Circuit breaker activated after multiple failures
- **fallback_active**: Using fallback responses

## Validation Rules

1. **Connection Pool Size Validation**:
   - pool_size must be less than Neon connection limit
   - min_pool_size must be less than or equal to pool_size
   - pool_size must be greater than 0

2. **Timeout Validation**:
   - connection_timeout must be between 1-60 seconds
   - command_timeout must be between 1-300 seconds

3. **Performance Threshold Validation**:
   - percentile_95 must not exceed 200ms for normal operation
   - window_size must be at least 100 requests for meaningful measurement

4. **Configuration Validation**:
   - database_url must be a valid PostgreSQL connection string
   - retry_attempts must be between 1-10