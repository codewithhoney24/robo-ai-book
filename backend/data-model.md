# Data Model: Better-Auth with User Background Collection

## Overview

This document defines the data model for the authentication system that collects user's software and hardware background during signup for content personalization in a Physical AI textbook.

## Entity Definitions

### User Entity
- **Description**: Represents a registered user of the platform
- **Fields**:
  - `id` (string, required, unique): User identifier
  - `name` (string, required): User's display name
  - `email` (string, required, unique): User's email address
  - `password_hash` (string, required): Bcrypt hash of user's password
  - `softwareBackground` (string, required, enum): User's software background level
    - Values: "beginner", "python_intermediate", "ros2_developer", "ai_robotics_expert"
  - `hardwareBackground` (string, required, enum): User's hardware availability
    - Values: "no_gpu", "rtx_laptop", "rtx_workstation", "jetson_kit", "cloud"
  - `created_at` (timestamp, required): Account creation time
  - `updated_at` (timestamp, required): Last modification time
- **Validation**:
  - Email must match standard email format
  - Password must be at least 8 characters with complexity requirements
  - Background fields must be from the predefined enum values
- **Relationships**:
  - One-to-many with Session entities
  - One-to-one with (optional) Personalization Profile entity

### Session Entity
- **Description**: Represents an active user session
- **Fields**:
  - `id` (string, required, unique): Session identifier
  - `user_id` (string, required, foreign key to User): Associated user
  - `expires_at` (timestamp, required): Session expiration time
  - `created_at` (timestamp, required): Session creation time
- **Validation**:
  - Session must be linked to a valid user
  - `expires_at` must be in the future
- **Relationships**:
  - Many-to-one with User entity

### Personalization Profile Entity (Optional)
- **Description**: Stores additional user preferences for content personalization
- **Fields**:
  - `id` (string, required, unique): Profile identifier
  - `user_id` (string, required, foreign key to User): Associated user
  - `content_preferences` (JSON, optional): Custom content preferences in JSON format
  - `last_accessed` (timestamp, optional): Last time the user accessed personalized content
  - `created_at` (timestamp, required): Profile creation time
  - `updated_at` (timestamp, required): Last modification time
- **Validation**:
  - `user_id` must reference a valid user
  - `content_preferences` must be valid JSON
- **Relationships**:
  - One-to-one with User entity

## Database Schema

```sql
-- User table (extended from Better-Auth's base user table)
-- Note: Better-Auth handles base fields (id, email, etc.); these are additional fields
ALTER TABLE user 
ADD COLUMN softwareBackground TEXT NOT NULL DEFAULT 'beginner',
ADD COLUMN hardwareBackground TEXT NOT NULL DEFAULT 'no_gpu';

-- Session table (handled by Better-Auth; fields listed for reference)
-- id, user_id, expires_at, created_at

-- Personalization Profile table (optional)
CREATE TABLE user_personalization (
    id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL REFERENCES user(id),
    content_preferences JSONB,
    last_accessed TIMESTAMP,
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW()
);
```

## Validation Rules

1. **User Registration**:
   - All required fields must be provided
   - Email must be unique
   - Background selections must be from the predefined enum values
   - Password must meet minimum strength requirements (8+ characters)

2. **User Updates**:
   - Background fields can be updated after registration
   - Email updates require verification
   - Personal information must continue to meet validation rules

3. **Session Management**:
   - Sessions expire after 7 days of inactivity
   - Maximum of 5 concurrent sessions per user
   - Sessions are invalidated on password change

## State Transitions

### User Account States
1. **Registered**: User account created during signup
2. **Active**: User has logged in at least once
3. **Updated**: User has modified their background information
4. **Inactive**: Account not accessed in >90 days

### Session States
1. **Active**: Valid session with valid user
2. **Expired**: Session timeout reached
3. **Invalidated**: Session explicitly terminated or invalid

## Indexing Strategy

- User table: Index on `email` for fast login lookups
- User table: Index on `softwareBackground` and `hardwareBackground` for personalization queries
- Session table: Index on `user_id` and `expires_at` for session management
- Personalization table: Index on `user_id` for profile lookups