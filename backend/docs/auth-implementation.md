# Better-Auth Implementation Guide

## Overview
This document describes the implementation of the Better-Auth authentication system with user background collection for personalized content in the Physical AI textbook platform.

## Features

### 1. User Registration with Background Information
- Collects software background (beginner, python_intermediate, ros2_developer, ai_robotics_expert)
- Collects hardware availability (no_gpu, rtx_laptop, rtx_workstation, jetson_kit, cloud)
- Validates all input fields using Zod schemas
- Enforces password strength (minimum 8 characters)

### 2. User Authentication
- Secure sign-in with email and password
- Remember Me functionality
- Session management with 7-day expiration
- Rate limiting to prevent abuse

### 3. Profile Management
- Allow users to update their background information
- View current profile details
- Change password functionality

### 4. Content Personalization
- Content displayed based on user's software background
- Hardware-appropriate content recommendations
- GPU-intensive content warnings for users with limited hardware

## Architecture

### Backend (Node.js + Better-Auth + Hono)
- **Authentication**: Better-Auth handles user registration, login, and session management
- **API Framework**: Hono provides a lightweight, fast web framework
- **Database**: PostgreSQL with Neon for the backend persistence
- **Validation**: Zod schemas for input validation

### Frontend (Docusaurus + React)
- **Authentication Context**: Manages user state across the application
- **Protected Routes**: Ensures only authenticated users can access certain content
- **Personalization Components**: Display content based on user background

## API Endpoints

### Authentication
- `POST /api/auth/signup` - Create a new user with background info
- `POST /api/auth/signin` - Authenticate a user
- `GET /api/auth/session` - Get current user session
- `POST /api/auth/signout` - Logout user
- `PATCH /api/auth/user` - Update user profile

### Rate Limiting
- Signups: 5 per hour per IP
- Logins: 10 per hour per IP

## Database Schema

The implementation extends Better-Auth's default user table with:
- `softwareBackground`: Text field with allowed values from enum
- `hardwareBackground`: Text field with allowed values from enum

## Security Considerations

1. **Password Security**:
   - bcrypt hashing with 10 rounds
   - Minimum 8-character password requirement
   - Secure HTTP-only cookies for sessions

2. **Rate Limiting**:
   - Prevents brute force attacks
   - Limits on signups and logins per IP

3. **CORS Policy**:
   - Restricted to allowed frontend origins
   - Credentials allowed for authentication

## Frontend Components

### Personalization Logic
- `PersonalizationService.ts`: Core logic for determining content based on user background
- `ProtectedRoute.tsx`: Ensures authentication before accessing content
- `SoftwareBasedContent.tsx`: Displays content based on user's software background
- `HardwareBasedContent.tsx`: Shows appropriate content based on user's hardware
- `LogoutButton.tsx`: Component for logging out

## Environment Variables

### Backend (.env)
- `DATABASE_URL`: PostgreSQL connection string
- `BETTER_AUTH_SECRET`: Secret key for signing JWTs (min 32 chars)
- `BETTER_AUTH_URL`: Base URL for the auth service
- `NODE_ENV`: Environment (development/production)
- `PORT`: Server port (default 3001)
- `FRONTEND_URL`: Allowed origin for CORS (default http://localhost:3000)

### Frontend (.env)
- `REACT_APP_AUTH_URL`: Backend auth service URL
- `REACT_APP_FRONTEND_URL`: Frontend URL
- `NODE_ENV`: Environment (development/production)

## Deployment

### Backend (Vercel)
1. Configure environment variables in Vercel dashboard
2. Deploy the backend service
3. Ensure CORS settings include your frontend URL

### Frontend (GitHub Pages)
1. Update the auth service URL in frontend environment
2. Deploy using GitHub Pages

## Troubleshooting

### Common Issues
- **CORS Errors**: Verify FRONTEND_URL matches your deployed frontend URL
- **Database Connection**: Check DATABASE_URL is properly configured
- **Session Issues**: Ensure BETTER_AUTH_SECRET is consistent across deployments
- **Rate Limiting**: Check if your IP has been temporarily blocked

### Logging
- The backend logs all authentication events
- Check server logs for detailed error information