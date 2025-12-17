# Authentication System Setup

This document explains how to set up and run the authentication system for the AI Robotics Book project.

## Overview

The authentication system uses Better-Auth service running on port 3100 which handles all authentication needs including custom user attributes for personalization.

## Running the Services

### 1. Start the Better-Auth Service

```bash
cd backend/auth-service
npm install
npm run dev
```

This will start the Better-Auth service on `http://localhost:3100`.

### 2. Run the Docusaurus Website

```bash
cd website
npm install
npm run start
```

The website will be available at `http://localhost:3000`.

## API Endpoints

- `POST /api/auth/login` - User login
- `POST /api/auth/register` - User registration with additional fields (handled by Better-Auth)
- `GET /api/auth/session` - Get current session
- `POST /api/auth/signout` - Sign out user

## Environment Variables

For production deployment, set the following environment variables:

- `DATABASE_URL`: Database connection string for Better-Auth
- `BETTER_AUTH_SECRET`: Secret key for Better-Auth (at least 32 characters long)

## Additional User Attributes

The system stores additional user attributes (software background, hardware availability) directly in the Better-Auth user model:
- `softwareBackground`: User's software experience level
- `hardwareBackground`: User's hardware availability
- `name`: User's display name

## Troubleshooting

1. If you get CORS errors, make sure the Better-Auth service is running on port 3100.
2. If registration fails, verify your database connection and environment variables.
3. If custom attributes aren't saved, check the Better-Auth configuration has the custom fields defined.