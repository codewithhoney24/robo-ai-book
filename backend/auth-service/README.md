# Better-Auth Service with GitHub OAuth

This service implements authentication using Better-Auth with GitHub OAuth support for our AI robotics platform.

## Features

- Email/password authentication
- GitHub OAuth authentication
- Session management
- User profile management
- Custom user fields (software/hardware background)

## Setup and Development

1. Install dependencies: `npm install`
2. Create a `.env` file with the required environment variables
3. Run the development server: `npm run dev`

## GitHub OAuth Setup

To enable GitHub OAuth:

1. Go to GitHub Developer Settings: https://github.com/settings/developers
2. Create a new OAuth application:
   - Set the homepage URL to your deployment URL (e.g., https://yourdomain.com)
   - Set the authorization callback URL to: `https://yourdomain.com/api/auth/login/github/callback`
3. Copy the Client ID and Client Secret
4. Add them to your `.env` file as `GITHUB_CLIENT_ID` and `GITHUB_CLIENT_SECRET`

## Environment Variables

- `DATABASE_URL`: Database connection string
- `BETTER_AUTH_SECRET`: Secret key for encryption
- `GITHUB_CLIENT_ID`: GitHub OAuth Client ID (optional)
- `GITHUB_CLIENT_SECRET`: GitHub OAuth Client Secret (optional)

## API Endpoints

- `POST /api/auth/login`: Login with email and password
- `POST /api/auth/register`: Register new user
- `GET /api/auth/me`: Get current user info
- `POST /api/auth/verify`: Verify session
- `POST /api/auth/forgot-password`: Request password reset
- `POST /api/auth/reset-password`: Reset password
- `GET /api/auth/login/github`: GitHub OAuth login endpoint
- `POST /api/auth/login/github/callback`: GitHub OAuth callback endpoint

## Deployment

Deploy this server to Vercel, Cloudflare Workers, or your preferred Node.js hosting platform.

## Security

- Uses bcrypt for password hashing
- Secure session cookies
- Rate limiting on auth endpoints