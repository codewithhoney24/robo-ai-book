# Quickstart Guide: Better-Auth with User Background Collection

This guide will help you quickly verify that all user stories are working correctly.

## Prerequisites

- Node.js 18+
- PostgreSQL database
- Git

## Setup

1. Clone the repository:
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. Set up the backend:
   ```bash
   cd backend
   npm install
   cp .env.example .env
   # Update .env with your database URL and auth secret
   npm run dev
   ```

3. Set up the frontend:
   ```bash
   cd frontend  # Open a new terminal
   npm install
   npm start
   ```

## Validation Checklist

### User Story 1: Create Account with Background Information

1. Navigate to http://localhost:3000/signup
2. Fill in the signup form with:
   - Name: Any name
   - Email: A valid email address
   - Password: At least 8 characters
   - Software Background: Select any option
   - Hardware Background: Select any option
3. Click "Sign Up"
4. Verify: Account is created successfully with background information stored

### User Story 2: Sign In to Access Personalized Content

1. Navigate to http://localhost:3000/signin
2. Enter the credentials from User Story 1
3. Click "Sign In"
4. Verify: Successfully authenticated and redirected to profile

### User Story 3: Update Personal Background Information

1. Ensure you're logged in
2. Navigate to http://localhost:3000/profile
3. Click "Edit Profile"
4. Change your software or hardware background
5. Click "Save Changes"
6. Verify: Profile is updated with new background information

### User Story 4: View Personalized Content

1. Ensure you're logged in with specific background information
2. Navigate to content pages (implementation-specific)
3. Verify:
   - Content appropriate for your skill level is shown
   - Hardware-appropriate recommendations are provided
   - GPU-intensive content warnings appear if relevant to your hardware

## Testing Personalization

1. Create accounts with different background combinations
2. Sign in with each account
3. Verify content personalization works as expected:
   - Beginners see detailed explanations
   - Users without GPU see cloud alternatives
   - Users with workstation see premium features
   - etc.

## API Endpoints Verification

- `POST /api/auth/signup` - Verify new user creation with background fields
- `POST /api/auth/signin` - Verify authentication
- `GET /api/auth/session` - Verify session retrieval with background info
- `POST /api/auth/signout` - Verify logout
- `PATCH /api/auth/user` - Verify profile updates

## Security Checks

- Passwords are properly hashed
- Sessions expire after 7 days
- Rate limiting is enforced (5 signups, 10 logins per hour per IP)
- CORS is properly configured

## Troubleshooting

If any of the above steps fail:

1. Check backend logs for error messages
2. Verify your `.env` configuration
3. Ensure your database is accessible
4. Confirm CORS settings match your frontend URL

## Next Steps

After successful validation:

1. Deploy the backend to Vercel
2. Deploy the frontend to GitHub Pages
3. Update environment variables for production
4. Test the deployed application