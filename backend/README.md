# Better-Auth with User Background Collection

This project implements an authentication system using Better-Auth that collects user's software and hardware background during signup for content personalization in a Physical AI textbook.

## Features

- User registration with software and hardware background information
- Secure login/logout functionality
- Profile management with background updates
- Content personalization based on user's background
- Rate limiting for security
- Session management with 7-day expiration

## Tech Stack

- **Backend**: Node.js, Better-Auth, Hono, PostgreSQL (Neon)
- **Frontend**: React (Docusaurus)
- **Deployment**: Vercel (auth), GitHub Pages (frontend)

## Setup Instructions

### Prerequisites
- Node.js 18+ 
- PostgreSQL database (Neon recommended)
- Git

### Backend Setup

1. Clone the repository:
   ```bash
   git clone [repository-url]
   cd [repository-name]/backend
   ```

2. Install dependencies:
   ```bash
   cd backend
   npm install
   ```

3. Create environment file:
   ```bash
   cp .env.example .env
   ```

4. Configure environment variables in `.env`:
   - `DATABASE_URL`: Your PostgreSQL connection string
   - `BETTER_AUTH_SECRET`: A secure secret key (minimum 32 characters)
   - `FRONTEND_URL`: URL of your frontend (e.g., http://localhost:3000)

5. Run the application:
   ```bash
   npm run dev
   ```

The backend will start on port 3001 by default.

### Frontend Setup

1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Create environment file:
   ```bash
   cp .env.example .env
   ```

4. Configure environment variables in `.env` if needed

5. Start the development server:
   ```bash
   npm start
   ```

The frontend will start on port 3000 by default.

## Database Setup

This implementation uses PostgreSQL with Neon. You'll need to:

1. Create a Neon account and project
2. Get your connection string from the Neon dashboard
3. Set it as the `DATABASE_URL` in your backend `.env` file
4. Run the migration to add background fields to the user table

## API Endpoints

- `POST /api/auth/signup` - Create a new user with background info
- `POST /api/auth/signin` - Authenticate a user
- `GET /api/auth/session` - Get current user session
- `POST /api/auth/signout` - Logout user
- `PATCH /api/auth/user` - Update user profile

## Environment Variables

### Backend (.env)
- `DATABASE_URL` - PostgreSQL connection string
- `BETTER_AUTH_SECRET` - Secret key for auth (min 32 chars)
- `BETTER_AUTH_URL` - Base URL for the auth service
- `NODE_ENV` - Environment (development/production)
- `PORT` - Server port (default 3001)
- `FRONTEND_URL` - Allowed origin for CORS

### Frontend (.env)
- `REACT_APP_AUTH_URL` - Backend auth service URL
- `NODE_ENV` - Environment (development/production)

## Running Tests

Currently, the implementation focuses on core functionality. Unit tests would be added as part of future enhancements.

## Deployment

### Backend to Vercel

1. Install Vercel CLI: `npm install -g vercel`
2. Login: `vercel login`
3. Deploy: `vercel --prod`
4. Set environment variables in Vercel dashboard

### Frontend to GitHub Pages

1. Configure your GitHub repository for GitHub Pages
2. Build: `npm run build`
3. Deploy the build folder to GitHub Pages

## Personalization Strategy

The system personalizes content based on:

### Software Background
- **Beginner**: More detailed explanations and Python basics
- **Python Intermediate**: Skip basics, focus on ROS 2 APIs
- **ROS 2 Developer**: Advanced topics only
- **AI Robotics Expert**: Research papers, custom implementations

### Hardware Availability
- **No GPU**: Hide Isaac Sim, show cloud alternatives with warning badges
- **RTX Laptop**: Enable content with VRAM warnings
- **RTX Workstation**: Full access, premium features
- **Jetson**: Edge AI focus, deployment guides
- **Cloud**: AWS/Azure setup, cost optimization

## Security Features

- bcrypt password hashing with 10 rounds
- HTTP-only cookies for session management
- Rate limiting (5 signups, 10 logins per hour per IP)
- CORS configured for trusted origins only
- Input validation with Zod schemas

## Troubleshooting

- **CORS Errors**: Ensure FRONTEND_URL matches your deployed frontend URL
- **Database Connection**: Verify your DATABASE_URL is correctly configured
- **Session Issues**: Confirm BETTER_AUTH_SECRET is consistent across environments
- **Rate Limiting**: Check if your IP has been temporarily blocked

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

[Specify your license here]