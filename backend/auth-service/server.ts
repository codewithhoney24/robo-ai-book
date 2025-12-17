import { auth } from './auth.js';
import { Hono } from 'hono';
import { serve } from '@hono/node-server';
import { cors } from 'hono/cors';

// Better-Auth provides its own server implementation
// In a standard setup, you would use:
const startServer = async () => {
  // Create a Hono app
  const app = new Hono();

  // Add CORS middleware
  app.use('*', cors({
    origin: ['http://localhost:3000', 'http://localhost:5173', 'http://localhost:8000'],
    credentials: true,
  }));

  // Health check endpoint
  app.get('/', (c) => c.json({
    message: 'Better-Auth API Server (Debug)',
    status: 'running',
    endpoints: {
      signup: 'POST /api/auth/sign-up/email',
      signin: 'POST /api/auth/sign-in/email',
      session: 'GET /api/auth/get-session',
      signout: 'POST /api/auth/sign-out',
      github: 'GET /api/auth/sign-in/social/github'
    }
  }));

  // Mount Better Auth routes
  app.all('/api/auth/*', async (c) => {
    // Better-Auth expects a native Request object
    const req = c.req.raw;
    const response = await auth.handler(req);
    return response;
  });

  // Health check endpoint
  app.get('/health', (c) => c.json({
    status: 'healthy',
    timestamp: new Date().toISOString(),
    database: process.env.DATABASE_URL ? 'connected' : 'using fallback'
  }));

  const port = parseInt(process.env.PORT || '3100', 10);
  console.log(`Better-Auth server running on port ${port}`);
  console.log(`Health check: http://localhost:${port}/health`);

  // Start the server
  serve({
    fetch: app.fetch,
    port,
  });
};

// Start the server
startServer().catch(console.error);

export { auth };