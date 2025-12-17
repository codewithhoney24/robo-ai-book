// ❌ DO NOT import dotenv
// ❌ DO NOT load .env manually

// ✅ Env vars are already loaded by Node (--env-file)

// ---- Validate ENV early ----
if (!process.env.DATABASE_URL) {
  console.error('❌ DATABASE_URL is missing');
  process.exit(1);
}

if (!process.env.BETTER_AUTH_SECRET) {
  console.error('❌ BETTER_AUTH_SECRET is missing');
  process.exit(1);
}

console.log('✅ Environment variables loaded');
console.log('🔍 DATABASE_URL:', '✅ Found');
console.log('🔍 BETTER_AUTH_SECRET:', '✅ Found');

// ---- Imports AFTER env is confirmed ----
import { serve } from '@hono/node-server';
import { Hono } from 'hono';
import { cors } from 'hono/cors';

const app = new Hono();

app.use('*', cors({
  origin: ['http://localhost:3000', 'http://localhost:5173'],
  credentials: true,
}));

// Dynamic import auth after env is confirmed loaded
const startServer = async () => {
  try {
    const { auth } = await import('./auth.js');

    app.get('/', (c) => c.json({
      message: 'Better-Auth API Server',
      status: 'running'
    }));

    app.get('/health', (c) => c.json({
      status: 'healthy',
      timestamp: new Date().toISOString()
    }));

    // Get session endpoint
    app.get('/api/auth/get-session', async (c) => {
      try {
        // Extract headers from the request
        const headers = new Headers();
        const reqHeaders = c.req.header();
        Object.keys(reqHeaders).forEach(key => {
          headers.append(key, reqHeaders[key]!);
        });

        // Forward to Better-Auth's session endpoint
        const request = new Request('http://localhost:8000/api/auth/session', {
          method: 'GET',
          headers: headers,
        });

        const response = await auth.handler(request);
        return response;
      } catch (error) {
        console.error('Error in session endpoint:', error);
        return c.json({ error: 'Failed to get session' }, 500);
      }
    });

    // ✅ Add custom aliases for sign-up and sign-in
    app.post('/api/auth/sign-up', async (c) => {
      try {
        // Check if content type is JSON
        const contentType = c.req.header('Content-Type');
        if (!contentType || !contentType.includes('application/json')) {
          return c.json({ error: 'Content-Type must be application/json' }, 400);
        }

        const body = await c.req.json();

        // Forward to Better-Auth's email sign-up endpoint
        const request = new Request('http://localhost:8000/api/auth/sign-up/email', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(body),
        });

        return await auth.handler(request);
      } catch (error) {
        console.error('Error in sign-up endpoint:', error);
        if (error instanceof SyntaxError && error.message.includes('JSON')) {
          return c.json({ error: 'Invalid JSON in request body' }, 400);
        }
        return c.json({ error: 'Bad Request' }, 400);
      }
    });

    app.post('/api/auth/sign-in', async (c) => {
      try {
        // Check if content type is JSON
        const contentType = c.req.header('Content-Type');
        if (!contentType || !contentType.includes('application/json')) {
          return c.json({ error: 'Content-Type must be application/json' }, 400);
        }

        const body = await c.req.json();

        // Forward to Better-Auth's email sign-in endpoint
        const request = new Request('http://localhost:8000/api/auth/sign-in/email', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(body),
        });

        return await auth.handler(request);
      } catch (error) {
        console.error('Error in sign-in endpoint:', error);
        if (error instanceof SyntaxError && error.message.includes('JSON')) {
          return c.json({ error: 'Invalid JSON in request body' }, 400);
        }
        return c.json({ error: 'Bad Request' }, 400);
      }
    });

    // Sign-out endpoint
    app.post('/api/auth/sign-out', async (c) => {
      try {
        // Extract headers from the request
        const headers = new Headers();
        const reqHeaders = c.req.header();
        Object.keys(reqHeaders).forEach(key => {
          headers.append(key, reqHeaders[key]!);
        });

        const request = new Request('http://localhost:8000/api/auth/sign-out', {
          method: 'POST',
          headers: headers,
        });

        return await auth.handler(request);
      } catch (error) {
        console.error('Error in sign-out endpoint:', error);
        return c.json({ error: 'Failed to sign out' }, 500);
      }
    });

    // ✅ Handle all other Better-Auth routes
    app.all('/api/auth/*', async (c) => {
      return await auth.handler(c.req.raw);
    });

    const port = Number(process.env.PORT) || 8000;

    serve({ fetch: app.fetch, port }, (info) => {
      console.log('\n✅ ═══════════════════════════════════');
      console.log('   Better-Auth Server Running');
      console.log('═══════════════════════════════════');
      console.log(`🌐 http://localhost:${info.port}`);
      console.log(`🏥 http://localhost:${info.port}/health`);
      console.log('📝 Custom endpoints:');
      console.log(`   POST   /api/auth/sign-up`);
      console.log(`   POST   /api/auth/sign-in`);
      console.log(`   GET    /api/auth/get-session`);
      console.log(`   POST   /api/auth/sign-out`);
      console.log(`   ALL    /api/auth/* (Better-Auth routes)`);
      console.log('═══════════════════════════════════\n');
    });

  } catch (error) {
    console.error('❌ Failed to start server:', error);
    process.exit(1);
  }
};

startServer();