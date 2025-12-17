import { betterAuth } from 'better-auth';
import { Pool } from 'pg';

/* -------------------------------------------------
   ENV VALIDATION
------------------------------------------------- */

if (!process.env.DATABASE_URL) {
  console.error('❌ DATABASE_URL is missing');
  process.exit(1);
}

if (!process.env.BETTER_AUTH_SECRET) {
  console.error('❌ BETTER_AUTH_SECRET is missing');
  process.exit(1);
}

console.log('✅ Loaded DATABASE_URL:', process.env.DATABASE_URL.substring(0, 30) + '...');

/* -------------------------------------------------
   DATABASE CONNECTION
------------------------------------------------- */

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  },
});

// Test connection
(async () => {
  try {
    const client = await pool.connect();
    console.log('✅ PostgreSQL connected');
    client.release();
  } catch (err: any) {
    console.error('❌ Database connection failed:', err.message);
  }
})();

/* -------------------------------------------------
   BETTER AUTH CONFIG
------------------------------------------------- */

export const auth = betterAuth({
  database: pool, // ✅ Use Pool directly

  secret: process.env.BETTER_AUTH_SECRET,

  /* ---------------- AUTH METHODS ---------------- */

  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    requireEmailVerification: false,
  },

  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24,
  },

  /* ---------------- USER FIELDS ---------------- */

  user: {
    additionalFields: {
      softwareBackground: {
        type: 'string',
        required: false,
        input: true,
        defaultValue: 'beginner',
      },

      programmingLevel: {
        type: 'string',
        required: false,
        input: true,
        defaultValue: 'beginner',
      },

      knownLanguages: {
        type: 'string',
        required: false,
        input: true,
        defaultValue: '[]',
      },

      aiMlExperience: {
        type: 'boolean',
        required: false,
        input: true,
        defaultValue: false,
      },

      rosRoboticsExperience: {
        type: 'boolean',
        required: false,
        input: true,
        defaultValue: false,
      },

      hasJetson: {
        type: 'boolean',
        required: false,
        input: true,
        defaultValue: false,
      },

      hasRTXGPU: {
        type: 'boolean',
        required: false,
        input: true,
        defaultValue: false,
      },

      hasRobotHardware: {
        type: 'boolean',
        required: false,
        input: true,
        defaultValue: false,
      },

      simulationOnly: {
        type: 'boolean',
        required: false,
        input: true,
        defaultValue: true,
      },
    },
  },

  /* ---------------- ACCOUNT ---------------- */

  account: {
    accountLinking: {
      enabled: true,
    },
  },

  /* ---------------- SECURITY ---------------- */

  trustedOrigins: [
    'http://localhost:3000',
    'http://localhost:5173',
    'http://localhost:8000',
  ],
});

console.log('✅ Better-Auth initialized with PostgreSQL');