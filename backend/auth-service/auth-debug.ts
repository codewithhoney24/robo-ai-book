import { betterAuth } from 'better-auth';
import { Pool } from 'pg';

// Define the custom user fields
type SoftwareBackground =
  | 'beginner'
  | 'python_intermediate'
  | 'ros2_developer'
  | 'ai_robotics_expert';

type HardwareBackground =
  | 'no_gpu'
  | 'rtx_laptop'
  | 'rtx_workstation'
  | 'jetson_kit'
  | 'cloud';

// Log environment variables to check if they are loaded
console.log('DATABASE_URL from process.env:', process.env.DATABASE_URL ? 'SET' : 'NOT SET');
console.log('BETTER_AUTH_SECRET from process.env:', process.env.BETTER_AUTH_SECRET ? 'SET' : 'NOT SET');

// Check if we have the required environment variables
const DATABASE_URL = process.env.DATABASE_URL;
const SECRET = process.env.BETTER_AUTH_SECRET;
const GITHUB_CLIENT_ID = process.env.GITHUB_CLIENT_ID;
const GITHUB_CLIENT_SECRET = process.env.GITHUB_CLIENT_SECRET;

console.log('DATABASE_URL variable:', DATABASE_URL ? 'SET' : 'NOT SET');
console.log('SECRET variable:', SECRET ? 'SET' : 'NOT SET');

if (!DATABASE_URL) {
  console.warn('DATABASE_URL is not set. Using memory adapter for development.');
}

if (!SECRET) {
  console.warn('BETTER_AUTH_SECRET is not set. Using fallback secret for development.');
}

if (!GITHUB_CLIENT_ID || !GITHUB_CLIENT_SECRET) {
  console.warn('GITHUB_CLIENT_ID or GITHUB_CLIENT_SECRET not set. GitHub OAuth will be disabled.');
}

// Extend the default user model with custom fields
let authConfig = {};

if (DATABASE_URL) {
  console.log('Using PostgreSQL database configuration');
  authConfig = {
    database: {
      url: DATABASE_URL,
      provider: 'postgresql',  // Updated to use 'postgresql' provider as per Better Auth documentation
      options: {
        // These options help with Neon connection handling
        ssl: {
          rejectUnauthorized: false  // Required for Neon's SSL certificate
        }
      }
    },
    secret: SECRET || 'fallback_dev_secret_that_should_be_changed_in_production',
    emailAndPassword: {
      enabled: true,
    },
    socialProviders: {
      github: GITHUB_CLIENT_ID && GITHUB_CLIENT_SECRET ? {
        clientId: GITHUB_CLIENT_ID,
        clientSecret: GITHUB_CLIENT_SECRET,
      } : undefined,
    },
    // Session configuration
    session: {
      expiresIn: 7 * 24 * 60 * 60, // 7 days in seconds
      updateAge: 24 * 60 * 60, // 24 hours in seconds
    },
    user: {
      fields: {
        name: "name",
        softwareBackground: "software_background",
        hardwareBackground: "hardware_background",
      }
    },
    plugins: [],
  };
} else {
  console.log('Using SQLite database configuration');
  authConfig = {
    database: {
      provider: 'sqlite',
      url: 'file:./dev.db',
    },
    secret: SECRET || 'fallback_dev_secret_that_should_be_changed_in_production',
    emailAndPassword: {
      enabled: true,
    },
    socialProviders: {
      github: GITHUB_CLIENT_ID && GITHUB_CLIENT_SECRET ? {
        clientId: GITHUB_CLIENT_ID,
        clientSecret: GITHUB_CLIENT_SECRET,
      } : undefined,
    },
    // Session configuration
    session: {
      expiresIn: 7 * 24 * 60 * 60, // 7 days in seconds
      updateAge: 24 * 60 * 60, // 24 hours in seconds
    },
    user: {
      fields: {
        name: "name",
        softwareBackground: "software_background",
        hardwareBackground: "hardware_background",
      }
    },
    plugins: [],
  };
}

export const auth = betterAuth(authConfig);