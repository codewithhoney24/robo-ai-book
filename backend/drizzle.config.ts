import { defineConfig } from 'drizzle-kit';

export default defineConfig({
  schema: './backend/src/database/schema.ts',
  out: './backend/migrations',
  dialect: 'postgresql',
  dbCredentials: {
    url: process.env.DATABASE_URL!,
  },
});