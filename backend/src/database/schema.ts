import { pgTable, serial, text, timestamp, boolean, varchar, json } from 'drizzle-orm/pg-core';

// Better-Auth creates its own user table schema, but we define the custom fields here
// The actual schema will be managed by Better-Auth, but we define additional fields
// that will be added to the Better-Auth user table
export const userProfiles = pgTable('user_profiles', {
  id: text('id').primaryKey(),
  userId: text('user_id').notNull().references(() => users.id, { onDelete: 'cascade' }), // Reference to Better-Auth user
  createdAt: timestamp('created_at').defaultNow().notNull(),
  updatedAt: timestamp('updated_at').defaultNow().notNull(),

  // Software background fields
  programmingLevel: varchar('programming_level', { length: 50 }).notNull().default('beginner'),
  knownLanguages: text('known_languages'), // JSON string of languages
  aiMlExperience: boolean('ai_ml_experience').notNull().default(false),
  rosRoboticsExperience: boolean('ros_robotics_experience').notNull().default(false),

  // Hardware background fields
  hasJetson: boolean('has_jetson').notNull().default(false),
  hasRTXGPU: boolean('has_rtx_gpu').notNull().default(false),
  hasRobotHardware: boolean('has_robot_hardware').notNull().default(false),
  simulationOnly: boolean('simulation_only').notNull().default(true),

  // Personalization preferences
  contentPreference: varchar('content_preference', { length: 50 }).default('balanced'),
  notificationPreference: json('notification_preference'),
});

// Define the Better-Auth user table with additional fields as defined in the auth.ts
export const users = pgTable('user', {
  id: text('id').primaryKey(),
  email: text('email').notNull().unique(),
  emailVerified: boolean('emailVerified').default(false),
  name: text('name').notNull(),
  password: text('password'), // for password-based auth
  createdAt: timestamp('createdAt').defaultNow().notNull(),
  updatedAt: timestamp('updatedAt').defaultNow().notNull(),
  // These fields are added via Better-Auth's additionalFields config
  softwareBackground: varchar('software_background', { length: 50 }).notNull().default('beginner'),
  programmingLevel: varchar('programming_level', { length: 50 }).notNull().default('beginner'),
  knownLanguages: text('known_languages'), // JSON string of languages
  aiMlExperience: boolean('ai_ml_experience').notNull().default(false),
  rosRoboticsExperience: boolean('ros_robotics_experience').notNull().default(false),
  hasJetson: boolean('has_jetson').notNull().default(false),
  hasRTXGPU: boolean('has_rtx_gpu').notNull().default(false),
  hasRobotHardware: boolean('has_robot_hardware').notNull().default(false),
  simulationOnly: boolean('simulation_only').notNull().default(true),
});