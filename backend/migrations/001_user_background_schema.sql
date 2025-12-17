-- Migration for Better-Auth with software and hardware background fields
-- Better-Auth creates its own base schema, we just need to add custom columns

-- Add custom fields to the user table created by Better-Auth
-- The user table in Better-Auth uses snake_case for custom fields
ALTER TABLE "user"
ADD COLUMN IF NOT EXISTS "software_background" TEXT DEFAULT 'beginner';

ALTER TABLE "user"
ADD COLUMN IF NOT EXISTS "hardware_background" TEXT DEFAULT 'no_gpu';

-- Add name column if needed (Better-Auth might already have this)
ALTER TABLE "user"
ADD COLUMN IF NOT EXISTS "name" TEXT DEFAULT '';

-- Create indexes for better query performance
CREATE INDEX IF NOT EXISTS "user_email_idx" ON "user"("email");
CREATE INDEX IF NOT EXISTS "user_software_background_idx" ON "user"("software_background");
CREATE INDEX IF NOT EXISTS "user_hardware_background_idx" ON "user"("hardware_background");

-- Update any existing users with missing profile data
-- This ensures all existing users have default values
UPDATE "user"
SET "software_background" = 'beginner'
WHERE "software_background" IS NULL OR "software_background" = '';

UPDATE "user"
SET "hardware_background" = 'no_gpu'
WHERE "hardware_background" IS NULL OR "hardware_background" = '';

UPDATE "user"
SET "name" = email
WHERE "name" IS NULL OR "name" = '';