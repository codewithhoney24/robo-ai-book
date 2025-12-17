CREATE TABLE "user" (
	"id" text PRIMARY KEY NOT NULL,
	"email" text NOT NULL,
	"emailVerified" boolean DEFAULT false,
	"name" text NOT NULL,
	"password" text,
	"createdAt" timestamp DEFAULT now() NOT NULL,
	"updatedAt" timestamp DEFAULT now() NOT NULL,
	"software_background" text DEFAULT 'beginner' NOT NULL,
	"hardware_background" text DEFAULT 'no_gpu' NOT NULL,
	CONSTRAINT "user_email_unique" UNIQUE("email")
);
