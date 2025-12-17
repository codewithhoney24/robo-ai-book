# Feature Specification: Better-Auth with User Background Collection

**Feature Branch**: `001-auth-background-collection`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: "Better-Auth with User Background Collection ## 🎯 Requirement Implement Better-Auth authentication system that collects user's software and hardware background during signup for content personalization in a Physical AI textbook. --- ## 🔧 Tech Stack - **Auth:** Better-Auth (Node.js/TypeScript) - **Backend:** Hono framework - **Database:** Neon Postgres - **Frontend:** Docusaurus (React/TypeScript) - **Deploy:** Vercel (auth), GitHub Pages (frontend) --- ## 📊 User Data Collection ### Standard Fields - Name, Email, Password (8+ chars, hashed) ### Custom Field 1: Software Background (Required) ```typescript type SoftwareBackground = | \"beginner\" // No Python/C++ | \"python_intermediate\" // Python basics | \"ros2_developer\" // ROS 1/2 experience | \"ai_robotics_expert\" // Professional level ``` ### Custom Field 2: Hardware Availability (Required) ```typescript type HardwareBackground = | \"no_gpu\" // Standard laptop | \"rtx_laptop\" // RTX 2060-4060 | \"rtx_workstation\" // RTX 3090/4090 | \"jetson_kit\" // Jetson Orin | \"cloud\" // AWS/Azure ``` --- ## 🗄️ Database Schema ```sql -- Better-Auth auto-creates base user table -- We add custom fields: ALTER TABLE user ADD COLUMN softwareBackground TEXT NOT NULL, ADD COLUMN hardwareBackground TEXT NOT NULL; -- Session table (auto-created by Better-Auth) -- userPreferences table (optional) ``` --- ## 🌐 API Endpoints **Base URL:** `https://auth-server.vercel.app` ```typescript POST /api/auth/signup // Create account with backgrounds POST /api/auth/signin // Login GET /api/auth/session // Get current user POST /api/auth/signout // Logout PATCH /api/auth/user // Update profile ``` **Signup Request:** ```json { \"email\": \"user@example.com\", \"password\": \"Pass123\", \"name\": \"Ahmed\", \"softwareBackground\": \"python_intermediate\", \"hardwareBackground\": \"rtx_laptop\" } ``` --- ## 🎨 Frontend Pages ### 1. Signup (`/signup`) - Name, Email, Password inputs - Software dropdown (4 options) - Hardware dropdown (5 options) - Validation + tooltips ### 2. Signin (`/signin`) - Email/Password form - Remember me checkbox - Forgot password link ### 3. Profile (`/profile`) - Display user background - Show personalization settings - Edit profile/Change password ### 4. Protected Routes - Auth check before render - Redirect to signin if unauthenticated - Loading spinner during check --- ## 🎯 Personalization Rules ### Software Impact: - **Beginner:** Show Python basics, detailed explanations - **Intermediate:** Skip basics, focus on ROS 2 API - **ROS 2 Dev:** Advanced topics only - **Expert:** Research papers, custom implementations ### Hardware Impact: - **No GPU:** Hide Isaac Sim, show cloud alternatives, ⚠️ badges - **RTX Laptop:** Enable content, VRAM warnings - **Workstation:** Full access, premium features - **Jetson:** Edge AI focus, deployment guides - **Cloud:** AWS/Azure setup, cost optimization --- ## 🔐 Security - bcrypt password hashing (10 rounds) - HTTP-only cookies, 7-day sessions - CORS: localhost:3000 + production URL - Rate limiting: 5 signups, 10 logins per hour --- ## 📦 Deliverables 1. `auth.ts` - Better-Auth config 2. `signup.tsx` - Signup page 3. `signin.tsx` - Signin page 4. `profile.tsx` - User profile 5. `ProtectedRoute.tsx` - Auth wrapper 6. `001_migration.sql` - DB schema 7. `.env.example` - Environment variables 8. `README.md` - Setup guide --- ## ✅ Success Criteria - [ ] User signup with both backgrounds - [ ] Data saved to Neon database - [ ] Session persists across reloads - [ ] Protected routes work - [ ] Profile displays correct data - [ ] No CORS errors in production --- **Points:** 50 (Bonus) + enables 100 more (Personalization + Translation) **Next Step:** `sp.plan` for implementation architecture"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Create Account with Background Information (Priority: P1)

As a new user of the Physical AI textbook platform, I want to create an account with my software and hardware background information so that the content can be personalized to my skill level and available resources.

**Why this priority**: This is the foundational user journey that enables all other functionality. Without this, users cannot access personalized content.

**Independent Test**: Can be fully tested by creating a new account with background information and verifying that the data is saved correctly, delivering the value of personalized content recommendations.

**Acceptance Scenarios**:

1. **Given** I am a new user on the signup page, **When** I provide my name, email, password, and select my software and hardware background, **Then** I should be able to create an account successfully with all provided information stored.

2. **Given** I have provided valid account information, **When** I submit the signup form, **Then** I should be logged in automatically and redirected to the personalized content dashboard.

---

### User Story 2 - Sign In to Access Personalized Content (Priority: P1)

As an existing user of the Physical AI textbook platform, I want to sign in to access my personalized content based on my software and hardware background information.

**Why this priority**: This is the primary way users access the platform after their initial signup. Essential for ongoing user engagement.

**Independent Test**: Can be fully tested by signing in with existing credentials and verifying access to personalized content, delivering the value of tailored learning experience.

**Acceptance Scenarios**:

1. **Given** I am an existing user with an account, **When** I enter my email and password on the sign in page, **Then** I should be authenticated and redirected to my personalized content.

2. **Given** I'm on the sign in page, **When** I check the "Remember me" option and sign in, **Then** my session should persist across browser restarts for a specified period.

---

### User Story 3 - Update Personal Background Information (Priority: P2)

As a registered user, I want to update my software and hardware background information so that my content personalization remains accurate over time.

**Why this priority**: This allows users to update their information as their skills or hardware changes, ensuring continued relevance of personalized content.

**Independent Test**: Can be fully tested by updating background information and verifying that changes are saved and reflected in content personalization.

**Acceptance Scenarios**:

1. **Given** I'm logged into my account, **When** I navigate to my profile and update my background information, **Then** my changes should be saved and reflected in the content personalization.

2. **Given** I'm on the profile page, **When** I make changes to my background and click save, **Then** I should receive confirmation that my information has been updated.

---

### User Story 4 - View Personalized Content (Priority: P2)

As a logged-in user, I want to view content that's personalized based on my software and hardware background so that I can learn more effectively with relevant examples.

**Why this priority**: This is the core value proposition that differentiates our platform from generic AI textbooks.

**Independent Test**: Can be fully tested by logging in with different background profiles and verifying that the content differs appropriately.

**Acceptance Scenarios**:

1. **Given** I'm logged in with beginner software background, **When** I browse the content, **Then** I should see more basic Python explanations and detailed introductions.

2. **Given** I'm logged in with no GPU hardware, **When** I encounter Isaac Sim content, **Then** I should see cloud alternatives with warning badges instead of GPU-intensive examples.

### Edge Cases

- What happens when a user tries to sign up with an already existing email?
- How does the system handle users who don't select any background information?
- What happens when a user changes their background information - does the content personalization update immediately?
- How does the system handle users with unstable internet connections during sign up/sign in?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow users to create accounts with name, email, password, software background, and hardware background
- **FR-002**: System MUST validate all user input during signup, including email format and password strength (at least 8 characters)
- **FR-003**: Users MUST be able to sign in with their email and password credentials
- **FR-004**: System MUST persist user session information for the duration of their login
- **FR-005**: Users MUST be able to sign out to end their current session
- **FR-006**: System MUST store user's software background information as one of: beginner, python_intermediate, ros2_developer, or ai_robotics_expert
- **FR-007**: System MUST store user's hardware background information as one of: no_gpu, rtx_laptop, rtx_workstation, jetson_kit, or cloud
- **FR-008**: Users MUST be able to update their profile information including background details
- **FR-009**: System MUST provide content personalization based on user's background information
- **FR-010**: System MUST implement rate limiting to prevent abuse (max 5 signups and 10 logins per hour per IP)
- **FR-011**: System MUST enforce security best practices including bcrypt password hashing with 10 rounds
- **FR-012**: System MUST use HTTP-only cookies for session management

### Key Entities

- **User**: Represents a registered user of the platform with attributes including name, email, password (hashed), software background, hardware background
- **Session**: Represents an active user session with attributes including session ID, associated user ID, and expiration time
- **Personalization Profile**: Represents the user's preferences and background information used to customize content delivery

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete account creation with background information in under 3 minutes
- **SC-002**: System supports at least 1,000 concurrent users without authentication performance degradation
- **SC-003**: 95% of users successfully complete the signup process with background information on first attempt
- **SC-004**: User session persistence works across browser reloads and remains active for 7 days
- **SC-005**: Content personalization is visible and appropriate based on user's background information
- **SC-006**: Security measures prevent unauthorized access and protect user credentials
- **SC-007**: System handles authentication rate limiting correctly (prevents more than 5 signups and 10 logins per hour per IP)