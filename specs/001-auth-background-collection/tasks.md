# Tasks: Better-Auth with User Background Collection

**Input**: Design documents from `/specs/001-auth-background-collection/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification did not explicitly request tests, so test tasks are not included in this implementation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths shown below follow the web app structure mentioned in the plan

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create project structure per implementation plan with backend and frontend directories
- [ ] T002 [P] Initialize backend Node.js project with package.json and dependencies (better-auth, hono, pg, dotenv, tsx)
- [ ] T003 [P] Initialize frontend Docusaurus project with proper configuration
- [ ] T004 Setup TypeScript configuration for both backend and frontend
- [ ] T005 Create environment configuration files (.env.example) for both backend and frontend

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 [P] Setup Neon Postgres database connection in backend/src/db.ts
- [ ] T007 [P] Configure Better-Auth with custom fields (softwareBackground, hardwareBackground) in backend/src/auth.ts
- [ ] T008 [P] Setup Hono server with CORS for frontend integration in backend/src/server.ts
- [ ] T009 Create User model/entity with background fields in backend/src/models/user.ts
- [ ] T010 Setup database migration for custom background fields in backend/migrations/001_user_background.sql
- [ ] T011 Configure rate limiting middleware in backend/src/middleware/rate-limit.ts
- [ ] T012 Initialize authentication context in frontend/src/contexts/AuthContext.tsx

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Create Account with Background Information (Priority: P1) 🎯 MVP

**Goal**: Enable new users to create an account with their software and hardware background information, storing this data for personalized content.

**Independent Test**: A new user can visit the signup page, provide name, email, password, and select their software and hardware background, then successfully create an account with all provided information stored in the database.

### Implementation for User Story 1

- [ ] T013 [P] [US1] Create signup page UI in frontend/src/pages/signup.tsx with form fields
- [ ] T014 [P] [US1] Add software background dropdown with required options in signup page
- [ ] T015 [P] [US1] Add hardware background dropdown with required options in signup page
- [ ] T016 [US1] Integrate signup form with Better-Auth API in frontend/src/services/auth.ts
- [ ] T017 [US1] Add form validation for all fields (email format, password strength, required backgrounds)
- [ ] T018 [US1] Implement signup API endpoint in backend/src/api/auth.ts
- [ ] T019 [US1] Validate custom background fields against allowed enum values in backend/src/validation/user.ts
- [ ] T020 [US1] Test successful signup with background collection

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Sign In to Access Personalized Content (Priority: P1)

**Goal**: Enable existing users to sign in to access their personalized content based on their software and hardware background information.

**Independent Test**: An existing user can visit the sign in page, enter their email and password, and successfully authenticate to access personalized content based on their background information.

### Implementation for User Story 2

- [ ] T021 [US2] Create signin page UI in frontend/src/pages/signin.tsx with email/password form
- [ ] T022 [US2] Add "Remember me" checkbox functionality to signin page
- [ ] T023 [US2] Implement "Forgot password" link functionality
- [ ] T024 [US2] Integrate signin form with Better-Auth API in frontend/src/services/auth.ts
- [ ] T025 [US2] Implement sign in API endpoint in backend/src/api/auth.ts
- [ ] T026 [US2] Ensure user background information is accessible in session data
- [ ] T027 [US2] Test successful sign in with session persistence
- [ ] T028 [US2] Test that background data is available after sign in

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Update Personal Background Information (Priority: P2)

**Goal**: Allow registered users to update their software and hardware background information so their content personalization remains accurate.

**Independent Test**: A logged-in user can navigate to their profile, update their background information, save the changes, and verify that their background information has been updated in the system.

### Implementation for User Story 3

- [ ] T029 [P] [US3] Create profile page UI in frontend/src/pages/profile.tsx showing user data
- [ ] T030 [US3] Add form to update background information on profile page
- [ ] T031 [US3] Integrate profile update with Better-Auth API in frontend/src/services/auth.ts
- [ ] T032 [US3] Implement profile update API endpoint in backend/src/api/auth.ts
- [ ] T033 [US3] Ensure background updates are validated against allowed enum values
- [ ] T034 [US3] Add functionality to change password on profile page
- [ ] T035 [US3] Test successful background information updates

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - View Personalized Content (Priority: P2)

**Goal**: Display content that's personalized based on the user's software and hardware background.

**Independent Test**: A logged-in user sees content that's appropriate for their software and hardware background levels (e.g., beginners see basic Python explanations, users with no GPU see cloud alternatives instead of GPU-intensive examples).

### Implementation for User Story 4

- [ ] T036 [P] [US4] Implement ProtectedRoute component in frontend/src/components/ProtectedRoute.tsx
- [ ] T037 [US4] Create content personalization logic based on user backgrounds
- [ ] T038 [US4] Implement display logic for different content based on software background
- [ ] T039 [US4] Implement display logic for different content based on hardware availability
- [ ] T040 [US4] Add warning badges for users with no GPU accessing GPU-intensive content
- [ ] T041 [US4] Test that content displays appropriately based on user's background
- [ ] T042 [US4] Implement logout functionality in frontend/src/components/LogoutButton.tsx

**Checkpoint**: All user stories should now be fully functional together

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T043 [P] Add documentation updates in docs/auth-implementation.md
- [ ] T044 [P] Create README.md with setup instructions for auth system
- [ ] T045 Configure security headers for auth endpoints in backend/src/middleware/security.ts
- [ ] T046 Add loading spinners during auth checks in frontend components
- [ ] T047 Add error handling and user-friendly messages for auth operations
- [ ] T048 Add unit tests for authentication services
- [ ] T049 Run final validation using quickstart.md to ensure all user stories work

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - Depends on authentication working (US1/US2)

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all UI components for User Story 1 together:
Task: "Create signup page UI in frontend/src/pages/signup.tsx with form fields"
Task: "Add software background dropdown with required options in signup page"
Task: "Add hardware background dropdown with required options in signup page"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: User Story 4
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence