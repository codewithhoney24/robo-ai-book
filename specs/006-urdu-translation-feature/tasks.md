# Implementation Tasks: Urdu Translation Feature

**Feature**: Urdu Translation Feature with Docusaurus & FastAPI  
**Branch**: `006-urdu-translation-feature`  
**Generated**: 2025-12-15  
**Source**: specs/006-urdu-translation-feature/

## Implementation Strategy

The implementation follows a user story-driven approach with the following priority order:
- P1: User Story 1 - Urdu Translation Button (Core frontend functionality)
- P1: User Story 2 - Preserve Technical Terms (Core translation functionality)
- P2: User Story 3 - Backend Translation Endpoint (Backend infrastructure)

MVP scope includes User Story 1 with minimal backend functionality to support translation.

## Phase 1: Setup Tasks

### Goal
Initialize project structure and configure required dependencies for the Urdu translation feature.

### Tasks
- [ ] T001 Initialize project structure with proper backend (FastAPI) and frontend (Docusaurus) directories
- [ ] T002 [P] Install required dependencies including FastAPI, OpenAI client, and Docusaurus
- [ ] T003 Set up environment configuration with OpenAI API key placeholder
- [ ] T004 [P] Configure project for both backend and frontend development

## Phase 2: Foundational Tasks

### Goal
Implement foundational backend services and models required for all user stories.

### Tasks
- [ ] T005 [P] Create TranslationRequest model in backend/app/models/translation.py
- [ ] T006 [P] Create TranslationResponse model in backend/app/models/translation.py
- [ ] T007 [P] Create ChapterContent model in backend/app/models/content.py
- [ ] T008 [P] Implement translation service with OpenAI integration in backend/app/services/translation_service.py
- [ ] T009 Set up API endpoint structure in backend/app/api/translate.py
- [ ] T010 Implement system prompt for technical translation preservation in backend/app/prompts.py

## Phase 3: [US1] Urdu Translation Button

### Goal
Implement the frontend functionality allowing users to click a button to translate the current chapter into Urdu.

### Independent Test Criteria
Can be fully tested by clicking the translation button and verifying that the content is correctly translated to Urdu while preserving technical terms in English and applying proper RTL styling.

### Tasks
- [ ] T011 [P] [US1] Swizzle the Docusaurus DocItem component using the command npm run swizzle @docusaurus/theme-classic DocItem -- --wrap
- [ ] T012 [US1] Create the swizzled component at website/src/theme/DocItem/index.js with the necessary functionality
- [ ] T013 [US1] Implement state management for translation status and Urdu text in DocItem component
- [ ] T014 [US1] Add the "Translate to Urdu" button UI to the DocItem component
- [ ] T015 [P] [US1] Create the handleTranslate function to extract text content and make the API call
- [ ] T016 [US1] Implement the handleReset function to revert to original content
- [ ] T017 [P] [US1] Add conditional rendering logic to display either Urdu content with RTL styling or original English content
- [ ] T018 [US1] Implement loading state "Translating..." display during translation
- [ ] T019 [P] [US1] Add error handling for failed translation requests in the DocItem component
- [ ] T020 [US1] Style the translation controls for proper placement and appearance

## Phase 4: [US2] Preserve Technical Terms

### Goal
Ensure that technical terms (ROS 2, Gazebo, Isaac Sim, Python, Nodes, Topics) remain in English during translation to maintain their meaning and prevent confusion.

### Independent Test Criteria
Can be tested by translating content containing technical terms and verifying that they remain unchanged in English while the rest of the text is translated to Urdu.

### Tasks
- [ ] T021 [US2] Implement the technical term preservation logic in the system prompt for translation requests
- [ ] T022 [US2] Update the OpenAI integration to use the proper system prompt that preserves technical terms
- [ ] T023 [US2] Create test content containing technical terms for validation
- [ ] T024 [US2] Implement validation logic to verify that technical terms remain in English after translation
- [ ] T025 [P] [US2] Add logic in translation service to verify technical terms preservation before returning response

## Phase 5: [US3] Backend Translation Endpoint

### Goal
Create the backend endpoint that receives text content, sends it to an LLM with appropriate prompts for technical Urdu translation, and returns the translated text.

### Independent Test Criteria
Can be tested by making direct API calls to the translation endpoint with sample text and verifying that the response contains properly translated Urdu text.

### Tasks
- [ ] T026 [P] [US3] Implement the POST /api/translate endpoint in backend/app/api/translate.py
- [ ] T027 [US3] Add request validation for the TranslationRequest model in the endpoint
- [ ] T028 [P] [US3] Connect the endpoint to the translation service implementation
- [ ] T029 [US3] Implement proper error handling for the translation endpoint
- [ ] T030 [P] [US3] Add logging for translation requests and responses in backend/app/api/translate.py
- [ ] T031 [US3] Set up rate limiting for the translation endpoint to prevent abuse
- [ ] T032 [US3] Add response validation for TranslationResponse model in the endpoint

## Phase 6: Polish & Cross-Cutting Concerns

### Goal
Implement cross-cutting concerns and polish the feature for production.

### Tasks
- [ ] T033 [P] Add RTL styling for Urdu content in website/src/css/custom.css
- [ ] T034 [P] Implement proper RTL text direction (dir="rtl") in the frontend component
- [ ] T035 [P] Add proper validation for text length limits (max 2000 words) in backend
- [ ] T036 Add unit tests for backend translation service
- [ ] T037 Add integration tests for the /api/translate endpoint
- [ ] T038 Add frontend tests for the DocItem component functionality
- [ ] T039 [P] Update documentation with usage instructions for the translation feature
- [ ] T040 Perform end-to-end testing of the entire translation flow
- [ ] T041 Optimize performance for large text translations
- [ ] T042 [P] Address any edge cases identified during testing (API availability, multiple requests, etc.)

## Dependencies

### User Story Completion Order
1. User Story 3 (Backend Translation Endpoint) must be completed before User Story 1 (Frontend Button)
2. User Story 2 (Technical Terms Preservation) can be implemented in parallel with User Story 3
3. User Story 1 (Frontend Button) is dependent on both 2 and 3 being completed

### Task Dependencies
- T005, T006, T007 must be completed before T008
- T008 must be completed before T026
- T026 must be completed before T015
- T021 and T022 should be completed before T026

## Parallel Execution Examples

### Per User Story
- **User Story 1**: T011 and T013 can be executed in parallel
- **User Story 2**: T021 and T022 can be executed in parallel with other User Story 3 tasks
- **User Story 3**: T026 and T027 can be executed in parallel with T028

### Across Stories
- User Story 2 and User Story 3 can be developed in parallel after foundational tasks
- Styling tasks (T033, T034) can be done in parallel with frontend implementation tasks

## Acceptance Criteria

### User Story 1:
- [ ] User can click "Translate to Urdu" button
- [ ] Button shows "Translating..." state during processing
- [ ] Content is replaced with Urdu translation upon completion
- [ ] RTL styling is properly applied to Urdu content

### User Story 2:
- [ ] Technical terms (ROS 2, Gazebo, Isaac Sim, Python, Nodes, Topics) remain in English during translation
- [ ] Non-technical content is translated to Urdu

### User Story 3:
- [ ] POST /api/translate endpoint accepts text content
- [ ] Endpoint returns translated Urdu text with technical terms preserved
- [ ] Error handling is implemented for invalid requests