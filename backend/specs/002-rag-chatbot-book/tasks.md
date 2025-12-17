# Tasks: Integrated RAG Chatbot for Published Book

**Input**: Design documents from `/specs/002-rag-chatbot-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are included as per functional requirements in spec.md (unit tests for RAG flow, API endpoints, database interactions, and vector search)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `src/`, `tests/` at repository root
- Paths shown below follow the structure from plan.md

<!--
  ============================================================================
  ACTUAL TASKS based on design documents from specs/002-rag-chatbot-book/
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure per implementation plan in backend/src/
- [x] T002 Initialize Python 3.11 project with FastAPI, qdrant-client, psycopg2, cohere dependencies in backend/
- [x] T003 [P] Configure linting and formatting tools (flake8, black, mypy) in backend/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Setup configuration management with environment variables in backend/src/config/settings.py
- [x] T005 [P] Configure Qdrant connection settings in backend/src/config/database_config.py
- [x] T006 [P] Configure Neon Postgres connection settings in backend/src/config/database_config.py
- [x] T007 Create base models that all stories depend on in backend/src/models/
- [x] T008 Setup API routing and middleware structure in backend/src/api/main.py
- [x] T009 Implement logging infrastructure in backend/src/config/logging_config.py
- [x] T010 Create error handling framework in backend/src/exceptions/
- [x] T011 [P] Setup Cohere API integration in backend/src/services/embedding_service.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Query Book Content via Chatbot (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about the book content and receive accurate responses with 95%+ relevance

**Independent Test**: The feature can be tested by asking specific questions about the book content and verifying that the chatbot provides accurate, relevant responses based on the book's information.

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [x] T012 [P] [US1] Contract test for /chat endpoint in backend/tests/contract/test_chatbot_api.py
- [x] T013 [P] [US1] Unit test for RAG flow in backend/tests/unit/test_rag_flow.py
- [x] T014 [P] [US1] Unit test for vector search functionality in backend/tests/unit/test_vector_search.py
- [x] T015 [P] [US1] Integration test for chatbot API in backend/tests/integration/test_chatbot_integration.py

### Implementation for User Story 1

- [x] T016 [P] [US1] Create UserQuery model in backend/src/models/user_query.py
- [x] T017 [P] [US1] Create RetrievedContext model in backend/src/models/retrieved_context.py
- [x] T018 [P] [US1] Create GeneratedResponse model in backend/src/models/generated_response.py
- [x] T019 [US1] Implement VectorStoreService for Qdrant integration in backend/src/services/vector_store_service.py
- [x] T020 [US1] Implement DatabaseService for Neon Postgres operations in backend/src/services/database_service.py
- [x] T021 [US1] Implement RAGService with retrieval and generation logic in backend/src/services/rag_service.py
- [x] T022 [US1] Implement /chat endpoint in backend/src/api/chatbot_router.py
- [x] T023 [US1] Add validation and error handling for /chat endpoint
- [x] T024 [US1] Add logging for user story 1 operations
- [x] T025 [US1] Implement book content indexing functionality in backend/src/services/book_content_service.py
- [x] T026 [US1] Set up health check endpoint in backend/src/api/main.py

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Interact with Chatbot via Web Interface (Priority: P2)

**Goal**: Enable web-based interaction with the RAG chatbot that works across different browsers and platforms without performance issues

**Independent Test**: The feature can be tested by accessing the book in a web browser, using the embedded chatbot interface to ask questions, and verifying the responses work across different browsers and platforms.

### Tests for User Story 2

- [x] T027 [P] [US2] Unit test for web interface integration in backend/tests/unit/test_web_interface.py
- [x] T028 [P] [US2] Integration test for cross-platform compatibility in backend/tests/integration/test_web_compatibility.py
- [x] T029 [P] [US2] Performance test for response time in backend/tests/integration/test_performance.py
- [x] T030 [P] [US2] Contract test for web-specific endpoints in backend/tests/contract/test_web_endpoints.py

### Implementation for User Story 2

- [x] T031 [US2] Create BookContent model in backend/src/models/book_content.py
- [x] T032 [US2] Implement book content retrieval in backend/src/services/book_content_service.py
- [x] T033 [US2] Implement session management for web users in backend/src/services/session_service.py
- [x] T034 [US2] Add /book-content endpoints in backend/src/api/book_content_router.py
- [x] T035 [US2] Update chatbot API to handle session data in backend/src/api/chatbot_router.py
- [x] T036 [US2] Implement performance optimization for web interface in backend/src/services/optimization_service.py
- [x] T037 [US2] Add cross-platform compatibility checks in backend/src/api/middleware/compatibility_check.py
- [x] T038 [US2] Update documentation for web integration in backend/docs/web_integration.md

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Process User-Selected Text in Queries (Priority: P3)

**Goal**: Allow users to select specific text in the book and ask questions about that text to get more targeted information

**Independent Test**: The feature can be tested by selecting specific text in the book interface, submitting questions about that text, and verifying that the responses are contextually relevant to the selected content.

### Tests for User Story 3

- [x] T039 [P] [US3] Contract test for /chat/selected-text endpoint in backend/tests/contract/test_selected_text_api.py
- [x] T040 [P] [US3] Unit test for selected text processing in backend/tests/unit/test_selected_text.py
- [x] T041 [P] [US3] Integration test for selected text queries in backend/tests/integration/test_selected_text_integration.py

### Implementation for User Story 3

- [x] T042 [US3] Update UserQuery model to include optional selected text in backend/src/models/user_query.py
- [x] T043 [US3] Implement selected text processing in backend/src/services/rag_service.py
- [x] T044 [US3] Create /chat/selected-text endpoint in backend/src/api/chatbot_router.py
- [x] T045 [US3] Add validation and error handling for selected text in backend/src/api/chatbot_router.py
- [x] T046 [US3] Update embedding service to handle selected text in backend/src/services/embedding_service.py
- [x] T047 [US3] Update vector store service to prioritize selected text in backend/src/services/vector_store_service.py
- [x] T048 [US3] Add logging for user story 3 operations
- [x] T049 [US3] Update documentation for selected text feature in backend/docs/selected_text.md

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T050 [P] Documentation updates in backend/README.md
- [x] T051 [P] API documentation updates in backend/docs/api.md
- [x] T052 [P] Update quickstart guide with implementation details in backend/docs/quickstart.md
- [x] T053 Code cleanup and refactoring across all services
- [x] T054 Performance optimization across all stories
- [x] T055 [P] Additional unit tests to achieve 80%+ coverage in backend/tests/unit/
- [x] T056 Security hardening for API endpoints
- [x] T057 GDPR compliance verification for session data handling
- [x] T058 Run quickstart.md validation to ensure all steps work
- [x] T059 Final integration tests for all user stories working together
- [x] T060 Update OpenAPI specification with implementation details in backend/specs/002-rag-chatbot-book/contracts/openapi.yaml

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
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together:
Task: "Contract test for /chat endpoint in backend/tests/contract/test_chatbot_api.py"
Task: "Unit test for RAG flow in backend/tests/unit/test_rag_flow.py"
Task: "Unit test for vector search functionality in backend/tests/unit/test_vector_search.py"
Task: "Integration test for chatbot API in backend/tests/integration/test_chatbot_integration.py"

# Launch all models for User Story 1 together:
Task: "Create UserQuery model in backend/src/models/user_query.py"
Task: "Create RetrievedContext model in backend/src/models/retrieved_context.py"
Task: "Create GeneratedResponse model in backend/src/models/generated_response.py"
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
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence