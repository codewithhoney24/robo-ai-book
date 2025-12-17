# Implementation Plan: Integrated RAG Chatbot for Published Book

**Branch**: `002-rag-chatbot-book` | **Date**: 2025-12-14 | **Spec**: [link to spec.md](./spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build and embed a Retrieval-Augmented Generation (RAG) chatbot within a published book using SpecifyKit Plus and Qwen CLI, integrated with Qdrant and Neon Postgres. The chatbot answers questions on book content, including user-selected text, with 95%+ relevance.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: SpecifyKit Plus, Qwen CLI, FastAPI, qdrant-client, psycopg2, cohere
**Storage**: Qdrant vector store, Neon Postgres
**Testing**: pytest
**Target Platform**: Linux server (web-based book integration)
**Project Type**: Web application
**Performance Goals**: 95%+ relevance for RAG responses, sub-second response time
**Constraints**: Free tier usage for Qdrant and Neon, GDPR compliance for user session data
**Scale/Scope**: Designed for book readership (hundreds to thousands of concurrent users during active reading sessions)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution, the following checks have been performed:
- Project starts as a standalone service that can function independently
- API follows text-in/text-out protocol via JSON over HTTP
- Test-first approach will be used with TDD principles
- Focus on integration testing for the RAG pipeline and external service interactions
- Adequate observability will be implemented with structured logging
- Simplicity principle followed - started with minimal viable implementation

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── openapi.yaml     # OpenAPI specification for chatbot API
├── checklists/
│   └── requirements.md  # Quality checklist for specification
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── user_query.py
│   │   ├── retrieved_context.py
│   │   ├── generated_response.py
│   │   └── book_content.py
│   ├── services/
│   │   ├── rag_service.py
│   │   ├── vector_store_service.py
│   │   ├── database_service.py
│   │   └── embedding_service.py
│   ├── api/
│   │   ├── main.py
│   │   ├── chatbot_router.py
│   │   └── book_content_router.py
│   └── config/
│       ├── settings.py
│       └── database_config.py
└── tests/
    ├── contract/
    ├── integration/
    └── unit/
```

**Structure Decision**: Web application structure chosen to support embedding in web-based book formats with a FastAPI backend to handle RAG operations.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |