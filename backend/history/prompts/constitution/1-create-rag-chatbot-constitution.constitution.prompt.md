---
id: 1
title: Create RAG Chatbot Constitution
stage: constitution
date: 2025-12-14
surface: agent
model: Qwen
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution,rag,chatbot,principles]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - history/prompts/constitution/1-create-rag-chatbot-constitution.constitution.prompt.md
tests:
 - none
---

## Prompt

```
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.constitution
Project: Integrated RAG Chatbot Development for a Published Book
Core principles:

Functionality focused on accurate retrieval and generation based on book content
User-centric design for seamless integration and interaction within the book
Scalability and efficiency using serverless and free-tier services
Security and privacy in handling user-selected text and queries
Modularity for easy maintenance and updates

Key standards:

Use specified technologies: OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier (adapt to SpecifyKit Plus and Qwen CLI for enhanced implementation)
RAG implementation must support context-aware responses from full book or user-selected text
API endpoints must be robust, with proper error handling and authentication
Database schema optimized for vector storage and quick retrieval
Code quality: Follow PEP 8 for Python, include unit tests covering 80%+ code
Documentation: Comprehensive README, API docs via Swagger, and inline comments

Constraints:

Deployment: Embeddable within a published book format (e.g., web-based or interactive PDF)
Budget: Rely on free tiers (Qdrant Cloud Free, Neon Serverless); no paid upgrades unless justified
Timeline: Assume iterative development with milestones for setup, integration, testing
Compatibility: Cross-platform support for web browsers and mobile if applicable
Data handling: No storage of user data beyond session; comply with GDPR-like principles

Success criteria:

Chatbot accurately answers 95%+ of queries based on book content or selected text
Successful embedding and functionality within the book without performance lags
Passes integration tests for all components (RAG flow, API, DB, vector search)
Positive user feedback on usability and response relevance
Zero critical bugs in production simulation

## Response snapshot

The project constitution has been successfully created with the following details:

**New Version**: 1.0.0 (Initial version for this project)
**Bump Rationale**: This is the first version of the constitution for the Integrated RAG Chatbot Development for a Published Book project, establishing the foundational principles and governance.

**Files Updated**:
- `.specify/memory/constitution.md` has been created with all principles and governance details

**Files Flagged for Manual Follow-up**:
- No files require immediate manual follow-up as all templates were reviewed and no specific constitution-related changes were required in dependent templates.

**Suggested Commit Message**: 
`docs: create constitution for RAG chatbot project (principles and governance established)`

The constitution includes 6 core principles based on your input:
1. Functionality Focused Accuracy
2. User-Centric Design
3. Scalability and Efficiency
4. Security and Privacy
5. Modularity and Maintainability
6. Technology Stack Compliance

It also includes sections on Additional Constraints, Development Workflow and Quality Standards, and Governance with all required compliance and testing guidelines.

## Outcome

- ✅ Impact: Created the project constitution file with 6 core principles and governance rules
- 🧪 Tests: No tests required for this documentation task
- 📁 Files: Created .specify/memory/constitution.md and this PHR file
- 🔁 Next prompts: User may want to create specs, plans, or tasks based on these principles
- 🧠 Reflection: The constitution provides a solid foundation for the RAG chatbot project, ensuring development stays aligned with core principles

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Not applicable