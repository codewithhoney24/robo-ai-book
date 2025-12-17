---
id: 1
title: ros2-course-content-planning
stage: plan
date: 2025-12-13
surface: agent
model: Qwen
feature: 001-ros2-course-content
branch: 001-ros2-course-content
user: user
command: /sp.plan
labels: [ros2, course, education, planning]
links:
  spec: D:\ai-book-pro\ai-robo-bk\specs\001-ros2-course-content\spec.md
  ticket: null
  adr: null
  pr: null
files:
 - D:\ai-book-pro\ai-robo-bk\specs\001-ros2-course-content\plan.md
 - D:\ai-book-pro\ai-robo-bk\specs\001-ros2-course-content\research.md
 - D:\ai-book-pro\ai-robo-bk\specs\001-ros2-course-content\data-model.md
 - D:\ai-book-pro\ai-robo-bk\specs\001-ros2-course-content\quickstart.md
 - D:\ai-book-pro\ai-robo-bk\specs\001-ros2-course-content\contracts\educational-content-api.yaml
tests:
 - D:\ai-book-pro\ai-robo-bk\specs\001-ros2-course-content\research.md
 - D:\ai-book-pro\ai-robo-bk\specs\001-ros2-course-content\data-model.md
 - D:\ai-book-pro\ai-robo-bk\specs\001-ros2-course-content\contracts\educational-content-api.yaml
---

## Prompt

```
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/setup-plan.ps1 -Json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType qwen`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

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

## Response snapshot

```
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/setup-plan.ps1 -Json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType qwen`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

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

/sp.plan                                                                       plan Advanced Course Content: Module 1 - The Robotic Nervous System (ROS 2)
Create: content architecture, chapter breakdown, learning path design, validation framework.
Architecture Sketch:
Module 1: The Robotic Nervous System (ROS 2)
├── Chapter 1: ROS 2 Architecture & Communication Patterns
│   ├── 1.1 Introduction to ROS 2 Middleware
│   ├── 1.2 Nodes: The Building Blocks
│   ├── 1.3 Topics: Pub-Sub Communication
│   ├── 1.4 Services: Request-Response Patterns
│   └── 1.5 Hands-on Lab: Multi-Node Robot System
├── Chapter 2: Bridging AI Agents to Robot Controllers
│   ├── 2.1 Understanding rclpy Interface
│   ├── 2.2 Agent-Controller Architecture Patterns
│   ├── 2.3 State Management & Feedback Loops
│   ├── 2.4 Error Handling & Recovery
│   └── 2.5 Hands-on Lab: AI-Driven Navigation Agent
└── Chapter 3: Humanoid Robot Representation with URDF
    ├── 3.1 URDF Syntax & Structure
    ├── 3.2 Links, Joints, and Kinematic Chains
    ├── 3.3 Humanoid-Specific Considerations
    ├── 3.4 Visualization & Validation Tools
    └── 3.5 Hands-on Lab: Building a Humanoid Model
Content Development Approach:

Use iterative-learning methodology (introduce → demonstrate → practice → validate)
Follow Docusaurus MDX format with interactive code components
Organize by phases: Concepts → Implementation → Integration → Validation

Decisions Needing Documentation:

ROS 2 Distribution Choice

Option A: ROS 2 Humble (LTS, stable, maximum compatibility)
Option B: ROS 2 Jazzy (latest features, cutting-edge)
Recommendation: Humble - prioritize stability for learners
Tradeoff: Some newer features unavailable, but production-ready


Code Example Complexity Level

Option A: Minimal examples (10-20 lines, core concepts only)
Option B: Production-ready examples (50-100 lines, error handling included)
Recommendation: Hybrid - minimal for concepts, expanded for labs
Tradeoff: Balance clarity vs real-world applicability


URDF Modeling Depth

Option A: Simplified humanoid (upper body only, 8-10 joints)
Option B: Full humanoid (complete skeleton, 25+ joints)
Recommendation: Simplified with extension exercises
Tradeoff: Faster learning curve vs comprehensive understanding


Simulation Environment

Option A: Gazebo Classic (widely used, stable)
Option B: Gazebo Ignition (modern, better performance)
Option C: Custom lightweight simulator
Recommendation: Gazebo Ignition with fallback notes to Classic
Tradeoff: Future-proof vs immediate compatibility



Testing Strategy (Validation Checks):
Based on acceptance criteria from /sp.specify:

Functional ROS 2 Node Creation:

✓ Learner creates 3+ working nodes from scratch
✓ Nodes successfully communicate via topics
✓ Service calls execute with proper response handling
Validation: Automated test scripts provided with expected outputs


AI-ROS Integration:

✓ Bidirectional data flow demonstrated (agent → controller → agent)
✓ Proper rclpy lifecycle management implemented
✓ Integration handles common failure modes gracefully
Validation: Checklist of integration patterns with example outputs


URDF Competency:

✓ Parse existing URDF files and identify components
✓ Modify joint parameters and verify kinematic changes
✓ Create valid URDF for 5+ joint structure
Validation: URDF validator tool checks + visualization screenshots


Hands-on Exercise Completion:

✓ Each of 3 chapters has 2+ exercises with solution files
✓ Exercises build progressively in complexity
✓ Clear success criteria for each exercise
Validation: Reference implementation + common error troubleshooting guide


Conceptual Understanding:

✓ End-of-chapter quiz covers key architectural concepts
✓ Learner can diagram ROS 2 communication flow
✓ Learner articulates ROS 1 vs ROS 2 differences
Validation: Knowledge check questions with detailed answer keys



Technical Details for Docusaurus Implementation:

File Structure:

  docs/
  ├── module-1/
  │   ├── intro.mdx
  │   ├── chapter-1/
  │   │   ├── 01-ros2-intro.mdx
  │   │   ├── 02-nodes.mdx
  │   │   ├── 03-topics.mdx
  │   │   ├── 04-services.mdx
  │   │   └── 05-lab.mdx
  │   ├── chapter-2/
  │   │   └── [similar structure]
  │   └── chapter-3/
  │       └── [similar structure]
  ├── code-examples/
  │   └── module-1/
  │       ├── chapter-1/
  │       ├── chapter-2/
  │       └── chapter-3/
  └── assets/
      └── diagrams/

Interactive Components:

Use MDX for embedded React components (live code editors if feasible)
Mermaid diagrams for architecture visualization
Collapsible code blocks for solution reveals
Admonitions (:::tip, :::warning) for key insights


Code Standards:

All Python code follows PEP 8
Inline comments explain ROS 2-specific patterns
Each code block includes language tag and filename context
Example: ```python title="src/my_node.py"


Quality Gates:

Technical review: ROS 2 expert validation of code accuracy
Pedagogical review: Learning progression and clarity
User testing: 3+ learners complete exercises with feedback
Code testing: All examples run successfully on Ubuntu 22.04 + ROS 2 Humble



Content Development Timeline:

Week 1: Chapter 1 draft + code examples + exercises
Week 2: Chapter 2 draft + integration examples + exercises
Week 3: Chapter 3 draft + URDF examples + exercises
Week 4: Review, testing, refinement, validation checklist completion

Success Metrics:

90%+ of code examples run without modification
Learners complete module in 45-60 minutes per chapter
Post-module quiz average score: 80%+
Learner feedback: "Clear path from concepts to implementation"

## Outcome

- ✅ Impact: Full implementation plan created for ROS 2 course content, including research, data models, API contracts, quickstart guide, and agent context update
- 🧪 Tests: All deliverables validated against constitutional principles and feature requirements
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, contracts/educational-content-api.yaml created
- 🔁 Next prompts: Content development, implementation of individual chapters, testing and validation
- 🧠 Reflection: Implementation planning workflow executed successfully, all constitutional gates passed

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All constitutional principles PASS
- Prompt variant (if applicable): Standard /sp.plan workflow
- Next experiment (smallest change to try): Begin content development for Chapter 1