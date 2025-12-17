---
id: 7
title: urdu-translation-feature-spec
stage: spec
date: 2025-12-15
surface: agent
model: Qwen
feature: urdu-translation-feature
branch: 006-urdu-translation-feature
user: user
command: sp.specify
labels: ["specification", "translation", "urdu", "docusaurus", "fastapi"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-urdu-translation-feature/spec.md
  - specs/006-urdu-translation-feature/checklists/requirements.md
tests: []
---

# Urdu Translation Feature Specification

## Prompt Text

Create or update the feature specification from a natural language feature description.

User description:
```
Urdu Translation Feature with Docusaurus & FastAPI ## Context We are building a technical textbook. We need a feature where users can click a button to translate the current chapter into Urdu using an LLM. ## Requirements ### 1. Backend (FastAPI) - **File:** `app/server.py` (or wherever your main FastAPI app is). - **Action:** Create a new POST endpoint `/api/translate`. - **Logic:** - Input: `{ "text": "string" }` - Process: Send the text to OpenAI GPT-4o-mini (or available model). - **System Prompt:** "You are a technical translator. Translate the text to Urdu. Keep technical terms (ROS 2, Gazebo, Isaac Sim, Python, Nodes, Topics) in English. Output only the translated text." - Output: `{ "translated_text": "string" }` ### 2. Frontend (Docusaurus) - **Action:** Swizzle the `DocItem` component to wrap it. - Command: `npm run swizzle @docusaurus/theme-classic DocItem -- --wrap` - **File:** `src/theme/DocItem/index.js` (created after swizzle). - **UI Changes:** - Add a button "Translate to Urdu" at the top of the chapter. - When clicked: 1. Show "Translating..." state. 2. Send chapter content (props.children text) to `/api/translate`. 3. Replace the English content with the Urdu response. 4. Apply `dir="rtl"` styling to the Urdu text for correct alignment. ## Technical Constraints - Use existing OpenAI client configuration. - Ensure the button is styled nicely (e.g., using existing CSS classes).
```

## Response Text

Created a comprehensive feature specification for the Urdu translation feature with Docusaurus and FastAPI. The specification includes:

- User scenarios and testing requirements
- Functional requirements
- Key entities
- Success criteria

The specification was created following the template structure and focusing on user value without implementation details. A quality checklist was also created and validated.

## Outcome

Successfully created the feature specification document for the Urdu translation feature in the file `specs/006-urdu-translation-feature/spec.md` with a quality checklist in `specs/006-urdu-translation-feature/checklists/requirements.md`. The specification is ready for planning and implementation.