# Feature Specification: Urdu Translation Feature

**Feature Branch**: `006-urdu-translation-feature`
**Created**: 2025-12-15
**Status**: Draft
**Input**: User description: "Urdu Translation Feature with Docusaurus & FastAPI ## Context We are building a technical textbook. We need a feature where users can click a button to translate the current chapter into Urdu using an LLM. ## Requirements ### 1. Backend (FastAPI) - **File:** `app/server.py` (or wherever your main FastAPI app is). - **Action:** Create a new POST endpoint `/api/translate`. - **Logic:** - Input: `{ \"text\": \"string\" }` - Process: Send the text to OpenAI GPT-4o-mini (or available model). - **System Prompt:** \"You are a technical translator. Translate the text to Urdu. Keep technical terms (ROS 2, Gazebo, Isaac Sim, Python, Nodes, Topics) in English. Output only the translated text.\" - Output: `{ \"translated_text\": \"string\" }` ### 2. Frontend (Docusaurus) - **Action:** Swizzle the `DocItem` component to wrap it. - Command: `npm run swizzle @docusaurus/theme-classic DocItem -- --wrap` - **File:** `src/theme/DocItem/index.js` (created after swizzle). - **UI Changes:** - Add a button \"Translate to Urdu\" at the top of the chapter. - When clicked: 1. Show \"Translating...\" state. 2. Send chapter content (props.children text) to `/api/translate`. 3. Replace the English content with the Urdu response. 4. Apply `dir=\"rtl\"` styling to the Urdu text for correct alignment. ## Technical Constraints - Use existing OpenAI client configuration. - Ensure the button is styled nicely (e.g., using existing CSS classes)."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Urdu Translation Button (Priority: P1)

As a reader of the technical textbook, I want to click a button to translate the current chapter content into Urdu so that I can better understand the material in my native language.

**Why this priority**: This is the core functionality of the feature and provides immediate value to Urdu-speaking users.

**Independent Test**: The feature can be fully tested by clicking the "Translate to Urdu" button and verifying that the content is translated while preserving technical terms in English, then ensuring the text is displayed with right-to-left alignment.

**Acceptance Scenarios**:

1. **Given** I am viewing a chapter in English, **When** I click the "Translate to Urdu" button, **Then** I see an indicator that translation is in progress and the content is replaced with Urdu translation
2. **Given** I have clicked the "Translate to Urdu" button, **When** the translation is complete, **Then** the translated text is displayed with proper right-to-left alignment and technical terms remain in English
3. **Given** I have translated content in Urdu, **When** I refresh the page, **Then** I see the original English content (translation is client-side only)

---

### User Story 2 - Technical Terms Preservation (Priority: P1)

As a technical reader, I want technical terms (ROS 2, Gazebo, Isaac Sim, Python, Nodes, Topics) to remain in English during translation so that I can maintain consistency with the rest of the technical ecosystem.

**Why this priority**: Maintaining technical terminology consistency is crucial for understanding and referencing documentation in the broader technical community.

**Independent Test**: The system can be tested by translating content containing technical terms and verifying they remain in English in the Urdu output.

**Acceptance Scenarios**:

1. **Given** a chapter contains technical terms like "ROS 2" and "Python", **When** I translate to Urdu, **Then** those terms remain in English in the translated output
2. **Given** I am viewing translated content, **When** I look for technical terms, **Then** they appear in English while the rest of the text is in Urdu

---

### User Story 3 - Right-to-Left Text Display (Priority: P2)

As a Urdu reader, I want the translated text to be displayed with proper right-to-left alignment so that it's readable according to Urdu writing conventions.

**Why this priority**: Proper text alignment is essential for readability and user experience in RTL languages.

**Independent Test**: Can be tested by verifying that translated Urdu content has correct right-to-left display styling.

**Acceptance Scenarios**:

1. **Given** content has been translated to Urdu, **When** I view the page, **Then** the text flows from right to left with proper alignment
2. **Given** I am viewing Urdu content, **When** I view the text, **Then** it displays with appropriate right-to-left styling

---

### Edge Cases

- What happens when the translation service is temporarily unavailable?
- How does the system handle very large chapters that might exceed service limits?
- What if the user clicks the translate button multiple times before the first translation completes?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a service endpoint that accepts text input and returns translated Urdu text
- **FR-002**: System MUST preserve technical terms (ROS 2, Gazebo, Isaac Sim, Python, Nodes, Topics) in English during translation
- **FR-003**: System MUST use an LLM with a technical translator prompt for translation
- **FR-004**: User interface MUST display a "Translate to Urdu" button at the top of each chapter
- **FR-005**: User interface MUST show an appropriate indicator while translation is in progress
- **FR-006**: User interface MUST replace English content with the translated Urdu response upon completion
- **FR-007**: User interface MUST apply right-to-left styling to Urdu content for proper text alignment
- **FR-008**: System MUST send chapter content to the translation service when the button is clicked
- **FR-009**: System MUST handle service errors gracefully and display appropriate user feedback
- **FR-010**: System MUST prevent multiple concurrent translation requests from the same user

### Key Entities

- **Translation Request**: Contains text to be translated (string)
- **Translation Response**: Contains the translated text in Urdu (string)
- **Chapter Content**: The English text content of a textbook chapter that can be translated

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can translate textbook chapters to Urdu with preserved technical terms in under 10 seconds (for chapters up to 2000 words)
- **SC-002**: 95% of translated chapters display with proper right-to-left alignment and readability
- **SC-003**: Translation preserves at least 95% of technical terms in English while successfully translating the remaining text to Urdu
- **SC-004**: User satisfaction rating for the translation feature is above 4.0/5.0 based on user feedback