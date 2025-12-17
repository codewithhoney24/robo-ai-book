# Research: Urdu Translation Feature

## Decision: Docusaurus DocItem Swizzling
**Rationale**: Based on the feature specification, we need to modify the Docusaurus DocItem component to add the translation functionality. The standard approach in Docusaurus is to "swizzle" components, which means creating a local copy that overrides the default theme component.

**Technical Implementation**: Use the command `npm run swizzle @docusaurus/theme-classic DocItem -- --wrap` to create a custom wrapper for the DocItem component.

**Alternatives considered**: 
1. Modifying the theme directly - rejected because it would be overwritten during updates
2. Using a different component - rejected because DocItem is the correct component for document content

## Decision: FastAPI Implementation
**Rationale**: The backend will use FastAPI to create the translation endpoint. This follows the requirements and is consistent with the existing technology stack.

**Technical Implementation**: Create a new POST endpoint `/api/translate` that accepts text and returns the translated text.

**Alternatives considered**:
1. Using a different framework - rejected because FastAPI is already in the stack
2. Using a separate service - rejected because integration with existing backend is preferred

## Decision: OpenAI API Integration
**Rationale**: The system needs to use an LLM for translation with specific system prompt to preserve technical terms.

**Technical Implementation**: Use the existing OpenAI client configuration to call the GPT-4o-mini model (or available model) with the specified system prompt.

**Alternatives considered**:
1. Using a different translation service - rejected because OpenAI is specified in requirements
2. Training our own model - rejected because it's not feasible for this feature scope

## Decision: Frontend Translation Process
**Rationale**: The translation needs to happen client-side when the user clicks the button.

**Technical Implementation**: 
1. Add a "Translate to Urdu" button to the DocItem component
2. Extract the chapter content text when the button is clicked
3. Send the content to the `/api/translate` endpoint
4. Replace the original English content with the Urdu translation
5. Apply `dir="rtl"` attribute for proper Urdu text alignment

**Alternatives considered**:
1. Server-side translation - rejected because it would require page refresh
2. Pre-translated content - rejected because it doesn't meet real-time needs

## Decision: RTL Styling for Urdu
**Rationale**: Urdu is written right-to-left, so proper styling is essential for readability.

**Technical Implementation**: Apply the `dir="rtl"` attribute to the translated content container and potentially adjust CSS for proper RTL display.

**Alternatives considered**:
1. No RTL styling - rejected because it would make the text unreadable for Urdu speakers
2. Custom RTL components - rejected because the `dir` attribute is the standard approach