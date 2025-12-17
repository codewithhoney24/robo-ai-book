# Data Model: Urdu Translation Feature

## Entities

### Translation Request
**Description**: Represents a text translation request from the frontend to the backend service.

**Fields**:
- `text` (string, required): The English text content to be translated to Urdu
- `context` (string, optional): Additional context to help with translation (e.g., technical domain)

**Validation**:
- `text` must not be empty
- `text` length should not exceed 2000 words (approximately 15,000 characters)

### Translation Response
**Description**: Represents the translated text response from the backend service to the frontend.

**Fields**:
- `translated_text` (string, required): The Urdu translation of the input text
- `status` (string, optional): Status of the translation (e.g., "completed", "error")
- `error_message` (string, optional): Error details if translation failed

**Validation**:
- `translated_text` must not be empty when status is "completed"
- `error_message` must be present when status is "error"

### Chapter Content
**Description**: Represents the English text content of a textbook chapter that can be translated.

**Fields**:
- `content` (string, required): The raw text content of the chapter
- `title` (string, optional): The title of the chapter
- `section` (string, optional): The section or module the chapter belongs to
- `technical_terms` (array of strings): List of technical terms that should remain in English during translation

**Validation**:
- `content` must not be empty
- `technical_terms` should be preserved in the translated output