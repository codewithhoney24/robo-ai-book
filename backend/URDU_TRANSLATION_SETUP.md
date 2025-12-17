# Urdu Translation Feature Setup

This document explains how to set up and use the Urdu translation feature in the backend.

## Prerequisites

1. An OpenAI API key to enable translation functionality.

## Setup

1. Update your `.env` file with your OpenAI API key:
   ```bash
   OPENAI_API_KEY=your_openai_api_key_here
   ```

2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Make sure all dependencies are installed:
   ```bash
   pip install openai
   ```

## API Endpoint

The translation functionality is available at the following endpoint:

- **POST** `/api/translate`
- **Request body**: `{"text": "your English text here"}`
- **Response**: `{"translated_text": "your Urdu translation here"}`

## How It Works

1. The system receives English text from the frontend.
2. It sends the text to the OpenAI GPT-4o-mini model with specific instructions to:
   - Translate the text to Urdu
   - Preserve technical terms (like ROS 2, Gazebo, Isaac Sim, Python, Nodes, Topics) in English
   - Output only the translated text
3. The translated text is returned to the frontend where it's displayed with right-to-left (RTL) styling.

## Testing the Translation Service

You can test the translation functionality using the test script:

```bash
python test_urdu_translation.py
```

This script will:
- Create a TranslationService instance
- Pass sample English text
- Display the translated Urdu text
- Verify that the translation contains Urdu characters

## Error Handling

The translation service includes error handling for:
- Empty or invalid input text
- API connection failures
- Invalid API keys
- Rate limiting

Error responses will include appropriate HTTP status codes and descriptive messages.

## Configuration

The translation service can be configured in `src/config/settings.py`:

- `OPENAI_API_KEY`: Your OpenAI API key
- Various other application settings